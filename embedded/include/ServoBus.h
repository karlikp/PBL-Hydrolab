// ServoBus — driver for the Waveshare SC-09 / Feetech SCS bus.
//
// Three smart servos (one per tank) sit on a single half-duplex UART
// bus. The board uses a non-standard ESP32-S3 UART0 pin mapping for
// this bus: TX = GPIO 44, RX = GPIO 43 (the opposite of the chip's
// default). See docs/hardware/servo_uart_pinout.md.
//
// This class wraps Waveshare's official SCServo library so callers get
// a deliberately small API (move() / move_all() / ping()) instead of
// the library's full surface area. Direction control on the half-
// duplex bus is handled by the board's analog circuitry — firmware
// just writes UART bytes.
//
// SCServo.h is intentionally NOT included here: TUs that only call
// the ServoBus API (e.g. MissionControl) shouldn't have to pull in the
// SCServo dependency. The implementation hides it behind PImpl.

#pragma once

#include <stdint.h>
#include <stddef.h>

class HardwareSerial;

class ServoBus {
public:
    static constexpr uint8_t  BROADCAST_ID = 0xFE;
    static constexpr uint32_t DEFAULT_BAUD = 1000000;  // SC-09 factory default

    // SC-09 bus UART pins on the production board. Anomalous vs. the
    // ESP32-S3 datasheet defaults — TX/RX are swapped. Do NOT change
    // without re-reading docs/hardware/servo_uart_pinout.md.
    //
    // Confirmed empirically (2026-06-22):
    //   pin 44 = bus TX direction (writes here move the motor)
    //   pin 43 = bus RX direction (TX echo arrives here)
    //   pin 44 as RX = sees nothing (the chip can't read its own TX
    //                  pin, and the board's RX path goes only to pin 43)
    static constexpr int TX_PIN = 44;
    static constexpr int RX_PIN = 43;

    // The caller hands in a HardwareSerial that has already been
    // begin()ed at the right baud with the swapped RX/TX pins.
    explicit ServoBus(HardwareSerial& serial);
    ~ServoBus();

    ServoBus(const ServoBus&)            = delete;
    ServoBus& operator=(const ServoBus&) = delete;

    // Send a goal-position WRITE to one servo (or BROADCAST_ID).
    //   position : 0..1023 (clipped)
    //   speed    : 0..1023 step rate; 1500 is the SCServo lib default
    // Best-effort — broadcast writes don't get a reply.
    bool move(uint8_t id, uint16_t position, uint16_t speed = 1500);

    // ---- Wheel / PWM mode ----
    //
    // The SC-09 has a hard ~300° single-turn range in position mode. For
    // applications that need multiple revolutions (winches that unspool
    // line longer than one rotation can pull), the servo can be switched
    // into PWM/wheel mode: the position-control loop is disabled and the
    // motor spins continuously, driven by a signed duty-cycle value.
    //
    // set_pwm_mode writes 0/0 into the EEPROM angle-limit registers,
    // which disables the angle limiter and turns the servo into a
    // continuous-rotation drive. NOTE: this is an EEPROM write — it
    // persists across power cycles. To revert, call set_position_mode.
    //
    // write_pwm drives the motor: pwm > 0 spins one direction, pwm < 0
    // spins the other, pwm == 0 coasts (no holding torque). Magnitude
    // 0..1023 controls duty.
    bool set_pwm_mode(uint8_t id);
    bool set_position_mode(uint8_t id);  // revert: angle limits 0..1023
    bool write_pwm(uint8_t id, int16_t pwm);
    bool stop_pwm(uint8_t id) { return write_pwm(id, 0); }

    // Read the servo's current present position (register
    // SCSCL_PRESENT_POSITION). Returns 0..1023 on success, -1 on
    // failure / timeout / no servo at this ID.
    //
    // CAVEAT — same half-duplex echo problem the existing ping()
    // suffers from. The 8-byte read request echoes back on RX before
    // the servo's reply arrives, and the SCServo library doesn't
    // drain the echo, so the return can be unreliable. Treat -1 OR
    // an obviously-bad value (e.g. > 1023) as "couldn't determine."
    // Callers should fall back to safe behaviour when this fails.
    int read_position(uint8_t id);

    // Broadcast move — every servo on the bus.
    bool move_all(uint16_t position, uint16_t speed = 1500) {
        return move(BROADCAST_ID, position, speed);
    }

    // PING — sends an SCS PING instruction and reads back.
    //
    // CAVEAT: the bus is half-duplex on a single wire, and the
    // SCServo lib does not drain the TX echo on RX before reading
    // the reply. The echo of a PING packet happens to match the
    // shape of a successful status reply, so this function returns
    // the queried `id` even when NO servo at that ID exists. In
    // other words: it cannot reliably distinguish "servo present"
    // from "echoing my own packet."
    //
    // For ground-truth bus state, send a MOVE to the suspected ID
    // and watch which servo (if any) physically moves. A proper
    // fix is to wrap the underlying Stream and discard N echoed
    // bytes after every TX before Ack reads — not done yet.
    int ping(uint8_t id);

    // Re-assign a servo's ID. Sequence: unlock EEPROM on the current
    // ID, write the new ID byte at address SCSCL_ID, lock EEPROM on
    // the new ID. Returns true if all three steps reported success.
    //
    // Requires exactly ONE servo on the bus to have current_id —
    // otherwise the simultaneous replies collide and the library
    // reports failure (so the write is skipped).
    bool set_id(uint8_t current_id, uint8_t new_id);

    // Broadcast variant: rewrite ID register on EVERY servo on the
    // bus to new_id. Uses ID 0xFE for unlock/write/lock — servos
    // execute but never reply, so collision is irrelevant. Always
    // returns true (the underlying broadcast can't be acked).
    //
    // Useful when you can't physically isolate one servo at a time
    // but can selectively unplug enough of them to set up a state
    // where a targeted set_id() can then pick off a single survivor.
    bool broadcast_set_id(uint8_t new_id);

    // Low-level diagnostic: send a Ping packet (6 bytes) for `id` and
    // capture up to `out_max` bytes received over the next `wait_ms`
    // milliseconds. Bypasses the SCServo library entirely. Returns
    // the number of bytes captured (≤ out_max). Used to verify what's
    // actually on the bus when the lib's high-level reads (ReadPos,
    // Ping) come back broken.
    //
    // Expected wire pattern for a healthy bus:
    //   echo of ping req (6 bytes) + servo reply (6 bytes) = 12 bytes
    //   ff ff <id> 02 01 <cksum>  ff ff <id> 02 <err> <cksum>
    // Just echo (6 bytes) → servo isn't replying.
    // Zero bytes → bus is electrically dead from the ESP's POV.
    size_t raw_ping_capture(uint8_t id, uint8_t* out, size_t out_max,
                            uint32_t wait_ms);

    // Write `level` to the Status Return Level register (EEPROM reg 8
    // on Feetech SCS-family servos). Wrapped with unlock/lock and
    // commit delays. Persists across power cycles.
    //   0 = never reply (factory default on some units — explains the
    //       silent bus syndrome we see)
    //   1 = reply only to READ_DATA instructions
    //   2 = reply on all instructions (what we want for diagnostics)
    bool set_status_return_level(uint8_t id, uint8_t level);

    // Write `value` to the Return Delay register (EEPROM reg 7 on
    // Feetech SCS-family). Encoding is value × 2 µs of delay before
    // the servo starts transmitting its reply. Used to push the reply
    // latency far enough out that the board's auto-direction BJT has
    // settled into RX mode before the first reply byte arrives.
    //   value 0   → ~0 µs (servo replies immediately — default, problematic)
    //   value 125 → ~250 µs
    //   value 250 → ~500 µs (safe margin for any TX→RX switching delay)
    bool set_return_delay(uint8_t id, uint8_t value);

private:
    struct Impl;
    Impl* impl_;
};
