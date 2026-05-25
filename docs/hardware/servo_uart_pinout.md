# Servo UART pinout — production ESP32-S3 board

## Summary

The SC-09 / Feetech SCS half-duplex servo bus must be driven from
UART0 with **TX = GPIO 44, RX = GPIO 43** — the *opposite* of the
ESP32-S3 datasheet default (TX = 43, RX = 44).

This was discovered empirically on 2026-05-25 after the standard pin
mapping completely failed to elicit any servo response, while the
swapped mapping made servos move on the first attempt.

## Working configuration

```cpp
// Arduino-ESP32 v3.x: begin(baud, cfg, RX, TX)
Serial.begin(1'000'000, SERIAL_8N1, /*RX=*/43, /*TX=*/44);
```

- Baud: **1 000 000** (SC-09 factory default)
- Library: Waveshare official SCServo (`SCSCL` class)
- Servo factory ID: **1** (all fresh-from-factory servos respond to ID
  1 — unique per-tank IDs need to be programmed later)
- Power latch (GPIO 4) must be driven HIGH at boot — see
  [`battery_divider_issue.md`](battery_divider_issue.md) context.

## Why the swap

The schematic labels the WROOM-1 module's pins 36 (RXD0) and 37 (TXD0)
as standard UART0. Between those pins and the `SERVO_HALF` wire there
is a small half-duplex circuit:

- U4, U5 — SN74LVC1G126 tri-state buffers (one for each direction)
- Q9 — S8550 PNP BJT, driving the buffers' `OE` inputs
- TXEN — output of the BJT's collector, *not* connected to any GPIO

That circuit gates buffer direction automatically using BJT-based
detection of UART idle / transmit state. The net effect: the chip-side
TX has to come out on the pin that the board's PCB layout actually
wires to the *driver-input* side of the half-duplex circuit, which
turns out to be GPIO 44, not GPIO 43.

Firmware therefore does **not** drive any GPIO for direction; it just
writes UART bytes and the board handles direction switching.

## UART0 ownership in deployment

In deployment, **UART0 is dedicated to the servo bus**. All operator
communication goes through the RFD868 radio on its own UART (see
[`radio_uart_pinout.md`](radio_uart_pinout.md)). The CP210x adapter on
the 4-pin UART0 header is a dev/programmer port only — it is not the
deployment comms channel.

During development:
- For protocol work without servos, use whatever env doesn't begin
  UART0 with the swapped mapping, or use the radio path with a paired
  module.
- For servo work, the `[env:esp32s3wroom1-servotest]` build dedicates
  UART0 to servos and disables every other UART0 consumer. Debug via
  JTAG in that mode.

## Half-duplex echo gotcha

The SC-09 bus is half-duplex on a single signal wire (`SERVO_HALF`).
The board's Q9/U4/U5 auto-direction circuit handles TX/RX switching
in hardware, but the ESP still sees every byte it transmits looped
back on its UART RX. The Waveshare SCServo library does **not**
drain this echo before calling `Ack()` to read the expected reply.

Two consequences:

1. **`Ping()` returns false positives.** Echo of a 6-byte PING packet
   has the same length as a valid 6-byte status reply, so the lib
   parses the echo as success and reports "got reply from ID=N" for
   whatever ID was queried — including IDs no servo is at. **Do not
   use `ping()` to determine ground-truth bus state.** Send a `MOVE`
   to the suspected ID and watch the servos.

2. **`writeByte()`-based ops report spurious failure.** Echo of an
   8-byte write packet is 8 bytes; the lib reads the first 6, sees
   the LEN field mismatch (`0x04` vs the `0x02` of a real status
   reply), and `Ack()` returns failure even though the underlying
   write was sent and the servo executed it. So the SET_ID flow
   cannot trust `unLockEprom`'s/`writeByte`'s/`LockEprom`'s return
   codes — `ServoBus::set_id` and `broadcast_set_id` deliberately
   ignore them and just blast the three packets with delays.

A proper fix would wrap the underlying `HardwareSerial` and discard
N echoed bytes after every TX, before any `Ack()` read. Not done
yet — works around it with comments and behavioural notes for now.

## EEPROM commit timing

SC-09 EEPROM writes take a few ms internally to commit. If the next
SCS packet hits the bus before the previous write has finished, the
servo can drop one or both. `ServoBus::set_id` and `broadcast_set_id`
insert a 20 ms delay between unlock → write → lock so the writes
actually land.

## Programming unique servo IDs

Fresh SC-09 servos all ship at ID = 1. To address three independently:

- Single-servo path (cleanest):
  1. Connect only servo A. `CMD,SERVO_SET_ID,1,1` (no-op, sanity) or
     just leave at 1.
  2. Connect only servo B. `CMD,SERVO_SET_ID,1,2`.
  3. Connect only servo C. `CMD,SERVO_SET_ID,1,3`.

- Always-≥2-on-bus path (when single-servo isn't possible — the
  half-duplex driver may need ≥2 connectors populated to behave):
  1. Plug B + C only, broadcast `CMD,SERVO_BCAST_SET_ID,2` to rename
     both to ID = 2.
  2. Plug A back, unplug B. Bus now has A at ID=1 and C at ID=2.
     `CMD,SERVO_SET_ID,2,3` — targets only C since A is at 1.
  3. Plug B back. State: A=1, B=2, C=3.

Verify by motion only (`MOVE N` should move exactly one servo).

## Code references

- `embedded/include/ServoBus.h` — public driver API
- `embedded/src/ServoBus.cpp` — PImpl wrapper over Waveshare SCServo
- `embedded/src/main.cpp` — servotest mode that exercises the bus
- `embedded/platformio.ini` — `[env:esp32s3wroom1-servotest]` flashes
  the bring-up build with the SCServo dependency wired in.
