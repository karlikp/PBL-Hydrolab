#include "ServoBus.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SCServo.h>

// Delay between EEPROM writes — servo's internal commit takes a few
// ms; without this the next packet can arrive before the previous
// write has landed. Used by set_id and the PWM/position-mode helpers.
static constexpr uint32_t EEPROM_COMMIT_MS = 20;

// Half-duplex echo-drain wrapper.
//
// The SC-09 bus is a single wire shared between TX and RX. Every byte
// the ESP transmits is also echoed back into its own RX. The stock
// SCServo library's wFlushSCS() is a no-op, so after writing a request
// the library immediately tries to read the reply — and reads the
// ECHO of its own request instead. This breaks every read in the lib
// (ReadPos returns 14338 = the request's MemAddr byte plus garbage,
// Ping always "succeeds" because the echoed header matches the reply
// header shape, and Acks on writes erratically fail/pass).
//
// We override:
//   writeSCS variants → count outgoing bytes into echo_pending_
//   rFlushSCS         → reset echo_pending_ (drains old RX anyway)
//   wFlushSCS         → flush TX to the wire, then read and discard
//                       exactly echo_pending_ bytes from RX (the echo)
//
// After wFlushSCS, the RX buffer holds only the servo's actual reply
// (if any) — the lib's checkHead/readSCS see clean data.
class EchoDrainSCSCL : public SCSCL {
public:
    EchoDrainSCSCL() : SCSCL() {}

protected:
    int echo_pending_ = 0;

    int writeSCS(unsigned char* nDat, int nLen) override {
        const int wrote = SCSerial::writeSCS(nDat, nLen);
        if (wrote > 0) echo_pending_ += wrote;
        return wrote;
    }

    int writeSCS(unsigned char bDat) override {
        const int wrote = SCSerial::writeSCS(bDat);
        if (wrote > 0) echo_pending_ += wrote;
        return wrote;
    }

    void rFlushSCS() override {
        SCSerial::rFlushSCS();
        echo_pending_ = 0;
    }

    void wFlushSCS() override {
        // Wait for TX to physically reach the wire (base wFlushSCS is
        // empty; we use the underlying Stream's flush).
        if (pSerial) pSerial->flush();
        if (!pSerial || echo_pending_ < 2) {
            echo_pending_ = 0;
            return;
        }
        // Drain JUST the first 2 echo bytes (the leading 0xFF 0xFF of
        // our own request). The lib's checkHead() scans for 0xFF 0xFF
        // within a 10-byte window — once our header is out of the way,
        // the remaining echo bytes (6 for a Ping, 6–7 for Read/Write)
        // are not 0xFF 0xFF so they get skipped, and checkHead lands on
        // the real reply's 0xFF 0xFF. This works even if the servo has
        // zero return delay (no race with my drain).
        //
        // Hard cap (2 ms) so a missing echo can't hang the loop.
        int drained = 0;
        const unsigned long t_start = micros();
        while (drained < 2) {
            const int c = pSerial->read();
            if (c != -1) {
                drained++;
            } else if (micros() - t_start > 2000) {
                break;
            }
        }
        echo_pending_ = 0;
    }
};

struct ServoBus::Impl {
    EchoDrainSCSCL sc;
};

ServoBus::ServoBus(HardwareSerial& serial) : impl_(new Impl()) {
    impl_->sc.pSerial = &serial;
}

ServoBus::~ServoBus() {
    delete impl_;
}

bool ServoBus::move(uint8_t id, uint16_t position, uint16_t speed) {
    if (position > 1023) position = 1023;
    // SCSCL::WritePos(id, position, time, speed) — we leave time = 0,
    // speed alone gives smooth enough motion.
    return impl_->sc.WritePos(id, position, /*time=*/0, speed) >= 0;
}

bool ServoBus::set_pwm_mode(uint8_t id) {
    // Writes 0/0 into SCSCL_MIN/MAX_ANGLE_LIMIT — EEPROM write,
    // persists across power cycles. PWMMode() is the lib's shorthand
    // for the same write. Ack unreliable on half-duplex (see set_id).
    impl_->sc.PWMMode(id);
    delay(EEPROM_COMMIT_MS);
    return true;
}

bool ServoBus::set_position_mode(uint8_t id) {
    // Inverse of set_pwm_mode: restore full single-turn angle limits.
    // Lib has no helper for this — go through generic byte writes.
    // Registers 9..12 = MIN_L, MIN_H, MAX_L, MAX_H. MIN = 0, MAX = 1023.
    impl_->sc.writeByte(id, SCSCL_MIN_ANGLE_LIMIT_L, 0);
    impl_->sc.writeByte(id, SCSCL_MIN_ANGLE_LIMIT_H, 0);
    impl_->sc.writeByte(id, SCSCL_MAX_ANGLE_LIMIT_L, 1023 & 0xFF);
    impl_->sc.writeByte(id, SCSCL_MAX_ANGLE_LIMIT_H, (1023 >> 8) & 0xFF);
    delay(EEPROM_COMMIT_MS);
    return true;
}

bool ServoBus::write_pwm(uint8_t id, int16_t pwm) {
    // Magnitude clipped to 1023 (10-bit duty); sign = direction. The
    // lib repacks the sign into bit 10 of the wire word.
    if (pwm >  1023) pwm =  1023;
    if (pwm < -1023) pwm = -1023;
    impl_->sc.WritePWM(id, pwm);
    return true;  // half-duplex Ack unreliable; trust the write
}

int ServoBus::ping(uint8_t id) {
    return impl_->sc.Ping(id);
}

int ServoBus::read_position(uint8_t id) {
    // SCSCL::ReadPos sends an 8-byte READ instruction and reads back a
    // status reply containing the position word. On this board's
    // half-duplex bus the request echoes back on RX before the servo's
    // reply arrives — see the explainer above `set_id` for the same
    // problem on writes. The SCServo library has no echo-drain, so
    // this call may return garbage on real hardware. We at least drain
    // any leftover RX afterwards so the next operation starts clean.
    const int pos = impl_->sc.ReadPos(id);
    if (impl_->sc.pSerial) {
        while (impl_->sc.pSerial->available()) (void)impl_->sc.pSerial->read();
    }
    return pos;
}

// Both set_id variants intentionally ignore the SCServo library's
// Ack() return codes. Reason: the SC-09 bus is half-duplex on a single
// wire, so every byte the ESP transmits is also echoed back on its
// RX. For a PING the 6-byte echo happens to look like a 6-byte status
// reply, so Ack succeeds (falsely). For a WRITE (8 bytes) the echo is
// longer than a status reply; Ack reads the first 6 bytes of echo,
// sees a LEN-field mismatch, and returns failure even though the
// underlying write was successfully transmitted and the servo did
// execute it. So we cannot trust Ack for WRITE ops on this bus.
//
// We just blast the three EEPROM packets with a small delay between
// them so the servo's internal EEPROM commit finishes before the next
// packet arrives. Verification happens externally (re-ping at the new
// ID, or watch which servo moves in response to a MOVE).
//
// (A proper fix would drain the echo bytes in a custom Stream wrapper
// before each Ack read, but that is invasive and orthogonal to the
// immediate need.)

bool ServoBus::set_id(uint8_t current_id, uint8_t new_id) {
    SCSCL& sc = impl_->sc;
    sc.unLockEprom(current_id);
    delay(EEPROM_COMMIT_MS);
    sc.writeByte(current_id, SCSCL_ID, new_id);
    delay(EEPROM_COMMIT_MS);
    // Servo's ID has changed — lock EEPROM through the new ID.
    sc.LockEprom(new_id);
    delay(EEPROM_COMMIT_MS);
    return true;
}

// Build and send a ping packet via the raw HardwareSerial. Captures
// any bytes that come back over `wait_ms` ms. The SCServo lib is
// bypassed entirely so we can see exactly what's on the wire.
size_t ServoBus::raw_ping_capture(uint8_t id, uint8_t* out, size_t out_max,
                                  uint32_t wait_ms) {
    HardwareSerial* serial = impl_->sc.pSerial;
    if (!serial) return 0;

    // Drain any stale RX.
    while (serial->read() != -1) {}

    // Construct a ping packet: 0xFF 0xFF id 0x02 INST_PING(=1) ~cksum.
    // Checksum = ~(id + 0x02 + 0x01).
    uint8_t pkt[6];
    pkt[0] = 0xFF;
    pkt[1] = 0xFF;
    pkt[2] = id;
    pkt[3] = 0x02;
    pkt[4] = 0x01;
    pkt[5] = static_cast<uint8_t>(~(id + 0x02 + 0x01));
    serial->write(pkt, 6);
    serial->flush();  // wait for TX to physically complete

    // Capture bytes for wait_ms. Includes both the echo (if any) AND
    // the servo's reply (if any). No filtering — pure dump.
    size_t got = 0;
    const uint32_t t_start = millis();
    while (millis() - t_start < wait_ms && got < out_max) {
        const int c = serial->read();
        if (c != -1) {
            out[got++] = static_cast<uint8_t>(c);
        }
    }
    return got;
}

bool ServoBus::set_status_return_level(uint8_t id, uint8_t level) {
    // Register 8 is Status Return Level on Feetech SCS-family servos.
    // EEPROM write, so unlock → write → lock with commit delays.
    SCSCL& sc = impl_->sc;
    sc.unLockEprom(id);
    delay(EEPROM_COMMIT_MS);
    sc.writeByte(id, /*reg=*/8, level);
    delay(EEPROM_COMMIT_MS);
    sc.LockEprom(id);
    delay(EEPROM_COMMIT_MS);
    return true;
}

bool ServoBus::set_return_delay(uint8_t id, uint8_t value) {
    // Register 7 is Return Delay on Feetech SCS-family servos. EEPROM.
    SCSCL& sc = impl_->sc;
    sc.unLockEprom(id);
    delay(EEPROM_COMMIT_MS);
    sc.writeByte(id, /*reg=*/7, value);
    delay(EEPROM_COMMIT_MS);
    sc.LockEprom(id);
    delay(EEPROM_COMMIT_MS);
    return true;
}

bool ServoBus::broadcast_set_id(uint8_t new_id) {
    SCSCL& sc = impl_->sc;
    sc.unLockEprom(BROADCAST_ID);
    delay(EEPROM_COMMIT_MS);
    sc.writeByte(BROADCAST_ID, SCSCL_ID, new_id);
    delay(EEPROM_COMMIT_MS);
    sc.LockEprom(BROADCAST_ID);
    delay(EEPROM_COMMIT_MS);
    return true;
}
