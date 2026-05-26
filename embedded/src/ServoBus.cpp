#include "ServoBus.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SCServo.h>

struct ServoBus::Impl {
    SCSCL sc;
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

static constexpr uint32_t EEPROM_COMMIT_MS = 20;

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
