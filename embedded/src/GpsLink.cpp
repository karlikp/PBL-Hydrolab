#include "GpsLink.h"

#include <Arduino.h>

#include "Clock.h"

#ifdef MOCK_PERIPHERALS

// Mock build — synthesizes a fixed fix so the GCS sees plausible
// geo-tags on collection events and TLM has live lat/lon/UTC without
// the u-blox or I2C bus actually being present. Used by the
// `esp32s3wroom1-mock` env to develop the ground station against the
// real firmware running on a bare ESP32-S3 with nothing else connected.

struct GpsLink::Impl {};

GpsLink::GpsLink(Clock& clock) : impl_(new Impl()), clock_(clock) {}
GpsLink::~GpsLink() { delete impl_; }

bool GpsLink::begin(int sda_pin, int scl_pin) {
    sda_pin_ = sda_pin;
    scl_pin_ = scl_pin;
    return configure_module();
}

bool GpsLink::reinit() {
    return configure_module();
}

bool GpsLink::configure_module() {
    present_ = true;
    return true;
}

void GpsLink::tick() {
    const uint32_t now = clock_.now_ms();
    if (now - last_poll_ms_ < POLL_INTERVAL_MS) return;
    last_poll_ms_ = now;
    poll_module();
}

void GpsLink::poll_module() {
    // Plausible position near Warsaw — matches the protocol-doc example
    // so collection markers on the operator map land somewhere real.
    Fix f;
    f.lat       = 521230000;   // ~52.1230° N
    f.lon       = 210110000;   // ~21.0110° E
    f.sat_count = 8;
    f.fix_valid = true;

    // Date stays at a fixed mock day; time ticks off the system clock
    // so the operator sees the second hand moving and can verify
    // "fresh" vs "stale" GPS frames in the UI.
    const uint32_t s = clock_.now_ms() / 1000;
    f.year   = 2026;
    f.month  = 5;
    f.day    = 26;
    f.hour   = (s / 3600) % 24;
    f.minute = (s / 60) % 60;
    f.second =  s % 60;

    if (f.second != last_.second || f.fix_valid != last_.fix_valid) {
        last_    = f;
        changed_ = true;
    }
}

bool GpsLink::consume_changed() {
    const bool c = changed_;
    changed_ = false;
    return c;
}

#else  // !MOCK_PERIPHERALS — real u-blox driver

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

struct GpsLink::Impl {
    SFE_UBLOX_GNSS gnss;
};

GpsLink::GpsLink(Clock& clock) : impl_(new Impl()), clock_(clock) {}

GpsLink::~GpsLink() { delete impl_; }

bool GpsLink::begin(int sda_pin, int scl_pin) {
    sda_pin_ = sda_pin;
    scl_pin_ = scl_pin;
    Wire.begin(sda_pin_, scl_pin_);
    return configure_module();
}

bool GpsLink::reinit() {
    return configure_module();
}

bool GpsLink::configure_module() {
    // Default I2C address (0x42). begin(Wire) handshakes with the
    // module — returns false if no ACK on the bus.
    if (!impl_->gnss.begin(Wire)) {
        present_ = false;
        return false;
    }
    impl_->gnss.setI2COutput(COM_TYPE_UBX);   // suppress NMEA chatter
    impl_->gnss.setNavigationFrequency(1);    // 1 Hz solution rate
    present_ = true;
    return true;
}

void GpsLink::tick() {
    const uint32_t now = clock_.now_ms();
    if (now - last_poll_ms_ < POLL_INTERVAL_MS) return;
    last_poll_ms_ = now;

    if (!present_) return;
    poll_module();
}

void GpsLink::poll_module() {
    auto& g = impl_->gnss;
    g.checkUblox();  // drain incoming UBX so getters work off fresh data

    Fix f;
    f.lat       = g.getLatitude();
    f.lon       = g.getLongitude();
    f.sat_count = g.getSIV();
    f.fix_valid = g.getGnssFixOk();

    // Time / date can be valid before a position fix (the module
    // broadcasts time as soon as it decodes any satellite). Gate each
    // independently against the u-blox "valid" flags.
    if (g.getTimeValid()) {
        f.hour   = g.getHour();
        f.minute = g.getMinute();
        f.second = g.getSecond();
    }
    if (g.getDateValid()) {
        f.year  = g.getYear();
        f.month = g.getMonth();
        f.day   = g.getDay();
    }

    // Only mark "changed" when the data actually moves — keeps
    // Telemetry from re-emitting identical samples 1 Hz forever.
    if (f.lat       != last_.lat       ||
        f.lon       != last_.lon       ||
        f.year      != last_.year      ||
        f.month     != last_.month     ||
        f.day       != last_.day       ||
        f.hour      != last_.hour      ||
        f.minute    != last_.minute    ||
        f.second    != last_.second    ||
        f.sat_count != last_.sat_count ||
        f.fix_valid != last_.fix_valid) {
        last_    = f;
        changed_ = true;
    }
}

bool GpsLink::consume_changed() {
    const bool c = changed_;
    changed_ = false;
    return c;
}

#endif  // MOCK_PERIPHERALS
