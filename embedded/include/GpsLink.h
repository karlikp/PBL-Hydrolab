// GpsLink — driver for the u-blox NEO-M8U over I2C.
//
// The production board wires the NEO-M8U to the ESP32-S3's I2C pins:
//
//     SDA = IO8
//     SCL = IO9
//
// We use SparkFun's u-blox library to talk UBX over Wire. Polling
// runs at 1 Hz inside tick() and caches the latest Fix; callers
// (Telemetry, CMD,GPS,STATUS) read the cache rather than driving the
// bus themselves.
//
// SparkFun_u-blox_GNSS_Arduino_Library.h is intentionally NOT included
// here so TUs that just want a Fix don't pull the whole u-blox surface.
// The implementation hides it behind PImpl, same pattern as ServoBus.
//
// NEO-M8U is a dead-reckoning capable module — its fix can persist
// briefly without satellites by integrating inertial data, so we use
// the chip's own "GNSS fix OK" flag as the fix_valid signal rather
// than the simpler "fixType >= 2D" heuristic.

#pragma once

#include <stdint.h>

class Clock;

class GpsLink {
public:
    struct Fix {
        long     lat       = 0;    // degrees ×10^7 (signed)
        long     lon       = 0;    // degrees ×10^7 (signed)
        uint16_t year      = 0;    // 0 means no valid date yet
        uint8_t  month     = 0;
        uint8_t  day       = 0;
        uint8_t  hour      = 0;
        uint8_t  minute    = 0;
        uint8_t  second    = 0;
        uint8_t  sat_count = 0;
        bool     fix_valid = false;
    };

    explicit GpsLink(Clock& clock);
    ~GpsLink();

    GpsLink(const GpsLink&)            = delete;
    GpsLink& operator=(const GpsLink&) = delete;

    // Initialise I2C on the given pins and bring up the u-blox module.
    // Returns true on success (module ACKed and was configured).
    bool begin(int sda_pin, int scl_pin);

    // Poll the module at POLL_INTERVAL_MS. Updates the cached Fix.
    void tick();

    // Most recent cached values. fix_valid stays false until the
    // module reports a GNSS solution; date/time may become valid
    // earlier (broadcast time without a position fix).
    const Fix& last_fix() const { return last_; }

    // True iff anything in last_fix() changed since the last call.
    // Lets callers push samples downstream only when there's news.
    bool consume_changed();

    // True if begin() (or a subsequent reinit()) succeeded — i.e. the
    // module is on the bus and responding. False if the I2C ACK never
    // came; callers can still poll tick() but the cache won't update.
    bool present() const { return present_; }

    // Re-run the u-blox bring-up sequence on the existing I2C bus.
    // Used by CMD,GPS,RESET to recover after a bus glitch.
    bool reinit();

private:
    struct Impl;
    Impl*    impl_;
    Clock&   clock_;
    int      sda_pin_      = -1;
    int      scl_pin_      = -1;
    uint32_t last_poll_ms_ = 0;
    Fix      last_;
    bool     changed_      = false;
    bool     present_      = false;

    static constexpr uint32_t POLL_INTERVAL_MS = 1000;  // 1 Hz

    bool configure_module();
    void poll_module();
};
