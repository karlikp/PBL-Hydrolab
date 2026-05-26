// Telemetry — periodic TLM frame broadcaster.
//
// Emits one TLM frame every TLM_INTERVAL_MS as required by the protocol
// (2 Hz). Sensor / GPS fields default to zero — the GCS interprets any
// zero field as "sensor not ready" per docs/protocol.md §4, so the
// dashboard still gets a heartbeat showing the drone is alive even
// before any driver has called an update().
//
// The TLM also carries a measurement_valid flag (appended after
// water_flag). Per protocol §10 forward-compat, the GCS may ignore it,
// but when set it lets the GCS distinguish "real zero reading" from
// "probe present but still settling" — the in-air zero case is already
// covered by the §4 zero convention.
//
// Two narrow update paths, by field family, so the GPS-side and the
// probe-side updaters don't clobber each other's data:
//
//   update_gps(...)         — lat/lon/UTC/fix_valid only
//   update_measurement(...) — cond/temp/ph/oxygen/measurement_valid only
//
// The TLM timestamp uses the most recent GPS fix when fix_valid is set;
// otherwise it falls back to a millis()-synthesised `1970-01-01 HH:MM:SS`
// so frames remain well-formed during bring-up.

#pragma once

#include <stdint.h>

class Clock;
class RadioLink;

class Telemetry {
public:
    struct Sample {
        long     lat               = 0;       // degrees ×10^7
        long     lon               = 0;       // degrees ×10^7
        uint16_t year              = 0;       // UTC; 0 = use millis() fallback
        uint8_t  month             = 0;
        uint8_t  day               = 0;
        uint8_t  hour              = 0;
        uint8_t  minute            = 0;
        uint8_t  second            = 0;
        bool     fix_valid         = false;   // GPS reports a valid position
        float    cond              = 0.0f;    // mS/cm
        float    temp              = 0.0f;    // °C
        float    ph                = 0.0f;
        float    oxygen            = 0.0f;    // mg/L
        bool     water_flag        = false;
        bool     measurement_valid = false;   // probe steady-state reached
    };

    Telemetry(Clock& clock, RadioLink& radio);

    // Set the GPS-side fields (lat/lon/UTC/fix_valid). Probe-side
    // fields are left untouched. Called from the GPS pump in main loop.
    void update_gps(long lat, long lon,
                    uint16_t year, uint8_t month, uint8_t day,
                    uint8_t hour, uint8_t minute, uint8_t second,
                    bool fix_valid);

    // Set the probe-side fields. measurement_valid=true means the
    // values reflect a settled, trustworthy reading; false means the
    // probe is mid-settle (or otherwise not steady) and the values
    // should be treated as informational only.
    void update_measurement(float cond, float temp, float ph, float oxygen,
                            bool measurement_valid);

    // Clear probe-side fields back to "no probe / docked". Use when
    // the Elmetron returns to DOCKED so TLM doesn't keep emitting the
    // last in-water reading indefinitely.
    void clear_measurement();

    // Drive periodic emission. Call from the main loop.
    void tick();

private:
    Clock&     clock_;
    RadioLink& radio_;
    Sample     latest_;
    uint32_t   last_emit_ms_ = 0;

    static constexpr uint32_t TLM_INTERVAL_MS = 500;  // 2 Hz, per protocol §4

    void emit();
};
