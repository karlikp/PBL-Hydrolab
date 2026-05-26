// Telemetry — periodic TLM frame broadcaster.
//
// Emits one TLM frame every TLM_INTERVAL_MS as required by the protocol
// (2 Hz). Sensor / GPS fields default to zero — the GCS interprets any
// zero field as "sensor not ready" per docs/protocol.md §4, so the
// dashboard still gets a heartbeat showing the drone is alive even
// before any driver has called update().
//
// When real sensor / GPS drivers come online, they call update() with
// a fresh sample; the broadcast path itself doesn't change. The TLM
// timestamp prefix uses the Sample's GPS date/time when fix_valid is
// set; otherwise it falls back to a millis()-synthesised
// `1970-01-01 HH:MM:SS` so frames remain well-formed during bring-up.

#pragma once

#include <stdint.h>

class Clock;
class RadioLink;

class Telemetry {
public:
    struct Sample {
        long     lat        = 0;       // degrees ×10^7
        long     lon        = 0;       // degrees ×10^7
        uint16_t year       = 0;       // UTC; 0 = use millis() fallback
        uint8_t  month      = 0;
        uint8_t  day        = 0;
        uint8_t  hour       = 0;
        uint8_t  minute     = 0;
        uint8_t  second     = 0;
        bool     fix_valid  = false;   // GPS reports a valid position
        float    cond       = 0.0f;    // mS/cm
        float    temp       = 0.0f;    // °C
        float    ph         = 0.0f;
        float    oxygen     = 0.0f;    // mg/L
        bool     water_flag = false;
    };

    Telemetry(Clock& clock, RadioLink& radio);

    // Provide a fresh sample. Called by sensor/GPS layers when new
    // readings are available. The most recent sample is broadcast on
    // every TLM tick. Until anyone calls this, the broadcast carries
    // zeros (i.e. "sensor not ready").
    void update(const Sample& s);

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
