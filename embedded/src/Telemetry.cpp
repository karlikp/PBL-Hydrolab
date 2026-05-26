#include "Telemetry.h"

#include <stdio.h>

#include "Clock.h"
#include "RadioLink.h"

Telemetry::Telemetry(Clock& clock, RadioLink& radio)
    : clock_(clock), radio_(radio) {}

void Telemetry::update_gps(long lat, long lon,
                           uint16_t year, uint8_t month, uint8_t day,
                           uint8_t hour, uint8_t minute, uint8_t second,
                           bool fix_valid) {
    latest_.lat       = lat;
    latest_.lon       = lon;
    latest_.year      = year;
    latest_.month     = month;
    latest_.day       = day;
    latest_.hour      = hour;
    latest_.minute    = minute;
    latest_.second    = second;
    latest_.fix_valid = fix_valid;
}

void Telemetry::update_measurement(float cond, float temp, float ph, float oxygen,
                                   bool measurement_valid) {
    latest_.cond              = cond;
    latest_.temp              = temp;
    latest_.ph                = ph;
    latest_.oxygen            = oxygen;
    latest_.measurement_valid = measurement_valid;
}

void Telemetry::clear_measurement() {
    latest_.cond              = 0.0f;
    latest_.temp              = 0.0f;
    latest_.ph                = 0.0f;
    latest_.oxygen            = 0.0f;
    latest_.measurement_valid = false;
}

void Telemetry::tick() {
    const uint32_t now = clock_.now_ms();
    if (now - last_emit_ms_ < TLM_INTERVAL_MS) return;
    last_emit_ms_ = now;
    emit();
}

void Telemetry::emit() {
    // Use GPS UTC if the last sample has a valid date+time; otherwise
    // fall back to a millis()-synthesised stamp so frames stay parseable
    // during bring-up before the module has a fix.
    unsigned yyyy, mo, dd, hh, mm, ss;
    if (latest_.year != 0) {
        yyyy = latest_.year;
        mo   = latest_.month;
        dd   = latest_.day;
        hh   = latest_.hour;
        mm   = latest_.minute;
        ss   = latest_.second;
    } else {
        const uint32_t s = clock_.now_ms() / 1000;
        yyyy = 1970;
        mo   = 1;
        dd   = 1;
        hh   = (s / 3600) % 24;
        mm   = (s / 60) % 60;
        ss   =  s % 60;
    }

    char buf[200];
    const int n = snprintf(buf, sizeof(buf),
        "TLM,%04u-%02u-%02u %02u:%02u:%02u,%ld,%ld,%.2f,%.2f,%.2f,%.2f,%d,%d",
        yyyy, mo, dd, hh, mm, ss,
        latest_.lat, latest_.lon,
        latest_.cond, latest_.temp, latest_.ph, latest_.oxygen,
        latest_.water_flag ? 1 : 0,
        latest_.measurement_valid ? 1 : 0);

    // Refuse to send a truncated frame (defends against future field
    // additions that overflow the buffer — see comms-robustness notes).
    if (n <= 0 || static_cast<size_t>(n) >= sizeof(buf)) return;

    radio_.send(buf);
}
