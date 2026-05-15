#include "Telemetry.h"

#include <stdio.h>

#include "Clock.h"
#include "RadioLink.h"

Telemetry::Telemetry(Clock& clock, RadioLink& radio)
    : clock_(clock), radio_(radio) {}

void Telemetry::update(const Sample& s) {
    latest_ = s;
}

void Telemetry::tick() {
    const uint32_t now = clock_.now_ms();
    if (now - last_emit_ms_ < TLM_INTERVAL_MS) return;
    last_emit_ms_ = now;
    emit();
}

void Telemetry::emit() {
    // Synthesise a clock from millis() until GPS time is wired in.
    const uint32_t ms  = clock_.now_ms();
    const uint32_t s   = ms / 1000;
    const uint32_t hh  = (s / 3600) % 24;
    const uint32_t mm  = (s / 60) % 60;
    const uint32_t ss  =  s % 60;

    char buf[200];
    const int n = snprintf(buf, sizeof(buf),
        "TLM,1970-01-01 %02u:%02u:%02u,%ld,%ld,%.2f,%.2f,%.2f,%.2f,%d",
        (unsigned)hh, (unsigned)mm, (unsigned)ss,
        latest_.lat, latest_.lon,
        latest_.cond, latest_.temp, latest_.ph, latest_.oxygen,
        latest_.water_flag ? 1 : 0);

    // Refuse to send a truncated frame (defends against future field
    // additions that overflow the buffer — see comms-robustness notes).
    if (n <= 0 || static_cast<size_t>(n) >= sizeof(buf)) return;

    radio_.send(buf);
}
