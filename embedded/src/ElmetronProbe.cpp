#include "ElmetronProbe.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>
#include <stdlib.h>

#include "Clock.h"

ElmetronProbe::ElmetronProbe(HardwareSerial& serial, Clock& clock)
    : serial_(serial), clock_(clock) {}

void ElmetronProbe::begin(int rx_pin, int tx_pin) {
    // 8E1 per the legacy CX driver. If the probe stays silent on the
    // bench, this parity (or the rx/tx orientation) is the first thing
    // to flip.
    serial_.begin(BAUD, SERIAL_8E1, rx_pin, tx_pin);
    step_ = Step::REQUEST;
    len_  = 0;
}

void ElmetronProbe::send_query() {
    serial_.write(static_cast<uint8_t>(0x01));  // SOH
    serial_.print("#0#0#0#");
    serial_.write(static_cast<uint8_t>(0x03));  // ETX
}

void ElmetronProbe::tick() {
    const uint32_t now = clock_.now_ms();
    switch (step_) {
        case Step::REQUEST:
            // Flush any stale bytes, fire a fresh query.
            while (serial_.available()) serial_.read();
            len_ = 0;
            send_query();
            last_step_ms_ = now;
            step_ = Step::WAIT;
            break;

        case Step::WAIT:
            if (now - last_step_ms_ >= READ_DELAY_MS) step_ = Step::PROCESS;
            break;

        case Step::PROCESS:
            process_incoming();
            step_ = Step::REQUEST;
            break;
    }
}

void ElmetronProbe::process_incoming() {
    while (serial_.available()) {
        const int ci = serial_.read();
        if (ci < 0) break;
        const char c = static_cast<char>(ci);
        if (c == 0x03) {                 // ETX → end of frame
            buf_[len_] = '\0';
            if (len_ > 0) parse_frame(buf_);
            len_ = 0;
        } else if (c != 0x01 && c != 0x02) {  // strip SOH / STX
            if (len_ < sizeof(buf_) - 1) buf_[len_++] = c;
        }
    }
}

void ElmetronProbe::parse_frame(char* s) {
    Reading r;
    // '#'-delimited tokens; each carries a value labelled by its unit.
    char* save = nullptr;
    for (char* tok = strtok_r(s, "#", &save); tok;
         tok = strtok_r(nullptr, "#", &save)) {
        if (strstr(tok, "pH")) {
            r.ph = atof(tok);
        } else if (strstr(tok, "O2")) {
            r.oxygen = atof(tok);
        } else if (const char* s = strstr(tok, "S/cm")) {
            // Explicit unit-prefix detection (replaces the legacy >10
            // magnitude heuristic). The byte immediately before 'S' is
            // 'm' for "mS/cm" (keep as-is) — anything else (a 'µ' byte,
            // single 0xB5 or the trailing 0xBC of UTF-8 0xCE 0xBC, or
            // no prefix at all) is treated as "µS/cm" and divided.
            // Works for the full range, including very low cond where
            // the magnitude heuristic broke.
            float v = atof(tok);
            const bool already_mS = (s > tok) && (*(s - 1) == 'm');
            if (!already_mS) v /= 1000.0f;
            r.conductivity = v;
        } else if (strchr(tok, 'C') && !strstr(tok, "CX")) {
            r.temperature = atof(tok);       // exclude the "CX" model tag
        }
    }
    r.water = r.conductivity > water_threshold_ms_;
    r.valid = true;
    last_     = r;
    changed_  = true;
    present_  = true;
}

bool ElmetronProbe::consume_changed() {
    const bool c = changed_;
    changed_ = false;
    return c;
}
