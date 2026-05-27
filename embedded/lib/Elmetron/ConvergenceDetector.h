// ConvergenceDetector — "has the reading stopped moving?" for a single
// scalar (the Elmetron conductivity during a measurement).
//
// Holds a sliding window of recent samples. converged() is true once the
// window spans at least WINDOW_MS AND the spread (max-min)/mean across it
// is within TOLERANCE — i.e. the value has been roughly flat for the
// whole window. This replaces a fixed sample count: the measurement runs
// as long as it needs to settle, with a strict upstream timeout as the
// backstop.
//
// Fed at the probe's poll rate (~2 Hz); add() only on genuinely-new
// readings so the window reflects real elapsed time, not the loop rate.
// Pure / header-only so it's unit-testable off-target.

#pragma once

#include <stdint.h>
#include <stddef.h>

class ConvergenceDetector {
public:
    // Rough defaults — tune on real hardware once probe noise is known.
    static constexpr uint32_t WINDOW_MS = 10000;  // 10 s flat ⇒ converged
    static constexpr float    TOLERANCE = 0.05f;  // 5% spread (rough)
    static constexpr size_t   CAP       = 48;     // ~2 Hz × 24 s headroom
    static constexpr size_t   MIN_SAMPLES = 4;

    void reset() { count_ = 0; head_ = 0; }

    void add(uint32_t now_ms, float value) {
        ts_[head_]  = now_ms;
        val_[head_] = value;
        head_ = (head_ + 1) % CAP;
        if (count_ < CAP) ++count_;
    }

    // True when the in-window samples span >= WINDOW_MS and are flat to
    // within TOLERANCE (relative spread).
    bool converged(uint32_t now_ms) const {
        float mn = 0.0f, mx = 0.0f, sum = 0.0f;
        size_t n = 0;
        uint32_t oldest = now_ms;
        for (size_t i = 0; i < count_; ++i) {
            if (now_ms - ts_[i] > WINDOW_MS) continue;  // outside window
            const float v = val_[i];
            if (n == 0) { mn = v; mx = v; }
            else { if (v < mn) mn = v; if (v > mx) mx = v; }
            sum += v;
            ++n;
            if (ts_[i] < oldest) oldest = ts_[i];
        }
        if (n < MIN_SAMPLES) return false;
        // Require the window to actually be full (oldest in-window sample
        // ~WINDOW_MS old) so we don't declare convergence on a brief
        // flat patch right after entering the water.
        if (now_ms - oldest < WINDOW_MS) return false;
        const float mean = sum / static_cast<float>(n);
        if (mean <= 0.0f) return false;
        return ((mx - mn) / mean) <= TOLERANCE;
    }

private:
    uint32_t ts_[CAP]  = {0};
    float    val_[CAP] = {0};
    size_t   head_  = 0;
    size_t   count_ = 0;
};
