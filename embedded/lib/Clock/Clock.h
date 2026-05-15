// Clock — time-source abstraction.
//
// This is the one and only interface in the embedded tree. It exists
// so the FSMs can be unit-tested by advancing a fake clock instead of
// waiting for real wall-clock time. RealClock wraps Arduino's millis();
// TestClock returns whatever value the test set last.
//
// Anything else that needs the current time goes through this — never
// call millis() directly from FSM code.

#pragma once

#include <stdint.h>

class Clock {
public:
    virtual ~Clock() = default;
    virtual uint32_t now_ms() const = 0;
};

// Production clock — reads Arduino millis().
class RealClock : public Clock {
public:
    uint32_t now_ms() const override;
};

// Test clock — manually advanced by the test harness.
class TestClock : public Clock {
public:
    uint32_t now_ms() const override { return now_; }
    void advance(uint32_t ms) { now_ += ms; }
    void set(uint32_t ms) { now_ = ms; }
private:
    uint32_t now_ = 0;
};
