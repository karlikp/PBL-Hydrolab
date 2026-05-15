#include "Clock.h"
#include <Arduino.h>

uint32_t RealClock::now_ms() const {
    return millis();
}
