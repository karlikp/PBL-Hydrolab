#include "PumpControl.h"

#include <Arduino.h>

int PumpControl::pin_for(uint8_t tank_id) {
    switch (tank_id) {
        case 1: return PUMP1_PIN;
        case 2: return PUMP2_PIN;
        case 3: return PUMP3_PIN;
    }
    return -1;
}

void PumpControl::begin() {
    for (uint8_t id = 1; id <= TANK_COUNT; ++id) {
        const int pin = pin_for(id);
        pinMode(pin, OUTPUT);
        // Start in the OFF state.
        digitalWrite(pin, ACTIVE_HIGH ? LOW : HIGH);
        state_[id - 1] = false;
    }
}

void PumpControl::set(uint8_t tank_id, bool on) {
    const int pin = pin_for(tank_id);
    if (pin < 0) return;
    digitalWrite(pin, (on == ACTIVE_HIGH) ? HIGH : LOW);
    state_[tank_id - 1] = on;
}

void PumpControl::stop_all() {
    for (uint8_t id = 1; id <= TANK_COUNT; ++id) set(id, false);
}

bool PumpControl::last_state(uint8_t tank_id) const {
    if (tank_id < 1 || tank_id > TANK_COUNT) return false;
    return state_[tank_id - 1];
}
