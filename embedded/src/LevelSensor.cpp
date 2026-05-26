#include "LevelSensor.h"

#include <Arduino.h>
#include <driver/gpio.h>

int LevelSensor::pin_for(uint8_t tank_id) {
    switch (tank_id) {
        case 1: return TOPCN1_PIN;
        case 2: return TOPCN2_PIN;
        case 3: return TOPCN3_PIN;
    }
    return -1;
}

void LevelSensor::begin() {
    // Build a single bitmask for all three pins and configure them in
    // one ESP-IDF call. gpio_config() forcefully takes the IO_MUX away
    // from JTAG (Arduino's pinMode is supposed to do this too, but
    // it's been unreliable on 40/41/42 in some framework versions).
    uint64_t mask = 0;
    for (uint8_t id = 1; id <= TANK_COUNT; ++id) {
        const int pin = pin_for(id);
        if (pin >= 0) mask |= (1ULL << pin);
    }
    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

bool LevelSensor::is_full(uint8_t tank_id) const {
    const int pin = pin_for(tank_id);
    if (pin < 0) return false;
    const int level = digitalRead(pin);
    return ACTIVE_LOW ? (level == LOW) : (level == HIGH);
}

int LevelSensor::raw(uint8_t tank_id) const {
    const int pin = pin_for(tank_id);
    if (pin < 0) return -1;
    return digitalRead(pin);
}
