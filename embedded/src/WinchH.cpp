#include "WinchH.h"

#include <Arduino.h>

WinchH::WinchH(int en_l_pin, int en_r_pin, int pwm_pin, int limit_pin)
    : en_l_(en_l_pin), en_r_(en_r_pin), pwm_(pwm_pin), limit_(limit_pin) {}

void WinchH::begin() {
    pinMode(en_l_, OUTPUT);
    pinMode(en_r_, OUTPUT);
    digitalWrite(en_l_, LOW);
    digitalWrite(en_r_, LOW);
    pinMode(limit_, INPUT_PULLUP);
    ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(pwm_, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0);
}

void WinchH::drive(Direction dir, uint8_t duty_pct) {
    if (duty_pct > 100) duty_pct = 100;
    const uint32_t max_duty = (1u << PWM_RES_BITS) - 1;
    uint32_t duty = (static_cast<uint32_t>(duty_pct) * max_duty) / 100u;

    bool en_l = false, en_r = false;
    switch (dir) {
        case Direction::STOP:
            duty = 0;
            break;
        case Direction::DOWN:
            if (down_is_en_l_) en_l = true; else en_r = true;
            break;
        case Direction::UP:
            if (down_is_en_l_) en_r = true; else en_l = true;
            break;
    }
    digitalWrite(en_l_, en_l ? HIGH : LOW);
    digitalWrite(en_r_, en_r ? HIGH : LOW);
    ledcWrite(PWM_CHANNEL, duty);
}

bool WinchH::at_home() const {
    const int v = digitalRead(limit_);
    return limit_active_low_ ? (v == LOW) : (v == HIGH);
}

int WinchH::raw_limit() const {
    return digitalRead(limit_);
}
