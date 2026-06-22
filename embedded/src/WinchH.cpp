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

void WinchH::brake() {
    // IBT-2 (BTS7960 dual half-bridge) wiring as best-guess: H_PWM is
    // the tied R_EN+L_EN (module-enable), and H_EN_L / H_EN_R are the
    // direction-specific PWM signals (RPWM / LPWM). For BTS7960 brake
    // mode the module must be ENABLED (ENs high) with BOTH direction
    // PWMs LOW — this shorts the motor through both low-side FETs.
    //   ENs (H_PWM)  = full duty (logical HIGH)
    //   RPWM (en_l_) = 0
    //   LPWM (en_r_) = 0
    // First commit f0f532d had this inverted (ENs LOW + PWM full = coast)
    // — corrected here per IBT-2 datasheet.
    digitalWrite(en_l_, LOW);
    digitalWrite(en_r_, LOW);
    const uint32_t max_duty = (1u << PWM_RES_BITS) - 1;
    ledcWrite(PWM_CHANNEL, max_duty);
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
