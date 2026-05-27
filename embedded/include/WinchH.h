// WinchH — H-bridge driver for the Elmetron probe winch.
//
// The probe hangs on a brushed DC motor driven by an external H-bridge
// module on the H_BRIDGE1 connector (the board only routes the signals;
// the bridge IC is off-board, so the exact EN semantics aren't on the
// schematic — see ASSUMPTION below). Per the board pinout:
//
//     H_EN_L = IO6   direction-select A
//     H_EN_R = IO7   direction-select B
//     H_PWM  = IO15  speed (10 kHz PWM, matching the legacy winch)
//     H_LIMIT= IO12  home limit switch (LIMIT_E connector, switch→GND)
//
// !!! UNVERIFIED — no Elmetron hardware on hand. Direction polarity and
// limit-switch polarity are documented assumptions to confirm on the
// bench (one-line flips below). The FSM's safety-cap timeouts are the
// backstop if a polarity is wrong (motor stops on cap → FAULT).

#pragma once

#include <stdint.h>

class WinchH {
public:
    enum class Direction : uint8_t { STOP, DOWN, UP };

    WinchH(int en_l_pin, int en_r_pin, int pwm_pin, int limit_pin);

    // Configure pins + the LEDC PWM channel. Call from setup().
    void begin();

    // Drive the motor. duty_pct 0..100. STOP forces both enables low
    // and 0 duty. Idempotent — safe to call repeatedly.
    void drive(Direction dir, uint8_t duty_pct);
    void stop() { drive(Direction::STOP, 0); }

    // True when the home limit switch is engaged (cable fully retracted).
    bool at_home() const;
    int  raw_limit() const;   // diagnostic: unfiltered digitalRead

    // Bring-up flips, provisioned from the GCS so polarity can be
    // corrected without a reflash:
    //   invert=false → DOWN = EN_L high; true → DOWN = EN_R high.
    void set_direction_invert(bool invert) { down_is_en_l_ = !invert; }
    //   active_low=true → limit engaged reads LOW (switch-to-GND + pull-up).
    void set_limit_active_low(bool active_low) { limit_active_low_ = active_low; }

    static constexpr uint32_t PWM_FREQ_HZ  = 10000;  // legacy ran 10 kHz
    static constexpr uint8_t  PWM_RES_BITS = 8;      // 0..255 duty
    static constexpr int      PWM_CHANNEL  = 7;      // LEDC ch (nothing else uses LEDC)

    // Defaults (verify on bench): DOWN drives EN_L high; limit is
    // active-low. Both runtime-overridable via the setters above.
    static constexpr bool DOWN_IS_EN_L_DEFAULT    = true;
    static constexpr bool LIMIT_ACTIVE_LOW_DEFAULT = true;

private:
    const int en_l_;
    const int en_r_;
    const int pwm_;
    const int limit_;
    bool down_is_en_l_     = DOWN_IS_EN_L_DEFAULT;
    bool limit_active_low_ = LIMIT_ACTIVE_LOW_DEFAULT;
};
