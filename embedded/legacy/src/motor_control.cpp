#include "motor_control.h"
#include <math.h>

static volatile bool g_motorArmed = false;
static const float MOTOR_DEADBAND_PERCENT = 1.0f;

static float clampDuty(float value) {
    if (value > 100.0f) return 100.0f;
    if (value < -100.0f) return -100.0f;
    if (fabsf(value) < MOTOR_DEADBAND_PERCENT) return 0.0f;
    return value;
}

void motor_stop(const motor_pwm& motor) {
    mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_A, 0);
    mcpwm_set_duty_type(motor.unit, motor.timer, MCPWM_OPR_A, motor.duty_type);

    mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_B, 0);
    mcpwm_set_duty_type(motor.unit, motor.timer, MCPWM_OPR_B, motor.duty_type);

    digitalWrite(R_en, LOW);
    digitalWrite(L_en, LOW);
}

void init_motor_safe(const motor_pwm& motor, const mcpwm_config_t& cfg) {
    g_motorArmed = false;

    pinMode(R_en, OUTPUT);
    pinMode(L_en, OUTPUT);
    digitalWrite(R_en, LOW);
    digitalWrite(L_en, LOW);

    pinMode(R_s, OUTPUT);
    pinMode(L_s, OUTPUT);
    digitalWrite(R_s, LOW);
    digitalWrite(L_s, LOW);

    mcpwm_gpio_init(motor.unit, motor.sigA, R_s);
    mcpwm_gpio_init(motor.unit, motor.sigB, L_s);
    mcpwm_init(motor.unit, motor.timer, &cfg);

    motor_stop(motor);
}

void motor_arm(const motor_pwm& motor, bool arm) {
    if (!arm) {
        g_motorArmed = false;
        motor_stop(motor);
        return;
    }

    mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_A, 0);
    mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_B, 0);

    digitalWrite(R_en, HIGH);
    digitalWrite(L_en, HIGH);

    g_motorArmed = true;
}

bool motor_is_armed() {
    return g_motorArmed;
}

void update_speed(const motor_pwm& motor, float control_signal) {
    if (!g_motorArmed) {
        motor_stop(motor);
        return;
    }

    float speed = clampDuty(control_signal);

    if (speed > 0) {
        mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_B, 0);
        mcpwm_set_duty_type(motor.unit, motor.timer, MCPWM_OPR_B, motor.duty_type);

        mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_A, speed);
        mcpwm_set_duty_type(motor.unit, motor.timer, MCPWM_OPR_A, motor.duty_type);
    }
    else if (speed < 0) {
        mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_A, 0);
        mcpwm_set_duty_type(motor.unit, motor.timer, MCPWM_OPR_A, motor.duty_type);

        mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_B, -speed);
        mcpwm_set_duty_type(motor.unit, motor.timer, MCPWM_OPR_B, motor.duty_type);
    }
    else {
        mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_A, 0);
        mcpwm_set_duty_type(motor.unit, motor.timer, MCPWM_OPR_A, motor.duty_type);

        mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_B, 0);
        mcpwm_set_duty_type(motor.unit, motor.timer, MCPWM_OPR_B, motor.duty_type);
    }
}