#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "driver/mcpwm.h"
#include "config.h"
#include "pinout_def.h"

void init_motor_safe(const motor_pwm& motor, const mcpwm_config_t& cfg);
void motor_stop(const motor_pwm& motor);
void motor_arm(const motor_pwm& motor, bool arm);
bool motor_is_armed();

void update_speed(const motor_pwm& motor, 
                  float control_signal);


#endif