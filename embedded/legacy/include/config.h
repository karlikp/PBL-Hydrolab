#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include "driver/mcpwm.h"


// --- SETTINGS ---
#define CX_BAUD      115200
#define RADIO_BAUD   57600
#define GPS_HZ       1
#define BROADCAST_INTERVAL_MS 500

const uint8_t resolution = 10;
const uint16_t  pls_per_r = 546;
const int r_channel = 0;
const int l_channel = 1;
const uint16_t dt = 20; // In millis
const float  dist_per_r = 71.75; //512 pulses per revolution and 71.75 mm
const uint8_t dbnc_time = 100;

// PWM generator config
const int pwm_freq = 10000; // 10kHz

//----------------------------------------------------
// STRUCTS
//----------------------------------------------------
 struct motor_pwm{
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_io_signals_t sigA;
    mcpwm_io_signals_t sigB;
    mcpwm_duty_type_t  duty_type;
 };

typedef enum {
    IDLE,
    HOMING,
    RUN,
    MEASURE
} STATE;


#endif