#include "pid_ctr.h"
#include "config.h"
#include <math.h>

PID:: PID(): k(0),Ti(0),Td(0),error_integral(0),prev_error(0),max_integral_error(200)
{
}

PID:: PID(float k, float Ti, float Td): k(k),Ti(Ti),Td(Td),error_integral(0),prev_error(0),max_integral_error(4000)
{
}

float PID::update(float setpoint, float fback){
    float control_output = 0;
    float err = setpoint - fback;
    error_integral += err*dt;
    if(abs(error_integral) > max_integral_error){
        error_integral = (error_integral>0)?
         max_integral_error : -max_integral_error;
    }

    control_output = k*err + Ti*error_integral + Td*(err - prev_error)/dt;
    prev_error = err;
    return control_output;

}

void PID::reset() {
    error_integral = 0;
    prev_error = 0;
}