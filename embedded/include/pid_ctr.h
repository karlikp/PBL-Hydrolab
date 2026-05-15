#ifndef PID_CTR_H
#define PID_CTR_H

class PID
{
    public:

        PID();
        PID(float k, float Ti, float Td);

        float update(float setpoint, float fback);
        void tune(float k, float Ti, float Td);
        void reset();

    private:

        float k = 0;
        float Ti = 0;
        float Td = 0;

        float error_integral = 0;
        float max_integral_error = 0;
        float prev_error = 0;
       
};

#endif 