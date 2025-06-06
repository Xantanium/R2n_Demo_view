#pragma once
#include <Arduino.h>

class PID {
public:
    float kp, ki, kd;
    float imax, pmax;
    float output;

    PID(float KP, float KI, float KD, float PMAX, float IMAX);
    float get_pid(float error, float scaler);
    void reset_I();

private:
    unsigned long t_now, dt, last_t;

    float last_error;
    float last_derivative, derivative;
    float integrator;
    float delta_time;
    float RC;
};