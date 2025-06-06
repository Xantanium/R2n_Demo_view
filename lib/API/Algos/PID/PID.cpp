#include "PID.h"

PID::PID(const float KP, const float KI, const float KD, const float PMAX, const float IMAX) {
    kp = KP;
    ki = KI;
    kd = KD;
    imax = IMAX;
    pmax = PMAX;
}

float PID::get_pid(const float error, const float scaler) {
    t_now = millis();
    dt = t_now - last_t;
    output = 0.0f;
    if ((last_t == 0) || (dt > 1000)) {
        dt = 0.0f;
        integrator = 0.0f;
    }
    last_t = t_now;
    delta_time = static_cast<float>(dt) * 0.001f;
    output += (error * kp);
    if ((abs(kd) > 0) && (dt > 0)) {
        derivative = (error - last_error) / delta_time;

        last_error = error;
        output += (kd * derivative);
    }
    output *= scaler;
    if ((abs(ki) > 0) && dt > 0) {
        if (abs(output) < imax) {
            integrator += (error * ki) * scaler * delta_time;
            if (integrator < (-1 * imax)) {
                integrator = (-1 * imax);
            } else if (integrator > imax) {
                integrator = imax;
            }
            output += integrator;
        }
    }

    if (output < (-1 * pmax)) {
        output = (-1 * pmax);
    } else if (output > pmax) {
        output = pmax;
    }

    return output;
}

void PID::reset_I() { integrator = 0; }