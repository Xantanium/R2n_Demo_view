//
// Created by xantanium on 19/5/25.
//
#pragma once

struct Motion {
    int stickX, stickY, stickW; // Joystick values
    float valX, valY, valW; // Motion vectors
    float maSpeed, mbSpeed, mcSpeed, mdSpeed; // Motor speeds
    float maCap, mbCap, mcCap, mdCap; // Motor PWM caps
    bool invertAxis : 1; // Axis inversion flag
    bool mpUse : 1; // Yaw PID flag

    void drive(int argX, int argY, int argW);
    void getSpeed();
    void setMotion();
};

extern Motion motion;
