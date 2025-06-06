//
// Created by xantanium on 19/5/25.
//
#pragma once

#include <cstdint>
#include "constants.h"

#ifdef R1_pins
#include "r1_pins.h"
#else
#include "r2_pins.h"
#endif // R1_pins
#define TURRET_ENC_PPR 286
#define TURRET_ENC_RES (TURRET_ENC_PPR / 360.0f)


struct TurretYaw {
    long count;
    float angle;
    float setpoint;
    int velocity;
    bool useJoyForMovement : 1;
    bool runManually : 1;

    void turretTask();

private:
    void moveTurret(const int &velocity) const;
};

extern TurretYaw turret;
