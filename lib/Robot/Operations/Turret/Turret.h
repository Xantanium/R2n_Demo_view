//
// Created by xantanium on 19/5/25.
//
#pragma once

#define TURRET_ENC_PPR 286
#define TURRET_ENC_RES (TURRET_ENC_PPR / 360.0f)

struct TurretYaw {
    long count;
    float angle;
    float setpoint;
    int velocity;
    bool useJoyForMovement : 1;

    void turretTask() const;

private:
    void moveTurret();
};

extern TurretYaw turret;
