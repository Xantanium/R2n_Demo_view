//
// Created by xantanium on 19/5/25.
//
#pragma once

struct OdometryState {
    float xSetpoint, ySetpoint, wSetpoint;
    float yaw;

    void odomTask();
};
