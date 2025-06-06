//
// Created by xantanium on 19/5/25.
//
#pragma once
#include <Algos/Odometry/ThreeXYEncoder/ThreeXYEncoder.hpp>
#include "Algos/Odometry/Primitives.h"

struct OdometryState {
    float xSetpoint, ySetpoint, wSetpoint;
    float yaw;

    void odomTask();
};
