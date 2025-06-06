#pragma once

#include "../Odometry/Primitives.h"
#include "ShooterData.h"

#include <cmath>

namespace rudra {
class Projectile {
private:
    int ppm();
    float angle();
    float projectileAngle();

    Pose hoopRelativePose;

    float hoopX;
    float hoopY;

    // distance of bot relative to hoop
    float botHoopX;
    float botHoopY;

    void convertToHoopRelative(Pose& botPose);

    float distanceFromHoop;

public:
    void getProjectile(ShooterData& shooter, Pose& botPose);
};

} // namespace rudra
