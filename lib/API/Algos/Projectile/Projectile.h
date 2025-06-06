#pragma once

#include "../Odometry/Primitives.h"
#include "ShooterData.h"

#include <cmath>
#include "maths.h"

namespace rudra {
    class Projectile {
    private:
        // Private getters to return stuff into `ShooterData` struct instance
        int ppm();
        float angle(const float &);
        float projectileAngle();

        Pose hoopRelativePose;

        const float hoopX = 0.0f;
        const float hoopY = 0.0f;

        // distance of bot relative to hoop
        float botHoopX;
        float botHoopY;

        void convertToHoopRelative(const Pose &botPose);

        float distanceFromHoop;

    public:
        void getProjectile(ShooterData &shooter, const Pose &botPose);
    };

} // namespace rudra
