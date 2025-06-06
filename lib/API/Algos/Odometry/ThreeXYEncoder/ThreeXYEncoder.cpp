#include "ThreeXYEncoder.hpp"

namespace rudra {

    /// @brief Updates distances based on encoder counts
    /// @param countX x encoder count
    /// @param count_Y_R right y encoder count
    /// @param count_Y_L left y encoder count
    /// @param yaw external yaw from IMU
    /// @return Pose object
    Pose ThreeXYEncoder::update(long countX, long count_Y_R, long count_Y_L, float yaw) {
        // Convert counts to distances
        rawX = countX * mmsPerPulse.X;
        rawYR = count_Y_R * mmsPerPulse.X;
        rawYL = count_Y_L * mmsPerPulse.X;

        // calculate yaw movement

        /**
         * This comes from:
         *
         * Theta = arcLength / radius.
         *
         * arclength is calculated by the simultaneous movements of opposite y encoders.
         * arcLength = (y1movement - y2movement) / 2.
         * e.g.: y1 moves 10, hence y2 moves -10. therefore net movement is
         * ((10) - (-10)) / 2 = 10.
         *
         * radius = wheelSeperation / 2
         *
         * combining arclength and radius, 2s in denominators cancel out to give below expression.*/
        deltaYaw = (rawYR - rawYL) / wheelSeperation;

        // Calculate X correction

        /**
         * The X correction works like this:
         *
         * While the robot is rotating about its axis, x encoder also traces an arc.
         * Manipulating the above formula, the arc traced by X encoder can be calculated by:
         *
         * xArcLength = theta * radius.
         *
         * X encoder readings obtained while rotation are garbage for net x movement.
         * Hence, we subtract the x arc length from the raw x to obtain net x movement. */
        xCorrection = deltaYaw * (wheelSeperation / 2);
        xCorrected = rawX - xCorrection;

        // calculate local changes in positions
        dXLocal = xCorrected;
        dYLocal = (rawYR + rawYR) / 2.0f; // average of both Y encs

        // Globalization
        dXGlobal = (dXLocal * std::cos(deltaYaw)) - (dYLocal * std::sin(deltaYaw));

        dYGlobal = (dXLocal * std::sin(deltaYaw)) + (dYLocal * std::cos(deltaYaw));

        // Append to pose
        odomPose.x += dXGlobal;
        odomPose.y += dYGlobal;

        odomPose.w += deltaYaw;

        // Normalize the angle
        odomPose.w = fmod(odomPose.w, 2 * M_PI);
        if (odomPose.w >= M_PI) {
            odomPose.w -= 2 * M_PI;
        } else if (odomPose.w < -M_PI) {
            odomPose.w += 2 * M_PI;
        }

        return odomPose;
    }

    Pose ThreeXYEncoder::update(int lidarLeft, int lidarRight, long count_Y_R, long count_Y_L, float yaw) {
#define EXT_YAW
/*
 * First we calculate the change in yaw of the robot, maybe IMU can be used for that? demn*/
#if defined(EXT_YAW)

        static float prevYaw;
        static float deltaYaw = yaw - prevYaw;

        prevYaw = yaw;
#else
        deltaYaw = (rawYR - rawYL) / wheelSeperation;
#endif
        /*
         * Lidar Odom:
         * */
        dYLocal = (rawYR + rawYR) / 2.0f; // average of both Y encs
    }

} // namespace rudra
