#pragma once

#include "../Primitives.h"
#include <Arduino.h>
#include <ArxContainer.h>

namespace rudra {

class ThreeXYEncoder {
private:
    // Encoder configuration
    float wheelSeperation, wheelDiameter;
    int encoderPPR;
    float mmsPerPulse;

    struct Config {
        Arena arena;
        Pose pose;
        float wheelSep;
        float wheelD;
        int encPPR;
    };

    // Calculation
    double rawX, rawYR, rawYL;
    double deltaYaw, dXLocal, dYLocal;
    double dXGlobal, dYGlobal;

    double xCorrection, xCorrected;

public:
    Arena odomArena;
    Pose odomPose;
    Config config;
    // Constructor with required parameters for odometry.
    explicit ThreeXYEncoder(
        // Distance between opposite Y wheels
        float _wheelSep,

        // Diameter of Encoder wheel
        float _wheelDiam,

        // Pulse per rotation of odometry encoder
        int _encPPR,

        // Starting position for the robot in arena
        const Pose& _initPose,

        // Arena dimensions
        const Arena& _arena)
        : wheelSeperation(_wheelSep)
        , wheelDiameter(_wheelDiam)
        , encoderPPR(_encPPR)
        , odomArena(_arena)
        , odomPose(_initPose)
    {
        mmsPerPulse = (2 * M_PI * wheelDiameter) / encoderPPR;
    }

    explicit ThreeXYEncoder(Config& conf)
        : wheelSeperation(conf.wheelSep)
        , wheelDiameter(conf.wheelD)
        , encoderPPR(conf.encPPR)
        , odomArena(conf.arena)
        , odomPose(conf.pose)
    {
        mmsPerPulse = (2 * M_PI * wheelDiameter) / encoderPPR;
    }

    ///////////////////////////// ALGORITHMS /////////////////////////////
    Pose update(int lidarLeft, int lidarRight, long count_Y_R, long count_Y_L, float yaw);
    Pose update(long countX, long count_Y_R, long count_Y_L, float yaw);
};

} // namespace rudra
