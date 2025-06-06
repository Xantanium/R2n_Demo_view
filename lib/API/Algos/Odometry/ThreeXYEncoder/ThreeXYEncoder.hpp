#pragma once

#include <Arduino.h>
#include <ArxContainer.h>
#include "../Primitives.h"


namespace rudra {
    class ThreeXYEncoder {
    private:
        // Encoder configuration
        float wheelSeperation = 510.40f, wheelDiameter;
        struct mmPp {
            float YL, X, YR;
        } mmsPerPulse;

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
        struct PPRs {
            int YL, X, YR;
            PPRs(int YL, int X, int YR) : YL(YL), X(X), YR(YR) { }
        } pprs;
        Arena odomArena;
        Pose odomPose;
        Config config;

        ThreeXYEncoder(float wheelSep, float wheelDiam, PPRs &ppr) :
            wheelSeperation(wheelSep), wheelDiameter(wheelDiam), pprs(ppr) {
            mmsPerPulse.X = (2 * M_PI * wheelDiameter) / ppr.X;
            mmsPerPulse.YL = (2 * M_PI * wheelDiameter) / ppr.YL;
            mmsPerPulse.YR = (2 * M_PI * wheelDiameter) / ppr.YR;
        }
        ///////////////////////////// ALGORITHMS /////////////////////////////
        Pose update(int lidarLeft, int lidarRight, long count_Y_R, long count_Y_L, float yaw);
        Pose update(long countX, long count_Y_R, long count_Y_L, float yaw);
    };
} // namespace rudra
