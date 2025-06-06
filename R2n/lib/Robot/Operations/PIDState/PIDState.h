//
// Created by xantanium on 19/5/25.
//
#pragma once

struct PIDState {
    float locoW, locoY, locoX, turretOut;
#ifdef TUNE_PID
    void tunePid(PID& obj);
    void testPid(float setPoint, char dof);
#endif
};