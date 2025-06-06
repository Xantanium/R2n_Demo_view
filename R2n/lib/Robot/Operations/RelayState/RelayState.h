//
// Created by xantanium on 19/5/25.
//
#pragma once

struct RelayState {
    bool isRelayOn : 1;
    bool doAtRelayOnce : 1;

    void relayTask();
};
