//
// Created by xantanium on 19/5/25.
//
#pragma once

struct DefenceState {
    enum extensionState { extend,
        stop,
        retract } state;

    void defenceTask();
};
