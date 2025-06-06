//
// Created by xantanium on 19/5/25.
//

#pragma once

struct Dribbler {
    unsigned long dribbleTimer, buttonDebounce;
    bool isBallInRange : 1;
    bool isClawOpen : 1;
    bool manual : 1;

    void dribbleTask(unsigned long);

private:
    void dribbleAuto(unsigned long);
    void dribbleManual();
};