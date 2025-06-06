//
// Created by xantanium on 19/5/25.
//
#pragma once

struct Thrower {
    bool isThrowing : 1;
    int throwSpeed;

    bool throwTask();

private:
    void throwBall();
};
