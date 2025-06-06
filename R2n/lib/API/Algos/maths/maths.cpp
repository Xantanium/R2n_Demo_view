//
// Created by xantanium on 20/5/25.
//
#include "maths.h"

auto map1(const double x, const double in_min, const double in_max, const double out_min, const double out_max)
        -> double {
    return  (x-in_min) * (out_max-out_min)/(in_max-in_min)+out_min;
}
