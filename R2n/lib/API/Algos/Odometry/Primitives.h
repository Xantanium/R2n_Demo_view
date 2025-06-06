#pragma once

namespace rudra {
// Pose Struct
struct Pose {
    double x, y, w;

    Pose(double _x = 0.0, double _y = 0.0, double _w = 0.0)
        : x(_x)
        , y(_y)
        , w(_w)
    {
    }
    /// @brief Does exactly what it claims to do.
    void setZeros()
    {
        this->x = 0;
        this->y = 0;
        this->w = 0;
    }
};

struct Arena {
    double width, length;
    Pose initialPose;

    Arena(double _width = 0.0, double _length = 0.0, Pose* _initialPose = new Pose())
        : width(_width)
        , length(_length)
        , initialPose(*_initialPose)
    {
    }
};

};
