#ifndef CONTROLLER_ENUMS_H
#define CONTROLLER_ENUMS_H
//#define ORIGINAL // Add this if using Original Xbox 360 receiver

#ifndef ORIGINAL
enum ButtonEnum
{
    X = 0x40,
    Y = 0x80,
    A = 0x10,
    B = 0x20,
    START = 0x1000,
    BACK = 0x2000,
    XBOX = 0x4,
    UP = 0x100,
    DOWN = 0x200,
    RIGHT = 0x800,
    LEFT = 0x400,
    L1 = 0x01,
    R1 = 0x02,
    L3 = 0x4000,
    R3 = 0x8000
};
#else
enum ButtonEnum{
    X = 0x4000,
    Y = 0x8000,
    A = 0x1000,
    B = 0x2000,
    START = 0x10,
    BACK = 0x20,
    XBOX = 0x400,
    UP = 0x01,
    DOWN = 0x02,
    RIGHT = 0x08,
    LEFT = 0x04,
    L1 = 0x100,
    R1 = 0x200,
    L3 = 0x40,
    R3 = 0x80
};
#endif

enum JoyEnum
{
    LeftHatX = 0,
    LeftHatY = 1,
    RightHatX = 2,
    RightHatY = 3,
    L2 = 4,
    R2 = 5
};


#endif
