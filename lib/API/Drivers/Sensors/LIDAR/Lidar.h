#pragma once

#include <Arduino.h>

namespace rudra {
#define LIDAR_CHECKSUM 0x59
class Lidar {
public:
    Lidar(HardwareSerial& serial)
        : lidar(serial)
    {
    }

    void begin(int baud);
    int distance();

private:
    HardwareSerial& lidar;

    uint8_t data[9];
    bool checksum();
};

} // namespace rudra
