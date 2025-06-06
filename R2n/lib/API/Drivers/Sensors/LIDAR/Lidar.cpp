#include "Lidar.h"

void rudra::Lidar::begin(int baud)
{
    lidar.begin(baud);
}

int rudra::Lidar::distance()
{
    if (lidar.available() >= 9) {
        lidar.readBytes(data, 9);
        if (data[0] == 0x59 && data[1] == 0x59 && checksum()) {
            return (data[3] << 8) + data[2];
        } else
            return -1;
    }
    return -1;
}

bool rudra::Lidar::checksum()
{
    uint8_t checksum = 0;
    checksum = data[0]
        + data[1]
        + data[2]
        + data[3]
        + data[4]
        + data[5]
        + data[6]
        + data[7];

    checksum &= 0xff;

    return checksum == data[8];
}
