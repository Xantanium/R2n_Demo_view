#pragma once

#include <Wire.h>

#define BNO_ADDRESS 0x28
#define OPR_MODE_REGISTER 0x3D
#define UNIT_SEL_REGISTER 0x3B

#define NDOF_OPR_MODE 0x0C // 0b----1100
#define STD_UNITS 0x00 //

// #define YAW_MSB_REGISTER 0x1B
#define YAW_LSB_REGISTER 0x1A

namespace rudra {

/**
 * @class BNO
 * @brief Class to interface with the BNO sensor.
 */
class BNO {
private:
    int m_rawYaw; ///< Raw yaw value from the sensor
    int m_rawPitch; ///< Raw pitch value from the sensor
    int m_rawRoll; ///< Raw roll value from the sensor

public:
    /**
     * @brief Initialize the BNO sensor.
     *
     * @param wire Reference to the TwoWire object for I2C communication.
     */
    void bnoInit(TwoWire& wire) const;

    /**
     * @brief Get the yaw value from the BNO sensor.
     *
     * @param wire Reference to the TwoWire object for I2C communication.
     * @return Yaw value in degrees.
     */
    float yaw(TwoWire& wire);

    /// @brief Maps to
    /// @param val
    void normalize(float& val);

    // Additional functions for pitch and roll can be added here
    // float pitch(TwoWire& wire);
    // float roll(TwoWire& wire);
};

} // namespace rudra
