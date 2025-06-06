#include "BNO.h"

void rudra::BNO::bnoInit(TwoWire& wire) const
{
    wire.beginTransmission(BNO_ADDRESS);
    wire.write(OPR_MODE_REGISTER);
    wire.write(NDOF_OPR_MODE);
    wire.endTransmission();

    wire.beginTransmission(BNO_ADDRESS);
    wire.write(UNIT_SEL_REGISTER);
    wire.write(STD_UNITS);
    wire.endTransmission();
}

float rudra::BNO::yaw(TwoWire& wire)
{
    wire.beginTransmission(BNO_ADDRESS);
    wire.write(YAW_LSB_REGISTER);
    wire.endTransmission(false);
    wire.requestFrom(BNO_ADDRESS, 2);
    wire.endTransmission(false);
    uint8_t lsb = wire.read();
    uint8_t msb = wire.read();

    m_rawYaw = (msb << 8) | lsb;
    return static_cast<float>(m_rawYaw / 16.0f);
}
