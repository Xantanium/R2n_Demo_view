#include "Turret.h"
#include <cstdint>

void TurretYaw::moveTurret(const int &velocity) const {
#define TURRET_MAX 60
    digitalWrite(TURRET_YAW_DIR, velocity > 0 ? 1 : 0);
    analogWrite(TURRET_YAW_PWM, round(abs((velocity / 128.0f) * TURRET_MAX)));
}
