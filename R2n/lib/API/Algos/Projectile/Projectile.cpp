#include "Projectile.h"

int rudra::Projectile::ppm() { return 0; }

float rudra::Projectile::angle(const float &currentW) {
    float targetAngle = std::atan2(botHoopY, botHoopX) * RAD_TO_DEG; // arctan(y/x)
    float relativeAngle = targetAngle - currentW;

    if (relativeAngle > 180.0f)
        relativeAngle -= 360.0f;
    if (relativeAngle < -180.0f)
        relativeAngle += 360.0f;

    return relativeAngle;
}

float rudra::Projectile::projectileAngle() { return 45.0f; }

void rudra::Projectile::convertToHoopRelative(const Pose &botPose) {
    botHoopX = hoopX - botPose.x;
    botHoopY = hoopY - botPose.y;
}

void rudra::Projectile::getProjectile(ShooterData &shooter, const Pose &botPose) {
    convertToHoopRelative(botPose);

    distanceFromHoop = std::sqrt(botHoopX * botHoopX + botHoopY * botHoopY);

    shooter.shootPPM = ppm(); // Placeholder
    shooter.projectileAngle = projectileAngle(); // Fixed 45° for now
    shooter.turretAngle = angle(botPose.w); // In degrees
}
