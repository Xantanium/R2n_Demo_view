#include "Projectile.h"

int rudra::Projectile::ppm()
{
    return 0;
}

float rudra::Projectile::angle()
{
    return std::atan2(botHoopX, botHoopY) * (180.0f / M_PI); // Convert radians to degrees
}

float rudra::Projectile::projectileAngle()
{
    return 45.0f; // 45 degrees
}

void rudra::Projectile::convertToHoopRelative(Pose& botPose)
{
    botHoopX = hoopX - botPose.x;
    botHoopY = hoopY - botPose.y;
}

void rudra::Projectile::getProjectile(ShooterData& shooter, Pose& botPose)
{
    distanceFromHoop = std::sqrt(botPose.x * botPose.x + botPose.y * botPose.y);
    shooter.shootPPM = ppm();
    shooter.projectileAngle = projectileAngle();
    shooter.turretAngle = angle();
}
