/*
 * PhysicsBody.cpp
 */

#include "PhysicsBody.h"

namespace BulletPhysics {
namespace projectile {

// ProjectileSpecs

double ProjectileSpecs::calculateArea(double diameter)
{
    // cross-sectional area: S = pi * d ^ 2 / 4
    return math::constants::PI * diameter * diameter * 0.25;
}

double ProjectileSpecs::calculateMomentOfInertiaX(double mass, double diameter)
{
    // uniform cylinder approximation: Ix = 1/8 * m * d^2
    return 0.125 * mass * diameter * diameter;
}

double ProjectileSpecs::calculateSpinRate(double velocity, double twistRate, double diameter)
{
    // spin rate: p = 2 * pi * V / (n * d)
    return 2.0 * math::constants::PI * velocity / (twistRate * diameter);
}

} // namespace projectile
} // namespace BulletPhysics
