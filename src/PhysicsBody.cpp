/*
 * PhysicsBody.cpp
 */

#include "PhysicsBody.h"

namespace BulletPhysics {
namespace projectile {

// ProjectileSpecs

ProjectileSpecs ProjectileSpecs::create(double mass, double diameter)
{
    ProjectileSpecs specs{};
    specs.mass = mass;
    specs.diameter = diameter;
    specs.area = calculateArea(diameter);
    return specs;
}

ProjectileSpecs& ProjectileSpecs::withDragModel(ballistics::external::forces::drag::DragCurveModel model)
{
    dragModel = std::make_shared<ballistics::external::forces::drag::StandardDragModel>(model);
    return *this;
}

ProjectileSpecs& ProjectileSpecs::withCustomDragCoefficient(double C_d)
{
    dragModel = std::make_shared<ballistics::external::forces::drag::CustomDragModel>(C_d);
    return *this;
}

ProjectileSpecs& ProjectileSpecs::withMuzzle(double muzzleVelocity, Direction riflingDirection, double twistRate)
{
    muzzleSpecs = MuzzleSpecs{};
    muzzleSpecs->velocity = muzzleVelocity;
    muzzleSpecs->riflingDirection = riflingDirection;
    muzzleSpecs->twistRate = twistRate;

    muzzleSpecs->momentOfInertia = calculateMomentOfInertiaX(mass, diameter);
    muzzleSpecs->spinRate = calculateSpinRate(muzzleVelocity, twistRate, diameter);

    return *this;
}

ProjectileSpecs& ProjectileSpecs::withOvertuningCoefficient(double C_M_alpha)
{
    if (!muzzleSpecs.has_value())
    {
        return *this;
    }

    muzzleSpecs->overtuningCoefficient = C_M_alpha;
    return *this;
}

ProjectileSpecs& ProjectileSpecs::withLiftCoefficient(double C_L_alpha)
{
    if (!muzzleSpecs.has_value())
    {
        return *this;
    }

    muzzleSpecs->liftCoefficient = C_L_alpha;
    return *this;
}

ProjectileSpecs& ProjectileSpecs::withMagnusCoefficient(double C_mag_f)
{
    if (!muzzleSpecs.has_value())
    {
        return *this;
    }

    muzzleSpecs->magnusCoefficient = C_mag_f;
    return *this;
}


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

namespace presets {

ProjectileSpecs Sphere(double mass, double diameter)
{
    return ProjectileSpecs::create(mass, diameter)
        .withCustomDragCoefficient(0.47);
}

ProjectileSpecs Nato762()
{
    return ProjectileSpecs::create(0.00952, 0.00762)                                // 9.52 g, 7.62 mm
        .withDragModel(ballistics::external::forces::drag::DragCurveModel::G7)
        .withMuzzle(838.0, Direction::RIGHT, 12.0);                                 // 838 m/s, right-hand rifling, 12 cal/turn

}

} // namespace presets
} // namespace projectile
} // namespace BulletPhysics
