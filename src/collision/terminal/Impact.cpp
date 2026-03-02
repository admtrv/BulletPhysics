/*
 * Impact.cpp
 */

#include "Impact.h"
#include "Constants.h"
#include "math/Angles.h"
#include "math/Constants.h"

#include <cmath>
#include <algorithm>

namespace BulletPhysics {
namespace collision {
namespace terminal {

namespace {

// critical grazing angle for ricochet (radians from surface), Wijk
double criticalAngle(double mass, double speed, double diameter, double yieldStrength)
{
    // tan^3(theta_crit) + tan(theta_crit) = 8 * m * v^2 / (3 * sigma * pi * D^3)

    double d3 = diameter * diameter * diameter;
    double denom = 3.0 * yieldStrength * math::constants::PI * d3;

    if (denom < 1e-12)
        return 0.0;

    double rhs = 8.0 * mass * speed * speed / denom;

    // solve tan^3(t) + t = rhs for t = tan(theta) via Newton's
    double t = std::cbrt(rhs);   // initial guess
    for (int i = 0; i < 8; ++i)
    {
        double f = t * t * t + t - rhs;
        double df = 3.0 * t * t + 1.0;

        t -= f / df;

        if (t < 0.0)
            t = 0.0;
    }
    double thetaCritical = std::atan(t);

    // convert to grazing angle
    double alphaCritical = math::constants::PI * 0.5 - thetaCritical;

    return std::clamp(alphaCritical, 0.0, math::constants::PI * 0.5);
}

// penetration depth (meters), energy-based model
double maxPenetrationDepth(double mass, double speed, double area, double penetrationResistance)
{
    double resistForce = penetrationResistance * area;

    if (resistForce < 1e-9)
        return 1e6;

    double kineticEnergy = 0.5 * mass * speed * speed;
    return kineticEnergy / resistForce;
}

// residual velocity after penetration, energy-based model
double residualSpeed(double speed, double thickness, double maxPen)
{
    if (maxPen < 1e-9)
        return 0.0;

    double ratio = thickness / maxPen;
    if (ratio >= 1.0)
        return 0.0;

    return speed * std::sqrt(1.0 - ratio);
}

} // anonymous namespace

ImpactResult Impact::resolve(const dynamics::projectile::IProjectileBody& projectile, const ImpactInfo& info)
{
    const auto& specs = projectile.getProjectileSpecs();
    math::Vec3 velocity = projectile.getVelocity();
    double speed = velocity.length();

    if (speed < 1e-6)
    {
        return {ImpactOutcome::Embed, {0.0, 0.0, 0.0}, 0.0, 0.0};
    }

    const Material& material = info.material;

    double diameter = specs.diameter.value_or(constants::DEFAULT_DIAMETER);
    double area = specs.area.value_or(constants::DEFAULT_AREA);
    double mass = specs.mass;

    // E_k = 0.5 * m * v^2
    double kineticEnergy = 0.5 * mass * speed * speed;

    // surface normal
    math::Vec3 normal = info.normal.normalized();

    // decompose velocity
    double vDotN = velocity.dot(normal);
    math::Vec3 vNormal = normal * vDotN;
    math::Vec3 vTangent = velocity - vNormal;

    // grazing angle
    double grazingAngle = std::asin(std::clamp(std::abs(vDotN) / speed, 0.0, 1.0));

    // ricochet check
    double alphaAngle = criticalAngle(mass, speed, diameter, material.yieldStrength);
    if (grazingAngle < alphaAngle)
    {
        // step 1: classical mechanics: restitution on normal + Coulomb friction on tangential
        double vNormalMag = std::abs(vDotN);
        double vTangentMag = vTangent.length();

        // reflected normal
        math::Vec3 reflectedNormal = normal * (-vDotN * material.restitutionN);

        // Coulomb friction: max tangential impulse change
        double maxFrictionDv = material.frictionT * (1.0 + material.restitutionN) * vNormalMag;
        double newTangentMag = std::max(vTangentMag - maxFrictionDv, 0.0);

        math::Vec3 tangentDir = (vTangentMag > 1e-9) ? vTangent * (1.0 / vTangentMag) : math::Vec3{0.0, 0.0, 0.0};
        math::Vec3 reflectedTangent = tangentDir * newTangentMag;

        math::Vec3 classicalVelocity = reflectedTangent + reflectedNormal;
        double classicalSpeed = classicalVelocity.length();
        double classicalEnergy = 0.5 * mass * classicalSpeed * classicalSpeed;

        // step 2: empirical correction: apply only "missing" losses
        double targetEnergy = kineticEnergy * (1.0 - material.empiricalEnergyLoss);
        math::Vec3 residualVelocity = classicalVelocity;

        if (classicalEnergy > targetEnergy && classicalSpeed > 1e-9)
        {
            double correction = std::sqrt(targetEnergy / classicalEnergy);
            residualVelocity = classicalVelocity * correction;
        }

        double finalSpeed = residualVelocity.length();
        double energyAbsorbed = kineticEnergy - 0.5 * mass * finalSpeed * finalSpeed;

        return {ImpactOutcome::Ricochet, residualVelocity, std::max(energyAbsorbed, 0.0), 0.0};
    }

    // penetration calculation
    double penetrationThickness = maxPenetrationDepth(mass, speed, area, material.penetrationResistance);

    if (penetrationThickness < info.thickness)
    {
        return {ImpactOutcome::Embed, {0.0, 0.0, 0.0}, kineticEnergy, penetrationThickness};
    }

    // full penetration
    double vRes = residualSpeed(speed, info.thickness, penetrationThickness);
    math::Vec3 direction = velocity.normalized();
    math::Vec3 residualVelocity = direction * vRes;
    double residualEnergy = 0.5 * mass * vRes * vRes;

    return {ImpactOutcome::Penetration, residualVelocity, kineticEnergy - residualEnergy, info.thickness};
}

} // namespace terminal
} // namespace collision
} // namespace BulletPhysics
