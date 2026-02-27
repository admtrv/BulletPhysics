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
float criticalAngle(float mass, float speed, float diameter, float yieldStrength)
{
    // tan^3(theta_crit) + tan(theta_crit) = 8 * m * v^2 / (3 * sigma * pi * D^3)

    float d3 = diameter * diameter * diameter;
    float denom = 3.0f * yieldStrength * math::constants::PI * d3;

    if (denom < 1e-12f)
        return 0.0f;

    float rhs = 8.0f * mass * speed * speed / denom;

    // solve tan^3(t) + t = rhs for t = tan(theta) via Newton's
    float t = std::cbrt(rhs);   // initial guess
    for (int i = 0; i < 8; ++i)
    {
        float f = t * t * t + t - rhs;
        float df = 3.0f * t * t + 1.0f;

        t -= f / df;

        if (t < 0.0f)
            t = 0.0f;
    }
    float thetaCritical = std::atan(t);

    // convert to grazing angle
    float alphaCritical = math::constants::PI * 0.5f - thetaCritical;

    return std::clamp(alphaCritical, 0.0f, math::constants::PI * 0.5f);
}

// penetration depth (meters), energy-based model
float maxPenetrationDepth(float mass, float speed, float area, float penetrationResistance)
{
    float resistForce = penetrationResistance * area;

    if (resistForce < 1e-9f)
        return 1e6f;

    float kineticEnergy = 0.5f * mass * speed * speed;
    return kineticEnergy / resistForce;
}

// residual velocity after penetration, energy-based model
float residualSpeed(float speed, float thickness, float maxPen)
{
    if (maxPen < 1e-9f)
        return 0.0f;

    float ratio = thickness / maxPen;
    if (ratio >= 1.0f)
        return 0.0f;

    return speed * std::sqrt(1.0f - ratio);
}

} // anonymous namespace

ImpactResult Impact::resolve(const dynamics::projectile::IProjectileBody& projectile, const ImpactInfo& info)
{
    const auto& specs = projectile.getProjectileSpecs();
    math::Vec3 velocity = projectile.getVelocity();
    float speed = velocity.length();

    if (speed < 1e-6f)
    {
        return {ImpactOutcome::Embed, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f};
    }

    const Material& material = info.material;

    float diameter = specs.diameter.value_or(constants::DEFAULT_DIAMETER);
    float area = specs.area.value_or(constants::DEFAULT_AREA);
    float mass = specs.mass;

    // E_k = 0.5 * m * v^2
    float kineticEnergy = 0.5f * mass * speed * speed;

    // surface normal
    math::Vec3 normal = info.normal.normalized();

    // decompose velocity
    float vDotN = velocity.dot(normal);
    math::Vec3 vNormal = normal * vDotN;
    math::Vec3 vTangent = velocity - vNormal;

    // grazing angle
    float grazingAngle = std::asin(std::clamp(std::abs(vDotN) / speed, 0.0f, 1.0f));

    // ricochet check
    float alphaAngle = criticalAngle(mass, speed, diameter, material.yieldStrength);
    if (grazingAngle < alphaAngle)
    {
        // step 1: classical mechanics: restitution on normal + Coulomb friction on tangential
        float vNormalMag = std::abs(vDotN);
        float vTangentMag = vTangent.length();

        // reflected normal
        math::Vec3 reflectedNormal = normal * (-vDotN * material.restitutionN);

        // Coulomb friction: max tangential impulse change
        float maxFrictionDv = material.frictionT * (1.0f + material.restitutionN) * vNormalMag;
        float newTangentMag = std::max(vTangentMag - maxFrictionDv, 0.0f);

        math::Vec3 tangentDir = (vTangentMag > 1e-9f) ? vTangent * (1.0f / vTangentMag) : math::Vec3{0.0f, 0.0f, 0.0f};
        math::Vec3 reflectedTangent = tangentDir * newTangentMag;

        math::Vec3 classicalVelocity = reflectedTangent + reflectedNormal;
        float classicalSpeed = classicalVelocity.length();
        float classicalEnergy = 0.5f * mass * classicalSpeed * classicalSpeed;

        // step 2: empirical correction: apply only "missing" losses
        float targetEnergy = kineticEnergy * (1.0f - material.empiricalEnergyLoss);
        math::Vec3 residualVelocity = classicalVelocity;

        if (classicalEnergy > targetEnergy && classicalSpeed > 1e-9f)
        {
            float correction = std::sqrt(targetEnergy / classicalEnergy);
            residualVelocity = classicalVelocity * correction;
        }

        float finalSpeed = residualVelocity.length();
        float energyAbsorbed = kineticEnergy - 0.5f * mass * finalSpeed * finalSpeed;

        return {ImpactOutcome::Ricochet, residualVelocity, std::max(energyAbsorbed, 0.0f), 0.0f};
    }

    // penetration calculation
    float penetrationThickness = maxPenetrationDepth(mass, speed, area, material.penetrationResistance);

    if (penetrationThickness < info.thickness)
    {
        return {ImpactOutcome::Embed, {0.0f, 0.0f, 0.0f}, kineticEnergy, penetrationThickness};
    }

    // full penetration
    float vRes = residualSpeed(speed, info.thickness, penetrationThickness);
    math::Vec3 direction = velocity.normalized();
    math::Vec3 residualVelocity = direction * vRes;
    float residualEnergy = 0.5f * mass * vRes * vRes;

    return {ImpactOutcome::Penetration, residualVelocity, kineticEnergy - residualEnergy, info.thickness};
}

} // namespace terminal
} // namespace collision
} // namespace BulletPhysics
