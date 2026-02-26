/*
 * ImpactResolver.cpp
 */

#include "ImpactResolver.h"

namespace BulletPhysics {
namespace collision {
namespace terminal {

// alpha_crit + theta_crit = pi / 2
// alpha_crit - grazing angle, angle relative to surface
// theta_crit - angle relative to normal

float ImpactResolver::criticalAngle(float mass, float speed, float diameter, float yieldStrength)
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

float ImpactResolver::maxPenetrationDepth(float mass, float speed, float area, float penetrationResistance)
{
    // energy-based penetration model:
    //
    // the projectile pushes through the material like a cylinder.
    // the material resists with constant pressure P (penetrationResistance, Pa)
    // acting on the projectile cross-section area A.
    //
    // resisting force:  F = P * A
    // work to penetrate depth e:  W = F * e
    // projectile stops when all kinetic energy is spent:  E_k = W
    //
    //   0.5 * m * v^2 = P * A * e
    //   e = (0.5 * m * v^2) / (P * A)

    float resistForce = penetrationResistance * area;

    if (resistForce < 1e-9f)
        return 1e6f;

    float kineticEnergy = 0.5f * mass * speed * speed;
    return kineticEnergy / resistForce;
}

float ImpactResolver::residualSpeed(float speed, float thickness, float maxPen)
{
    // energy spent traversing thickness t:  E_lost = F * t
    // total energy to stop projectile:      E_total = F * e_max = 0.5 * m * v^2
    // residual energy after penetration:    E_res = E_total - E_lost = F * (e_max - t)
    //
    // from E_res = 0.5 * m * v_res^2 and E_total = 0.5 * m * v^2:
    // v_res^2 / v^2 = (e_max - t) / e_max = 1 - t / e_max
    // v_res = v * sqrt(1 - t / e_max)

    if (maxPen < 1e-9f)
        return 0.0f;

    float ratio = thickness / maxPen;
    if (ratio >= 1.0f)
        return 0.0f;

    return speed * std::sqrt(1.0f - ratio);
}

float ImpactResolver::computeEffectiveThickness(const math::Vec3& position, const math::Vec3& velocity, const Collider* collider)
{
    if (!collider)
    {
        return 1e6f; // no geometry, treat as infinitely thick
    }

    if (collider->getShape() == CollisionShape::Ground)
    {
        return 1e6f; // ground is infinitely thick
    }

    if (collider->getShape() != CollisionShape::Box)
    {
        return 1e6f;
    }

    const auto* box = static_cast<const BoxCollider*>(collider);
    math::Vec3 center = box->getPosition();
    math::Vec3 halfSize = box->getSize() * 0.5f;

    // ray direction
    math::Vec3 dir = velocity.normalized();

    // ray-AABB intersection (slab method)
    float tMin = -1e30f;
    float tMax = 1e30f;

    // x axis
    if (std::abs(dir.x) > 1e-9f)
    {
        float t1 = (center.x - halfSize.x - position.x) / dir.x;
        float t2 = (center.x + halfSize.x - position.x) / dir.x;

        if (t1 > t2)
            std::swap(t1, t2);

        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
    }
    else
    {
        if (position.x < center.x - halfSize.x || position.x > center.x + halfSize.x)
            return 1e6f;
    }

    // y axis
    if (std::abs(dir.y) > 1e-9f)
    {
        float t1 = (center.y - halfSize.y - position.y) / dir.y;
        float t2 = (center.y + halfSize.y - position.y) / dir.y;

        if (t1 > t2)
            std::swap(t1, t2);

        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
    }
    else
    {
        if (position.y < center.y - halfSize.y || position.y > center.y + halfSize.y)
            return 1e6f;
    }

    // z axis
    if (std::abs(dir.z) > 1e-9f)
    {
        float t1 = (center.z - halfSize.z - position.z) / dir.z;
        float t2 = (center.z + halfSize.z - position.z) / dir.z;

        if (t1 > t2)
            std::swap(t1, t2);

        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
    }
    else
    {
        if (position.z < center.z - halfSize.z || position.z > center.z + halfSize.z)
            return 1e6f;
    }

    if (tMax < tMin || tMax < 0.0f)
    {
        return 1e6f; // no intersection, treat as infinitely thick
    }

    float entry = std::max(tMin, 0.0f);
    return tMax - entry;
}

ImpactResult ImpactResolver::resolve(const dynamics::projectile::IProjectileBody& projectile, const CollisionInfo& info, const Collider& collider)
{
    const auto& materialOpt = collider.getMaterial();
    if (!materialOpt.has_value())
    {
        return {ImpactOutcome::Embed, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f};
    }

    const Material& material = materialOpt.value();

    const auto& specs = projectile.getProjectileSpecs();
    math::Vec3 velocity = projectile.getVelocity();
    float speed = velocity.length();

    if (speed < 1e-6f)
    {
        return {ImpactOutcome::Embed, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f};
    }

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
        //
        // normal component: reflected with coefficient of restitution e
        //   v_n' = -e * v_n
        //
        // tangential component: Coulomb friction impulse
        //   |Δv_t| ≤ μ * (1 + e) * |v_n|
        //   friction cannot reverse tangential direction, only reduce it

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
        //
        // experimental data gives target energy loss fraction for each material.
        // if classical mechanics already removed enough energy, no correction needed.
        // otherwise, scale velocity down to match empirical target.

        float targetEnergy = kineticEnergy * (1.0f - material.empiricalEnergyLoss);
        math::Vec3 residualVelocity = classicalVelocity;

        if (classicalEnergy > targetEnergy && classicalSpeed > 1e-9f)
        {
            // scale velocity to match empirical energy target
            // E_target = 0.5 * m * v_target^2
            // v_target = v_classical * sqrt(E_target / E_classical)
            float correction = std::sqrt(targetEnergy / classicalEnergy);
            residualVelocity = classicalVelocity * correction;
        }

        float residualSpeed = residualVelocity.length();
        float energyAbsorbed = kineticEnergy - 0.5f * mass * residualSpeed * residualSpeed;

        return {ImpactOutcome::Ricochet, residualVelocity, std::max(energyAbsorbed, 0.0f), 0.0f};
    }

    // effective thickness
    float thickness = computeEffectiveThickness(projectile.getPosition(), velocity, &collider);

    // penetration calculation
    float penetrationThickness = maxPenetrationDepth(mass, speed, area, material.penetrationResistance);

    if (penetrationThickness < thickness)
    {
        return {ImpactOutcome::Embed, {0.0f, 0.0f, 0.0f}, kineticEnergy, penetrationThickness};
    }

    // full penetration
    float vRes = residualSpeed(speed, thickness, penetrationThickness);
    math::Vec3 direction = velocity.normalized();
    math::Vec3 residualVelocity = direction * vRes;
    float residualEnergy = 0.5f * mass * vRes * vRes;

    return {ImpactOutcome::Penetration, residualVelocity, kineticEnergy - residualEnergy, thickness};
}

} // namespace terminal
} // namespace collision
} // namespace BulletPhysics
