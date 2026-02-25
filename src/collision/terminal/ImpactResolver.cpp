/*
 * ImpactResolver.cpp
 */

#include "ImpactResolver.h"

namespace BulletPhysics {
namespace collision {
namespace terminal {

// reference constants for critical angle formula
static constexpr float REF_DIAMETER = 0.00762f;  // 7.62mm reference caliber
static constexpr float REF_SPEED = 800.0f;       // m/s reference speed
static constexpr float REF_BHN = 300.0f;         // reference Brinell hardness (RHA)

float ImpactResolver::criticalAngle(float bhn, float diameter, float speed)
{
    // Zukas (1990): theta_crit ~ 2 * (BHN/300)^0.25 * (d_ref/d)^0.33 * (v_ref/v)^0.2
    float angle = 2.0f * std::pow(bhn / REF_BHN, 0.25f) * std::pow(REF_DIAMETER / std::max(diameter, 1e-6f), 0.33f) * std::pow(REF_SPEED / std::max(speed, 1e-3f), 0.2f);

    float rad = math::deg2rad(angle);
    return std::clamp(rad, 0.0f, math::constants::PI * 0.5f);
}

float ImpactResolver::maxPenetration(float mass, float speed, float diameter, float bhn)
{
    // energy-based penetration model:
    // kinetic energy E_k = 0.5 * m * v^2
    // resisting force F = BHN * 1e6 * pi/4 * d^2
    //   (BHN in MPa approximation for penetration resistance)
    // penetration depth e = E_k / F

    float area = math::constants::PI * 0.25f * diameter * diameter;
    float resistForce = bhn * 1e6f * area;

    if (resistForce < 1e-9f) return 1e6f;

    float kineticEnergy = 0.5f * mass * speed * speed;
    return kineticEnergy / resistForce;
}

float ImpactResolver::residualSpeed(float speed, float thickness, float maxPen)
{
    // v_res = v * sqrt(1 - (t/e_max)^2)
    if (maxPen < 1e-9f) return 0.0f;

    float ratio = thickness / maxPen;
    if (ratio >= 1.0f) return 0.0f;

    return speed * std::sqrt(1.0f - ratio * ratio);
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
        if (t1 > t2) std::swap(t1, t2);
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
        if (t1 > t2) std::swap(t1, t2);
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
        if (t1 > t2) std::swap(t1, t2);
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

    float diameter = specs.diameter.value_or(REF_DIAMETER);
    float mass = specs.mass;
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
    float thetaCrit = criticalAngle(material.brinellHardness, diameter, speed);

    if (grazingAngle < thetaCrit)
    {
        math::Vec3 residualVelocity = vTangent + normal * (-vDotN * material.restitution);
        float resSpeed = residualVelocity.length();
        float energyAbsorbed = kineticEnergy - 0.5f * mass * resSpeed * resSpeed;

        return {ImpactOutcome::Ricochet, residualVelocity, std::max(energyAbsorbed, 0.0f), 0.0f};
    }

    // effective thickness
    float thickness = computeEffectiveThickness(projectile.getPosition(), velocity, &collider);

    // penetration calculation
    float maxPen = maxPenetration(mass, speed, diameter, material.brinellHardness);

    if (maxPen < thickness)
    {
        return {ImpactOutcome::Embed, {0.0f, 0.0f, 0.0f}, kineticEnergy, maxPen};
    }

    // full penetration
    float vRes = residualSpeed(speed, thickness, maxPen);
    math::Vec3 direction = velocity.normalized();
    math::Vec3 residualVelocity = direction * vRes;
    float residualEnergy = 0.5f * mass * vRes * vRes;

    return {ImpactOutcome::Penetration, residualVelocity, kineticEnergy - residualEnergy, thickness};
}

} // namespace terminal
} // namespace collision
} // namespace BulletPhysics
