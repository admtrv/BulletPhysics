/*
 * PhysicsBody.cpp
 */

#include "PhysicsBody.h"

namespace BulletPhysics {
namespace dynamics {

namespace projectile {

// ProjectileSpecs

float ProjectileSpecs::calculateArea(float diameter)
{
    // cross-sectional area: S = pi * d ^ 2 / 4
    return math::constants::PI * diameter * diameter * 0.25f;
}

float ProjectileSpecs::calculateMomentOfInertiaX(float mass, float diameter)
{
    // uniform cylinder approximation: Ix = 1/8 * m * d^2
    return 0.125f * mass * diameter * diameter;
}

float ProjectileSpecs::calculateSpinRate(float velocity, float twistRate, float diameter)
{
    // spin rate: p = 2 * pi * V / (n * d)
    return 2.0f * math::constants::PI * velocity / (twistRate * diameter);
}

}

// RigidBody

void RigidBody::setMass(float mass)
{
    m_mass = (mass > 0.0f ? mass : 1.0f);
}

void RigidBody::setPosition(const math::Vec3& pos)
{
    m_position = pos;
}

void RigidBody::setVelocity(const math::Vec3& v)
{
    m_velocity = v;
}

void RigidBody::setVelocityFromAngles(float speed, float elevationDeg, float azimuthDeg)
{
    const float elev = math::deg2rad(elevationDeg);
    const float azim = math::deg2rad(azimuthDeg);

    const float ce = std::cos(elev);
    const float se = std::sin(elev);
    const float sa = std::sin(azim);
    const float ca = std::cos(azim);

    m_velocity = {ce * sa * speed, se * speed, ce * ca * speed};
}

void RigidBody::clearForces()
{
    m_forces = math::Vec3{};
}

std::unique_ptr<IPhysicsBody> RigidBody::clone() const
{
    return std::make_unique<RigidBody>(*this);
}

// ProjectileRigidBody

namespace projectile {

ProjectileRigidBody::ProjectileRigidBody(const ProjectileSpecs& specs) : RigidBody(), m_specs(specs)
{
    setMass(specs.mass);

    if (!m_specs.area.has_value() && m_specs.diameter.has_value())
    {
        m_specs.area = ProjectileSpecs::calculateArea(m_specs.diameter.value());
    }

    if (m_specs.spinSpecs)
    {
        auto& spinSpecs = *m_specs.spinSpecs;

        if (!spinSpecs.momentOfInertia && m_specs.diameter)
        {
            spinSpecs.momentOfInertia = ProjectileSpecs::calculateMomentOfInertiaX(m_specs.mass, *m_specs.diameter);
        }
    }
}

// auto-calculate spin rate on first velocity set if not already set
void ProjectileRigidBody::setInitialSpinRate(float velocity)
{
    if (!m_specs.spinSpecs)
    {
        return;
    }

    auto& spinSpecs = *m_specs.spinSpecs;

    if (!spinSpecs.spinRate && spinSpecs.riflingSpecs && m_specs.diameter)
    {
        spinSpecs.spinRate = ProjectileSpecs::calculateSpinRate(velocity, spinSpecs.riflingSpecs->twistRate, *m_specs.diameter);
    }
}

std::unique_ptr<IPhysicsBody> ProjectileRigidBody::clone() const
{
    return std::make_unique<ProjectileRigidBody>(*this);
}

} // namespace projectile

} // namespace dynamics
} // namespace BulletPhysics
