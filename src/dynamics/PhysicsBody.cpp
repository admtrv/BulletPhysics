/*
 * PhysicsBody.cpp
 */

#include "PhysicsBody.h"

namespace BulletPhysics {
namespace dynamics {

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

}

// RigidBody

void RigidBody::setMass(double mass)
{
    m_mass = (mass > 0.0 ? mass : 1.0);
}

void RigidBody::setPosition(const math::Vec3& pos)
{
    m_position = pos;
}

void RigidBody::setVelocity(const math::Vec3& v)
{
    m_velocity = v;
}

void RigidBody::setVelocityFromAngles(double speed, double elevationDeg, double azimuthDeg)
{
    const double elev = math::deg2rad(elevationDeg);
    const double azim = math::deg2rad(azimuthDeg);

    const double ce = std::cos(elev);
    const double se = std::sin(elev);
    const double sa = std::sin(azim);
    const double ca = std::cos(azim);

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
void ProjectileRigidBody::setInitialSpinRate(double velocity)
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
