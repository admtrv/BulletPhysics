/*
 * RigidBody.cpp
 */

#include "RigidBody.h"
#include "math/Angles.h"

namespace BulletPhysics {
namespace builtin {
namespace bodies {

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

ProjectileRigidBody::ProjectileRigidBody(const projectile::ProjectileSpecs& specs) : RigidBody(), m_specs(specs)
{
    setMass(specs.mass);
}

std::unique_ptr<IPhysicsBody> ProjectileRigidBody::clone() const
{
    return std::make_unique<ProjectileRigidBody>(*this);
}

} // namespace bodies
} // namespace builtin
} // namespace BulletPhysics
