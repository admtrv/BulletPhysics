/*
 * RigidBody.cpp
 */

#include "RigidBody.h"

namespace BulletPhysics {
namespace dynamics {

void RigidBody::setMotionType(MotionType type)
{
    m_motionType = type;

    // static body has infinite mass, contacts and forces move it by zero
    m_inverseMass = (type == MotionType::Static) ? 0.0 : 1.0 / m_mass;

    if (type == MotionType::Static)
    {
        m_velocity = math::Vec3{};
    }
}

void RigidBody::setMass(double mass)
{
    m_mass = (mass > 0.0 ? mass : DEFAULT_MASS);
    m_inverseMass = isDynamic() ? 1.0 / m_mass : 0.0;
}

} // namespace dynamics
} // namespace BulletPhysics
