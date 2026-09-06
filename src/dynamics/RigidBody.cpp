/*
 * RigidBody.cpp
 */

#include "RigidBody.h"

namespace BulletPhysics {
namespace dynamics {

void RigidBody::setMotionType(MotionType type)
{
    m_motionType = type;

    if (type == MotionType::Static)
    {
        m_inverseMass = 0.0;
        m_inverseInertiaWorld = math::Mat3::zero();

        m_velocity = math::Vec3{};
        m_angularVelocity = math::Vec3{};
    }
    else
    {
        m_inverseMass = 1.0 / m_mass;
        updateInverseInertiaWorld();
    }
}

void RigidBody::setMass(double mass)
{
    m_mass = (mass > 0.0 ? mass : DEFAULT_MASS);
    m_inverseMass = isDynamic() ? 1.0 / m_mass : 0.0;
}

void RigidBody::setInverseInertiaLocal(const math::Mat3& inverseInertia)
{
    m_inverseInertiaLocal = inverseInertia;
    updateInverseInertiaWorld();
}

void RigidBody::setOrientation(const math::Quat& orientation)
{
    m_orientation = orientation.normalized();
    updateInverseInertiaWorld();
}

math::Vec3 RigidBody::getVelocityAt(const math::Vec3& point) const
{
    return m_velocity + m_angularVelocity.cross(point - m_position);
}

void RigidBody::addForceAtPoint(const math::Vec3& force, const math::Vec3& point)
{
    m_forces += force;

    // t = r x F, force through centre of mass gives no torque
    m_torque += (point - m_position).cross(force);
}

void RigidBody::wake()
{
    m_sleeping = false;
    m_stillTime = 0.0;
}

void RigidBody::updateSleep(double dt)
{
    if (m_velocity.length() > SLEEP_LINEAR_SPEED || m_angularVelocity.length() > SLEEP_ANGULAR_SPEED)
    {
        m_stillTime = 0.0;
        return;
    }

    m_stillTime += dt;

    if (m_stillTime >= SLEEP_DELAY)
    {
        m_sleeping = true;

        m_velocity = math::Vec3{};
        m_angularVelocity = math::Vec3{};
    }
}

void RigidBody::clearForces()
{
    m_forces = math::Vec3{};
    m_torque = math::Vec3{};
}

void RigidBody::updateInverseInertiaWorld()
{
    if (!isDynamic())
    {
        m_inverseInertiaWorld = math::Mat3::zero();
        return;
    }

    const math::Mat3 rotation = m_orientation.toMat3();
    m_inverseInertiaWorld = rotation * m_inverseInertiaLocal * rotation.transposed();
}

} // namespace dynamics
} // namespace BulletPhysics
