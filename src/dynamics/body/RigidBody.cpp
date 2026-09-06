/*
 * RigidBody.cpp
 */

#include "RigidBody.h"

namespace BulletPhysics {
namespace dynamics {

void RigidBody::setMotionType(MotionType type)
{
    m_motionType = type;

    if (type == MotionType::Dynamic)
    {
        m_inverseMass = 1.0 / m_mass;
        updateInverseInertiaWorld();

        return;
    }

    // both are unmovable by contacts, kinematic keeps its velocity
    m_inverseMass = 0.0;
    m_inverseInertiaWorld = math::Mat3::zero();

    if (type == MotionType::Static)
    {
        m_velocity = math::Vec3{};
        m_angularVelocity = math::Vec3{};
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
    addForce(force);

    // t = r x F, force through centre of mass gives no torque
    addTorque((point - m_position).cross(force));
}

void RigidBody::applyDamping(double dt)
{
    m_velocity *= std::max(1.0 - m_linearDamping * dt, 0.0);
    m_angularVelocity *= std::max(1.0 - m_angularDamping * dt, 0.0);
}

void RigidBody::applyImpulse(const math::Vec3& linear, const math::Vec3& angular)
{
    m_velocity += linear;
    m_angularVelocity += angular;
}

void RigidBody::advance(double dt)
{
    // semi-implicit euler, velocity of this step carries the body
    m_position += m_velocity * dt;

    // q' = q + 0.5 * w * q * dt, w as quaternion with zero scalar part
    const math::Quat spin{0.0, m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z};

    setOrientation(m_orientation + spin * m_orientation * (0.5 * dt));
}

void RigidBody::sleep()
{
    m_sleeping = true;

    m_velocity = math::Vec3{};
    m_angularVelocity = math::Vec3{};
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
