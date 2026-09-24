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

    // both unmovable by contacts, kinematic keeps its velocity
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

void RigidBody::applyConstraints(math::Vec3& linear, math::Vec3& angular) const
{
    if (m_constraints == CONSTRAIN_NONE)
    {
        return;
    }

    if (m_constraints & FREEZE_POSITION_X) linear.x = 0.0;
    if (m_constraints & FREEZE_POSITION_Y) linear.y = 0.0;
    if (m_constraints & FREEZE_POSITION_Z) linear.z = 0.0;

    if (m_constraints & FREEZE_ROTATION_X) angular.x = 0.0;
    if (m_constraints & FREEZE_ROTATION_Y) angular.y = 0.0;
    if (m_constraints & FREEZE_ROTATION_Z) angular.z = 0.0;
}

void RigidBody::applyImpulse(const math::Vec3& linear, const math::Vec3& angular)
{
    m_velocity += linear;
    m_angularVelocity += angular;
}

void RigidBody::separate(const math::Vec3& offset)
{
    math::Vec3 allowed = offset;
    math::Vec3 ignored{};

    applyConstraints(allowed, ignored);

    m_position += allowed;
}

void RigidBody::advance(double dt)
{
    // whatever set velocity, frozen axis stops here
    applyConstraints(m_velocity, m_angularVelocity);

    // semi-implicit euler, velocity of this step carries body
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

bool RigidBody::canMoveAlong(int axis) const
{
    static constexpr Constraints FROZEN[3] = {FREEZE_POSITION_X, FREEZE_POSITION_Y, FREEZE_POSITION_Z};

    return isMovable() && (m_constraints & FROZEN[axis]) == 0;
}

bool RigidBody::canTurnAround(int axis) const
{
    static constexpr Constraints FROZEN[3] = {FREEZE_ROTATION_X, FREEZE_ROTATION_Y, FREEZE_ROTATION_Z};

    return isMovable() && (m_constraints & FROZEN[axis]) == 0;
}

math::Mat3 RigidBody::getLinearMobility() const
{
    return math::Mat3::diagonal(canMoveAlong(0) ? m_inverseMass : 0.0,
                                canMoveAlong(1) ? m_inverseMass : 0.0,
                                canMoveAlong(2) ? m_inverseMass : 0.0);
}

// turned body couples world axes, so rows of inverse tensor cannot just be dropped,
// inertia is cut to free axes and inverted there, holding rest rigid
math::Mat3 RigidBody::getAngularMobility() const
{
    bool free[3];
    int freeCount = 0;

    for (int axis = 0; axis < 3; axis++)
    {
        free[axis] = canTurnAround(axis);
        freeCount += free[axis] ? 1 : 0;
    }

    if (freeCount == 3)
    {
        return m_inverseInertiaWorld;
    }

    if (freeCount == 0)
    {
        return math::Mat3::zero();
    }

    const math::Mat3 inertia = m_inverseInertiaWorld.inverted();

    // frozen rows and columns give way to identity, inverting whole then leaves
    // free block inverted and rest to be wiped
    math::Mat3 block;

    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            const bool kept = free[row] && free[column];

            block.rows[row][column] = kept ? inertia.rows[row][column] : (row == column ? 1.0 : 0.0);
        }
    }

    math::Mat3 mobility = block.inverted();

    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            if (!free[row] || !free[column])
            {
                mobility.rows[row][column] = 0.0;
            }
        }
    }

    return mobility;
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
