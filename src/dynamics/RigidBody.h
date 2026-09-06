/*
 * RigidBody.h
 */

#pragma once

#include "math/Mat3.h"
#include "math/Quat.h"
#include "math/Vec3.h"

#include <algorithm>

namespace BulletPhysics {
namespace dynamics {

inline constexpr double DEFAULT_MASS = 1.0;

inline constexpr double DEFAULT_LINEAR_DAMPING = 0.0;    // nothing slows a body in flight
inline constexpr double DEFAULT_ANGULAR_DAMPING = 0.8;   // spin fades, else a ball rolls forever

// damping only ever approaches zero, a body has to be put to sleep to truly stop
inline constexpr double SLEEP_LINEAR_SPEED = 0.08;
inline constexpr double SLEEP_ANGULAR_SPEED = 0.2;
inline constexpr double SLEEP_DELAY = 0.5;               // seconds of near stillness before it counts

enum class MotionType {
    Dynamic,    // moved by forces and contacts
    Static      // never moves, holds everything else up
};

class RigidBody {
public:
    RigidBody() = default;

    // motion
    MotionType getMotionType() const { return m_motionType; }
    void setMotionType(MotionType type);

    bool isDynamic() const { return m_motionType == MotionType::Dynamic; }

    // mass
    double getMass() const { return m_mass; }
    double getInverseMass() const { return m_inverseMass; }
    void setMass(double mass);

    // inertia in body space
    const math::Mat3& getInverseInertia() const { return m_inverseInertiaWorld; }
    void setInverseInertiaLocal(const math::Mat3& inverseInertia);

    // position
    const math::Vec3& getPosition() const { return m_position; }
    void setPosition(const math::Vec3& pos) { m_position = pos; }

    // orientation
    const math::Quat& getOrientation() const { return m_orientation; }
    void setOrientation(const math::Quat& orientation);

    // linear velocity
    const math::Vec3& getVelocity() const { return m_velocity; }
    void setVelocity(const math::Vec3& vel) { m_velocity = vel; }

    // angular velocity
    const math::Vec3& getAngularVelocity() const { return m_angularVelocity; }
    void setAngularVelocity(const math::Vec3& angularVel) { m_angularVelocity = angularVel; }

    // damping, share of velocity bled off per second
    // a rolling ball has no sliding left for friction to catch, this is what
    // brings it to a stop
    double getLinearDamping() const { return m_linearDamping; }
    void setLinearDamping(double damping) { m_linearDamping = std::max(damping, 0.0); }

    double getAngularDamping() const { return m_angularDamping; }
    void setAngularDamping(double damping) { m_angularDamping = std::max(damping, 0.0); }

    // scale to apply to velocity for a step of dt
    double dampingOver(double damping, double dt) const { return std::max(1.0 - damping * dt, 0.0); }

    // sleep
    // a body barely moving for long enough is parked, damping alone only ever
    // approaches zero and leaves everything drifting
    bool isSleeping() const { return m_sleeping; }
    void wake();

    void updateSleep(double dt);

    math::Vec3 getVelocityAt(const math::Vec3& point) const;

    // forces
    const math::Vec3& getAccumulatedForces() const { return m_forces; }
    const math::Vec3& getAccumulatedTorque() const { return m_torque; }

    void addForce(const math::Vec3& force) { m_forces += force; }
    void addForceAtPoint(const math::Vec3& force, const math::Vec3& point);
    void addTorque(const math::Vec3& torque) { m_torque += torque; }

    void clearForces();

private:
    // world inertia changes with orientation, cached
    void updateInverseInertiaWorld();

    MotionType m_motionType = MotionType::Dynamic;

    // mass
    double m_mass = DEFAULT_MASS;
    double m_inverseMass = 1.0 / DEFAULT_MASS;

    math::Mat3 m_inverseInertiaLocal = math::Mat3::zero();
    math::Mat3 m_inverseInertiaWorld = math::Mat3::zero();

    // state
    math::Vec3 m_position{};
    math::Quat m_orientation{};

    math::Vec3 m_velocity{};
    math::Vec3 m_angularVelocity{};

    double m_linearDamping = DEFAULT_LINEAR_DAMPING;
    double m_angularDamping = DEFAULT_ANGULAR_DAMPING;

    bool m_sleeping = false;
    double m_stillTime = 0.0;

    // accumulators
    math::Vec3 m_forces{};
    math::Vec3 m_torque{};
};

} // namespace dynamics
} // namespace BulletPhysics
