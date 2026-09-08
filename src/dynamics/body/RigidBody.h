/*
 * RigidBody.h
 */

#pragma once

#include "math/Mat3.h"
#include "math/Quat.h"
#include "math/Vec3.h"

#include <algorithm>
#include <cstdint>

namespace BulletPhysics {
namespace dynamics {

// defaults
inline constexpr double DEFAULT_MASS = 1.0;
inline constexpr double DEFAULT_LINEAR_DAMPING = 0.0;    // nothing slows a body in flight
inline constexpr double DEFAULT_ANGULAR_DAMPING = 0.8;   // else a ball rolls forever

// sleep
inline constexpr double SLEEP_LINEAR_SPEED = 0.08;
inline constexpr double SLEEP_ANGULAR_SPEED = 0.2;
inline constexpr double SLEEP_DELAY = 0.5;               // seconds of stillness before parking

enum class MotionType {
    Dynamic,      // moved by forces and contacts
    Kinematic,    // moved by hand, pushes others and ignores them back
    Static        // never moves, holds everything else up
};

// axes the body is not allowed to move or turn along
using Constraints = uint32_t;

inline constexpr Constraints CONSTRAIN_NONE = 0;

inline constexpr Constraints FREEZE_POSITION_X = 1u << 0;
inline constexpr Constraints FREEZE_POSITION_Y = 1u << 1;
inline constexpr Constraints FREEZE_POSITION_Z = 1u << 2;

inline constexpr Constraints FREEZE_ROTATION_X = 1u << 3;
inline constexpr Constraints FREEZE_ROTATION_Y = 1u << 4;
inline constexpr Constraints FREEZE_ROTATION_Z = 1u << 5;

inline constexpr Constraints FREEZE_POSITION = FREEZE_POSITION_X | FREEZE_POSITION_Y | FREEZE_POSITION_Z;
inline constexpr Constraints FREEZE_ROTATION = FREEZE_ROTATION_X | FREEZE_ROTATION_Y | FREEZE_ROTATION_Z;
inline constexpr Constraints FREEZE_ALL = FREEZE_POSITION | FREEZE_ROTATION;

class RigidBody {
public:
    RigidBody() = default;

    // motion
    MotionType getMotionType() const { return m_motionType; }
    void setMotionType(MotionType type);

    // utils
    bool isDynamic() const { return m_motionType == MotionType::Dynamic; }
    bool isKinematic() const { return m_motionType == MotionType::Kinematic; }
    bool isMovable() const { return m_motionType != MotionType::Static; }

    // constraints
    Constraints getConstraints() const { return m_constraints; }
    void setConstraints(Constraints constraints) { m_constraints = constraints; }

    // mass
    double getMass() const { return m_mass; }
    double getInverseMass() const { return m_inverseMass; }
    void setMass(double mass);

    // inertia in body space
    const math::Mat3& getInverseInertia() const { return m_inverseInertiaWorld; }
    void setInverseInertiaLocal(const math::Mat3& inverseInertia);

    // position
    const math::Vec3& getPosition() const { return m_position; }
    void setPosition(const math::Vec3& pos) { m_position = pos; wake(); }

    // orientation
    const math::Quat& getOrientation() const { return m_orientation; }
    void setOrientation(const math::Quat& orientation);

    // velocity

    // linear
    const math::Vec3& getVelocity() const { return m_velocity; }
    math::Vec3 getVelocityAt(const math::Vec3& point) const;
    void setVelocity(const math::Vec3& vel) { m_velocity = vel; wake(); }

    // angular
    const math::Vec3& getAngularVelocity() const { return m_angularVelocity; }
    void setAngularVelocity(const math::Vec3& angularVel) { m_angularVelocity = angularVel; wake(); }

    // damping

    void applyDamping(double dt);

    // linear
    double getLinearDamping() const { return m_linearDamping; }
    void setLinearDamping(double damping) { m_linearDamping = std::max(damping, 0.0); }

    // angular
    double getAngularDamping() const { return m_angularDamping; }
    void setAngularDamping(double damping) { m_angularDamping = std::max(damping, 0.0); }

    // checked along the path it travels, for a body fast enough to pass through a wall
    bool isContinuous() const { return m_continuous; }
    void setContinuous(bool continuous) { m_continuous = continuous; }

    // sleep, decided by the island the body belongs to
    bool isSleeping() const { return m_sleeping; }

    void sleep();
    void wake() { m_sleeping = false; }

    // solver side, moves the body without waking it, a resting pile would
    // never settle otherwise
    void applyImpulse(const math::Vec3& linear, const math::Vec3& angular);
    void advance(double dt);
    void separate(const math::Vec3& offset);

    // forces
    const math::Vec3& getAccumulatedForces() const { return m_forces; }
    const math::Vec3& getAccumulatedTorque() const { return m_torque; }

    void addForce(const math::Vec3& force) { m_forces += force; wake(); }
    void addForceAtPoint(const math::Vec3& force, const math::Vec3& point);
    void addTorque(const math::Vec3& torque) { m_torque += torque; wake(); }

    void clearForces();

private:
    void updateInverseInertiaWorld();

    // zeroes whatever the constraints forbid
    void applyConstraints(math::Vec3& linear, math::Vec3& angular) const;

    MotionType m_motionType = MotionType::Dynamic;
    Constraints m_constraints = CONSTRAIN_NONE;

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
    bool m_continuous = false;

    // accumulators
    math::Vec3 m_forces{};
    math::Vec3 m_torque{};
};

} // namespace dynamics
} // namespace BulletPhysics
