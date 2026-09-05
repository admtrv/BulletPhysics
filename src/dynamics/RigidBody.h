/*
 * RigidBody.h
 */

#pragma once

#include "math/Vec3.h"

namespace BulletPhysics {
namespace dynamics {

inline constexpr double DEFAULT_MASS = 1.0;

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

    // position
    const math::Vec3& getPosition() const { return m_position; }
    void setPosition(const math::Vec3& pos) { m_position = pos; }

    // velocity
    const math::Vec3& getVelocity() const { return m_velocity; }
    void setVelocity(const math::Vec3& vel) { m_velocity = vel; }

    // forces
    const math::Vec3& getAccumulatedForces() const { return m_forces; }
    void addForce(const math::Vec3& f) { m_forces += f; }
    void clearForces() { m_forces = math::Vec3{}; }

private:
    MotionType m_motionType = MotionType::Dynamic;

    double m_mass = DEFAULT_MASS;
    double m_inverseMass = 1.0 / DEFAULT_MASS;

    math::Vec3 m_position{};
    math::Vec3 m_velocity{};
    math::Vec3 m_forces{};
};

} // namespace dynamics
} // namespace BulletPhysics
