/*
 * RigidBody.h
 */

#pragma once

#include "PhysicsBody.h"

namespace BulletPhysics {
namespace builtin {
namespace bodies {

// concrete implementation of rigid body
class RigidBody : public virtual IPhysicsBody {
public:
    RigidBody() = default;
    RigidBody(const RigidBody&) = default;
    RigidBody& operator=(const RigidBody&) = default;
    virtual ~RigidBody() = default;

    std::unique_ptr<IPhysicsBody> clone() const override;

    // mass
    double getMass() const override { return m_mass; }
    void setMass(double mass);

    // position
    math::Vec3 getPosition() const override { return m_position; }
    void setPosition(const math::Vec3& pos) override;

    // velocity
    math::Vec3 getVelocity() const override { return m_velocity; }
    void setVelocity(const math::Vec3& vel) override;
    virtual void setVelocityFromAngles(double speed, double elevationDeg, double azimuthDeg);

    // forces
    const math::Vec3& getAccumulatedForces() const override { return m_forces; }
    void addForce(const math::Vec3& f) override { m_forces += f; }
    void clearForces() override;

private:
    double m_mass = 1.0;
    math::Vec3 m_position{};
    math::Vec3 m_velocity{};
    math::Vec3 m_forces{};
};

// concrete projectile rigid body implementation
class ProjectileRigidBody : public RigidBody, public projectile::IProjectileBody {
public:
    explicit ProjectileRigidBody(const projectile::ProjectileSpecs& specs);

    std::unique_ptr<IPhysicsBody> clone() const override;

    const projectile::ProjectileSpecs& getProjectileSpecs() const override { return m_specs; }

private:
    projectile::ProjectileSpecs m_specs;
};

} // namespace bodies
} // namespace builtin
} // namespace BulletPhysics
