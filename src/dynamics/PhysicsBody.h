/*
 * PhysicsBody.h
 */

#pragma once

#include "Constants.h"
#include "math/Vec3.h"
#include "math/Angles.h"
#include "math/Constants.h"
#include "forces/drag/DragModel.h"

#include <cmath>
#include <optional>
#include <memory>

namespace BulletPhysics {
namespace dynamics {

// base interface for all physics bodies
class IPhysicsBody {
public:
    virtual ~IPhysicsBody() = default;

    virtual std::unique_ptr<IPhysicsBody> clone() const = 0;

    // mass
    virtual float getMass() const = 0;

    // position
    virtual math::Vec3 getPosition() const = 0;
    virtual void setPosition(const math::Vec3& pos) = 0;

    // velocity
    virtual math::Vec3 getVelocity() const = 0;
    virtual void setVelocity(const math::Vec3& vel) = 0;

    // forces
    virtual const math::Vec3& getAccumulatedForces() const = 0;
    virtual void addForce(const math::Vec3& force) = 0;
    virtual void clearForces() = 0;
};

// interface for projectile bodies
namespace projectile {

// muzzle riffling specifications
struct RiflingSpecs {
    enum class Direction {
        RIGHT,      // clockwise
        LEFT        // counterclockwise
    };

    Direction direction;    // rifling direction
    float twistRate;        // n (calibers per turn)
};

// spin-related specifications
struct SpinSpecs {
    // dimensional specifications
    std::optional<float> momentOfInertia;      // I_x (kg * m^2)

    // aerodynamic coefficients
    float overtuningCoefficient = constants::DEFAULT_C_M_ALPHA;     // C_M_alpha
    float liftCoefficient = constants::DEFAULT_C_L_ALPHA;           // C_L_alpha
    float magnusCoefficient = constants::DEFAULT_C_MAG_F;           // C_mag_f

    std::optional<RiflingSpecs> riflingSpecs;
    std::optional<float> spinRate;              // initial spin rate (rad/s)
};

struct ProjectileSpecs {
    // basic specifications
    float mass;                 // kg

    // dimensional specifications
    std::optional<float> area;          // m^2 (cross-sectional area)
    std::optional<float> diameter;      // m (caliber)

    // aerodynamic coefficients
    std::optional<forces::drag::DragCurveModel> dragModel;      // C_d

    // spin-related specifications
    std::optional<SpinSpecs> spinSpecs;

    static float calculateArea(float diameter);
    static float calculateMomentOfInertiaX(float mass, float diameter);
    static float calculateSpinRate(float velocity, float twistRate, float diameter);
};

// projectile interface
class IProjectileBody : public virtual IPhysicsBody {
public:
    virtual ~IProjectileBody() = default;

    // projectile specifications
    virtual const ProjectileSpecs& getProjectileSpecs() const = 0;
};

} // namespace projectile

// concrete implementation of rigid body
class RigidBody : public virtual IPhysicsBody {
public:
    RigidBody() = default;
    RigidBody(const RigidBody&) = default;
    RigidBody& operator=(const RigidBody&) = default;
    virtual ~RigidBody() = default;

    std::unique_ptr<IPhysicsBody> clone() const override;

    // mass
    float getMass() const override { return m_mass; }
    void setMass(float mass);

    // position
    math::Vec3 getPosition() const override { return m_position; }
    void setPosition(const math::Vec3& pos) override;

    // velocity
    math::Vec3 getVelocity() const override { return m_velocity; }
    void setVelocity(const math::Vec3& vel) override;
    virtual void setVelocityFromAngles(float speed, float elevationDeg, float azimuthDeg);

    // forces
    const math::Vec3& getAccumulatedForces() const override { return m_forces; }
    void addForce(const math::Vec3& f) override { m_forces += f; }
    void clearForces() override;

private:
    float m_mass = 1.0f;
    math::Vec3 m_position{};
    math::Vec3 m_velocity{};
    math::Vec3 m_forces{};
};

// concrete projectile rigid body implementation
namespace projectile {

class ProjectileRigidBody : public RigidBody, public IProjectileBody {
public:
    ProjectileRigidBody() : RigidBody(), m_specs{1.0f} {}
    explicit ProjectileRigidBody(const ProjectileSpecs& specs);

    std::unique_ptr<IPhysicsBody> clone() const override;

    // projectile specifications
    const ProjectileSpecs& getProjectileSpecs() const override { return m_specs; }

    // override to calculate spin rate on first velocity set
    void setVelocity(const math::Vec3& vel) override
    {
        RigidBody::setVelocity(vel);
        setInitialSpinRate(vel.length());
    }
    void setVelocityFromAngles(float speed, float elevationDeg, float azimuthDeg) override
    {
        RigidBody::setVelocityFromAngles(speed, elevationDeg, azimuthDeg);
        setInitialSpinRate(speed);
    }
    void setInitialSpinRate(float velocity);

private:
    ProjectileSpecs m_specs;
};

} // namespace projectile

} // namespace dynamics
} // namespace BulletPhysics
