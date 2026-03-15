/*
 * PhysicsBody.h
 */

#pragma once

#include "Constants.h"
#include "ballistics/external/forces/drag/DragModel.h"

#include <memory>
#include <optional>

namespace BulletPhysics {

// base interface for all physics bodies
class IPhysicsBody {
public:
    virtual ~IPhysicsBody() = default;

    virtual std::unique_ptr<IPhysicsBody> clone() const = 0;

    // mass
    virtual double getMass() const = 0;

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

namespace projectile {

enum class Direction {
    RIGHT,              // clockwise
    LEFT                // counterclockwise
};

// muzzle (barrel) specifications
struct MuzzleSpecs {
    double velocity;                // muzzle velocity (m/s)
    Direction riflingDirection;     // rifling direction
    double twistRate;               // n (calibers per turn)

    // coefficients
    double overtuningCoefficient = constants::DEFAULT_C_M_ALPHA;     // C_M_alpha
    double liftCoefficient = constants::DEFAULT_C_L_ALPHA;           // C_L_alpha
    double magnusCoefficient = constants::DEFAULT_C_MAG_F;           // C_mag_f

    // auto-calculated
    double spinRate;             // initial spin rate (rad/s)
    double momentOfInertia;      // I_x (kg * m^2)
};

struct ProjectileSpecs {
    // basic specifications
    double mass;                 // kg
    double diameter;             // m (caliber)

    // drag-related data
    double area;                                                                // m^2 (cross-sectional area)
    std::shared_ptr<ballistics::external::forces::drag::IDragModel> dragModel;  // C_d

    // muzzle-related data
    std::optional<MuzzleSpecs> muzzleSpecs;

public:
    // builder
    static ProjectileSpecs create(double mass, double diameter);

    // builder methods
    ProjectileSpecs& withDragModel(ballistics::external::forces::drag::DragCurveModel model);   // G1..G8, GL
    ProjectileSpecs& withCustomDragCoefficient(double C_d);                                     // constant custom C_d

    ProjectileSpecs& withMuzzle(double muzzleVelocity, Direction riflingDirection, double twistRate);

    // if available
    ProjectileSpecs& withOvertuningCoefficient(double C_M_alpha);   // for yaw of repose (a_e)
    ProjectileSpecs& withLiftCoefficient(double C_L_alpha);         // for Lift force (F_l)
    ProjectileSpecs& withMagnusCoefficient(double C_mag_f);         // for Magnus force (F_m)

private:
    static double calculateArea(double diameter);
    static double calculateMomentOfInertiaX(double mass, double diameter);
    static double calculateSpinRate(double velocity, double twistRate, double diameter);
};

// projectile interface
class IProjectileBody : public virtual IPhysicsBody {
public:
    virtual ~IProjectileBody() = default;

    // projectile specifications
    virtual const ProjectileSpecs& getProjectileSpecs() const = 0;
};

namespace presets {

// Idealized sphere
ProjectileSpecs Sphere(double mass, double diameter);

// 7.62 NATO bullet
ProjectileSpecs Nato762();

} // namespace presets

} // namespace projectile
} // namespace BulletPhysics
