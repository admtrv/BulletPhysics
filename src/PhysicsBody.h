/*
 * PhysicsBody.h
 */

#pragma once

#include "Constants.h"
#include "ballistics/external/forces/drag/DragModel.h"
#include "math/Angles.h"

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

// interface for projectile bodies
namespace projectile {

// muzzle riffling specifications
struct RiflingSpecs {
    enum class Direction {
        RIGHT,      // clockwise
        LEFT        // counterclockwise
    };

    Direction direction;    // rifling direction
    double twistRate;       // n (calibers per turn)
};

// spin-related specifications
struct SpinSpecs {
    // dimensional specifications
    std::optional<double> momentOfInertia;      // I_x (kg * m^2)

    // aerodynamic coefficients
    double overtuningCoefficient = constants::DEFAULT_C_M_ALPHA;     // C_M_alpha
    double liftCoefficient = constants::DEFAULT_C_L_ALPHA;           // C_L_alpha
    double magnusCoefficient = constants::DEFAULT_C_MAG_F;           // C_mag_f

    std::optional<RiflingSpecs> riflingSpecs;
    std::optional<double> spinRate;              // initial spin rate (rad/s)
};

struct ProjectileSpecs {
    // basic specifications
    double mass;                 // kg

    // dimensional specifications
    std::optional<double> area;          // m^2 (cross-sectional area)
    std::optional<double> diameter;      // m (caliber)

    // aerodynamic coefficients
    std::optional<ballistics::external::forces::drag::DragCurveModel> dragModel;      // C_d

    // spin-related specifications
    std::optional<SpinSpecs> spinSpecs;

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

} // namespace projectile

} // namespace BulletPhysics
