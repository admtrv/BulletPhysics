/*
 * SpinDrift.cpp
 */

#include "SpinDrift.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace forces {

// yaw of repose: alpha_e = 2 * Ix * p * (g x V) / (rho * S * d * V^4 * C_M_alpha)
static math::Vec3 calculateYawOfRepose(const projectile::ProjectileSpecs& specs, const PhysicsContext& context, const math::Vec3& velocity)
{
    // requires muzzle specs
    if (!specs.muzzleSpecs.has_value())
    {
        return {0.0, 0.0, 0.0};
    }
    const auto& muzzleSpecs = *specs.muzzleSpecs;

    // velocity
    double velocityMagnitude = velocity.length();
    if (velocityMagnitude < 1e-3)
    {
        return {0.0, 0.0, 0.0};
    }

    // denominator
    double rho = context.airDensity;
    double S = specs.area;
    double d = specs.diameter;
    double velocityMagnitudePow4 = velocityMagnitude * velocityMagnitude * velocityMagnitude * velocityMagnitude;
    double C_M_alpha = muzzleSpecs.overtuningCoefficient;

    double denominator = rho * S * d * velocityMagnitudePow4 * C_M_alpha;

    // numerator
    double Ix = muzzleSpecs.momentOfInertia;
    double p = muzzleSpecs.spinRate;
    math::Vec3 g = context.gravity;
    math::Vec3 gCrossV = g.cross(velocity);

    math::Vec3 numerator = 2.0 * Ix * p * gCrossV;

    // final
    int spinSign = muzzleSpecs.riflingDirection == projectile::Direction::RIGHT ? 1 : -1;

    return spinSign * numerator / denominator;
}

void Lift::apply(IPhysicsBody& body, PhysicsContext& context)
{
    // requires projectile body
    auto* projectile = dynamic_cast<projectile::IProjectileBody*>(&body);
    if (!projectile)
    {
        return;
    }

    // requires muzzle specs
    const auto& specs = projectile->getProjectileSpecs();
    if (!specs.muzzleSpecs.has_value())
    {
        return;
    }
    const auto& muzzleSpecs = *specs.muzzleSpecs;

    // velocity
    math::Vec3 velocity = body.getVelocity();
    double velocityMagnitude = velocity.length();
    if (velocityMagnitude < 1e-3)
    {
        return;
    }
    double velocityMagnitudePow2 = velocityMagnitude * velocityMagnitude;

    // data
    double rho = context.airDensity;
    double S = specs.area;
    double C_L_alpha = muzzleSpecs.liftCoefficient;

    // yaw of repose
    math::Vec3 alpha_e = calculateYawOfRepose(specs, context, velocity);

    // F_l = 1/2 * rho * S * C_L_alpha * V^2 * alpha_e
    math::Vec3 force = 0.5 * rho * S * C_L_alpha * velocityMagnitudePow2 * alpha_e;

    m_force = force;
    body.addForce(force);
}

void Magnus::apply(IPhysicsBody& body, PhysicsContext& context)
{
    // requires projectile body
    auto* proj = dynamic_cast<projectile::IProjectileBody*>(&body);
    if (!proj)
    {
        return;
    }

    // requires muzzle specs
    const auto& specs = proj->getProjectileSpecs();
    if (!specs.muzzleSpecs.has_value())
    {
        return;
    }
    const auto& muzzleSpecs = *specs.muzzleSpecs;

    // velocity
    math::Vec3 velocity = body.getVelocity();
    double velocityMagnitude = velocity.length();
    if (velocityMagnitude < 1e-3)
    {
        return;
    }

    // data
    double rho = context.airDensity;
    double d = specs.diameter;
    double S = specs.area;
    double p = muzzleSpecs.spinRate;
    double C_mag_f = muzzleSpecs.magnusCoefficient;

    // yaw of repose
    math::Vec3 alpha_e = calculateYawOfRepose(specs, context, velocity);
    math::Vec3 alphaCrossV = alpha_e.cross(velocity);

    // F_m = -1/2 * rho * S * d * p * C_mag_f * (alpha_e x V)
    math::Vec3 force = -0.5 * rho * S * d * p * C_mag_f * alphaCrossV;

    m_force = force;
    body.addForce(force);
}

} // namespace forces
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
