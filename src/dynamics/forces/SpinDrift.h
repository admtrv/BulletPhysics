/*
 * SpinDrift.h
 */

#pragma once

#include "Force.h"
#include "dynamics/PhysicsBody.h"
#include "Constants.h"
#include "math/Vec3.h"

#include <cmath>
#include <memory>

namespace BulletPhysics {
namespace dynamics {
namespace forces {

// helper function to check if specs have required data for spin drift calculations
static bool hasSpinDriftData(const projectile::ProjectileSpecs& specs)
{
    if (!specs.diameter.has_value()) return false;
    if (!specs.area.has_value()) return false;

    if (!specs.spinSpecs.has_value()) return false;
    const auto& spin = *specs.spinSpecs;

    if (!spin.momentOfInertia.has_value()) return false;
    if (!spin.spinRate.has_value()) return false;

    return true;
}

// yaw of repose: alpha_e = 2 * Ix * p * (g x V) / (rho * S * d * V^4 * C_M_alpha)
static math::Vec3 calculateYawOfRepose(const projectile::ProjectileSpecs& specs, const PhysicsContext& context, const math::Vec3& velocity)
{
    // requers projectile spin specs
    if (!hasSpinDriftData(specs))
    {
        return {0.0, 0.0, 0.0};
    }
    const auto& spinSpecs = specs.spinSpecs.value();

    // velocity
    double velocityMagnitude = velocity.length();
    if (velocityMagnitude < 1e-3)
    {
        return {0.0, 0.0, 0.0};
    }

    // denominator
    double rho = context.airDensity.value_or(constants::BASE_ATMOSPHERIC_DENSITY);
    double S = specs.area.value();
    double d = specs.diameter.value();
    double velocityMagnitudePow4 = velocityMagnitude * velocityMagnitude * velocityMagnitude * velocityMagnitude;
    double C_M_alpha = spinSpecs.overtuningCoefficient;

    double denominator = rho * S * d * velocityMagnitudePow4 * C_M_alpha;

    // numerator
    double Ix = spinSpecs.momentOfInertia.value();
    double p = spinSpecs.spinRate.value();
    math::Vec3 g = constants::GRAVITY;
    math::Vec3 gCrossV = g.cross(velocity);

    math::Vec3 numerator = 2.0 * Ix * p * gCrossV;

    // final
    int spinSign = spinSpecs.riflingSpecs->direction == projectile::RiflingSpecs::Direction::RIGHT ? 1 : -1;

    return spinSign * numerator / denominator;
}

class Lift : public IForce {
public:
    void apply(IPhysicsBody& body, PhysicsContext& context, double /*dt*/) override
    {
        // requires projectile body
        auto* projectile = dynamic_cast<projectile::IProjectileBody*>(&body);
        if (!projectile)
        {
            return;
        }

        const auto& specs = projectile->getProjectileSpecs();

        // requers projectile spin specs
        if (!hasSpinDriftData(specs))
        {
            return;
        }
        const auto& spinSpecs = specs.spinSpecs.value();

        // velocity
        math::Vec3 velocity = body.getVelocity();
        double velocityMagnitude = velocity.length();
        if (velocityMagnitude < 1e-3)
        {
            return;
        }
        double velocityMagnitudePow2 = velocityMagnitude * velocityMagnitude;

        // data
        double rho = context.airDensity.value_or(constants::BASE_ATMOSPHERIC_DENSITY);
        double S = specs.area.value();
        double C_L_alpha = spinSpecs.liftCoefficient;

        // yaw of repose
        math::Vec3 alpha_e = calculateYawOfRepose(specs, context, velocity);

        // F_l = 1/2 * rho * S * C_L_alpha * V^2 * alpha_e
        math::Vec3 force = 0.5 * rho * S * C_L_alpha * velocityMagnitudePow2 * alpha_e;

        body.addForce(force);
        m_force = force;
    }

    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Lift";
    std::string m_symbol = "Fl";
};

class Magnus : public IForce {
public:
    void apply(IPhysicsBody& body, PhysicsContext& context, double /*dt*/) override
    {
        // requires projectile body
        auto* projectile = dynamic_cast<projectile::IProjectileBody*>(&body);
        if (!projectile)
        {
            return;
        }

        const auto& specs = projectile->getProjectileSpecs();

        // requers projectile spin specs
        if (!hasSpinDriftData(specs))
        {
            return;
        }
        const auto& spinSpecs = specs.spinSpecs.value();

        // velocity
        math::Vec3 velocity = body.getVelocity();
        double velocityMagnitude = velocity.length();
        if (velocityMagnitude < 1e-3)
        {
            return;
        }

        // data
        double rho = context.airDensity.value_or(constants::BASE_ATMOSPHERIC_DENSITY);
        double d = specs.diameter.value();
        double S = specs.area.value();
        double p = spinSpecs.spinRate.value();
        double C_mag_f = spinSpecs.magnusCoefficient;

        // yaw of repose
        math::Vec3 alpha_e = calculateYawOfRepose(specs, context, velocity);
        math::Vec3 alphaCrossV = alpha_e.cross(velocity);

        // F_m = -1/2 * rho * S * d * p * C_mag_f * (alpha_e x V)
        math::Vec3 force = -0.5 * rho * S * d * p * C_mag_f * alphaCrossV;

        body.addForce(force);
        m_force = force;
    }

    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Magnus";
    std::string m_symbol = "Fm";
};

// spin drift influence
class SpinDrift {
public:
    template<typename PhysicsWorldType>
    static void addTo(PhysicsWorldType& world)
    {
        world.addForce(std::make_unique<Lift>());
        world.addForce(std::make_unique<Magnus>());
    }
};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysics
