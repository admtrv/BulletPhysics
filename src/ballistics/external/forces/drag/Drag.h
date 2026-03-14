/*
 * Drag.h
 */

#pragma once

#include "ballistics/external/forces/Force.h"
#include "ballistics/external/forces/drag/DragModel.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace forces {

// aerodynamic drag force
class Drag : public IForce {
public:

    void apply(IPhysicsBody& body, PhysicsContext& context) override
    {
        math::Vec3 velocity = body.getVelocity();

        // apply wind if available
        if (context.wind.has_value())
        {
            velocity = velocity - *context.wind;
        }

        double velocityMagnitude = velocity.length();

        // get air density from context or use default
        double rho = context.airDensity.value_or(constants::BASE_ATMOSPHERIC_DENSITY);

        // Mach = u / c
        double mach = velocityMagnitude / constants::BASE_SPEED_OF_SOUND;

        double cd = constants::DEFAULT_CD;
        double area = constants::DEFAULT_AREA;

        // try to use projectile specs if available
        auto* proj = dynamic_cast<projectile::IProjectileBody*>(&body);
        if (proj)
        {
            const auto& specs = proj->getProjectileSpecs();
            if (specs.dragModel.has_value())
            {
                cd = drag::StandardDragModel(specs.dragModel.value()).getCd(mach);
            }

            if (specs.area.has_value())
            {
                area = specs.area.value();
            }
        }

        // F_d = -0.5 * rho * S * Cd * v * |v|
        math::Vec3 force = -0.5 * rho * area * cd * velocity * velocityMagnitude;

        body.addForce(force);
        m_force = force;
    }

    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Aerodynamic Drag";
    std::string m_symbol = "Fd";
};

} // namespace forces
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
