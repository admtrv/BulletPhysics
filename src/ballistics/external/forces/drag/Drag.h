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

        // apply wind
        velocity = velocity - context.wind;

        double velocityMagnitude = velocity.length();

        // air density from context
        double rho = context.airDensity;

        // Mach = u / c
        double mach = velocityMagnitude / constants::BASE_SPEED_OF_SOUND;

        double cd = constants::DEFAULT_CD;
        double area = constants::DEFAULT_AREA;

        // try to use projectile specs if available
        auto* proj = dynamic_cast<projectile::IProjectileBody*>(&body);
        if (proj)
        {
            const auto& specs = proj->getProjectileSpecs();
            if (specs.dragModel)
            {
                cd = specs.dragModel->getCd(mach);
            }
            area = specs.area;
        }

        // F_d = -0.5 * rho * S * Cd * v * |v|
        math::Vec3 force = -0.5 * rho * area * cd * velocity * velocityMagnitude;

        m_force = force;
        body.addForce(force);
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
