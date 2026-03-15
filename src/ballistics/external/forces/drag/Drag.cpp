/*
 * Drag.cpp
 */

#include "Drag.h"

#include "PhysicsBody.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace forces {

void Drag::apply(IPhysicsBody& body, PhysicsContext& context)
{
    math::Vec3 velocity = body.getVelocity();

    // apply wind
    velocity = velocity - context.wind;

    double velocityMagnitude = velocity.length();

    // air density from context
    double rho = context.airDensity;

    // Mach = u / c
    double mach = velocityMagnitude / constants::ISA_SPEED_OF_SOUND;

    double cd = constants::DEFAULT_C_D;
    double area = constants::DEFAULT_AREA;

    // try to use projectile specs if available
    auto* proj = dynamic_cast<::BulletPhysics::projectile::IProjectileBody*>(&body);
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

} // namespace forces
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
