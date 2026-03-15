/*
 * Gravity.cpp
 */

#include "Gravity.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace forces {

// gravity: F = m * g
void Gravity::apply(IPhysicsBody& body, PhysicsContext& context)
{
    if (body.getMass() > 0.0)
    {
        math::Vec3 force = body.getMass() * context.gravity;

        m_force = force;
        body.addForce(force);
    }
}

} // namespace forces
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
