/*
 * Wind.cpp
 */

#include "Wind.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace environments {

Wind::Wind(const math::Vec3& windVelocity) : m_velocity(windVelocity) {}

void Wind::update(IPhysicsBody& /*body*/, PhysicsContext& context)
{
    context.wind = m_velocity;
}

} // namespace environments
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
