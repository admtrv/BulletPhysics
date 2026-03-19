/*
 * Coriolis.cpp
 */

#include "Coriolis.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace forces {

void Coriolis::apply(IPhysicsBody& body, PhysicsContext& context)
{
    double latitude = context.latitude;
    math::Vec3 velocity = body.getVelocity();

    // Earth's angular velocity vector in ENU frame
    // omega = (0, omega*cos(lat), omega*sin(lat))
    // our coordinate system: x=East, y=Up, z=North
    double omegaNorth = constants::WGS84_EARTH_ANGULAR_SPEED * std::cos(latitude);
    double omegaUp = constants::WGS84_EARTH_ANGULAR_SPEED * std::sin(latitude);
    math::Vec3 omega(0.0, omegaUp, omegaNorth);

    // Coriolis acceleration: a = -2 * (omega x v)
    math::Vec3 coriolisAccel = -2.0 * omega.crossEUN(velocity);

    // apply force: F = m * a
    if (body.getMass() > 0.0)
    {
        math::Vec3 force = coriolisAccel * body.getMass();

        m_force = force;
        body.addForce(force);
    }
}

} // namespace forces
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
