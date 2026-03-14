/*
 * Coriolis.h
 */

#pragma once

#include "Force.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace forces {

class Coriolis : public IForce {
public:

    void apply(IPhysicsBody& body, PhysicsContext& context) override
    {
        // requires Geographic environment
        if (!context.latitude.has_value())
        {
            return;
        }

        double latitude = *context.latitude;
        math::Vec3 velocity = body.getVelocity();

        // Earth's angular velocity vector in ENU frame
        // omega = (0, omega*cos(lat), omega*sin(lat))
        // our coordinate system: x=East, y=Up, z=North
        double omegaNorth = constants::EARTH_ANGULAR_SPEED * std::cos(latitude);
        double omegaUp = constants::EARTH_ANGULAR_SPEED * std::sin(latitude);
        math::Vec3 omega(0.0, omegaUp, omegaNorth);

        // Coriolis acceleration: a = -2 * (omega x v)
        math::Vec3 coriolisAccel = -2.0 * omega.cross(velocity);

        // apply force: F = m * a
        if (body.getMass() > 0.0)
        {
            math::Vec3 force = coriolisAccel * body.getMass();
            body.addForce(force);
            m_force = force;
        }
    }

    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Coriolis";
    std::string m_symbol = "Fc";
};

} // namespace forces
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
