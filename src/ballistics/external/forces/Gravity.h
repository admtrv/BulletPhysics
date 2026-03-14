/*
 * Gravity.h
 */

#pragma once

#include "Force.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace forces {

class Gravity : public IForce {
public:
    // gravity: F = m * g
    void apply(IPhysicsBody& body, PhysicsContext& context) override
    {
        if (body.getMass() > 0.0)
        {
            math::Vec3 force;

            // get corrected gravity acceleration from context or use classic constant
            if (context.gravity.has_value())
            {
                math::Vec3 g = math::Vec3{0.0, -*context.gravity, 0.0};
                force = body.getMass() * g;
            }
            else
            {
                math::Vec3 g = constants::GRAVITY;
                force = body.getMass() * g;
            }

            m_force = force;
            body.addForce(force);
        }
    }

    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Gravitational";
    std::string m_symbol = "Fg";
};

} // namespace forces
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
