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
            math::Vec3 force = body.getMass() * context.gravity;

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
