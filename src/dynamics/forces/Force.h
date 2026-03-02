/*
 * Force.h
 */

#pragma once

#include "dynamics/PhysicsBody.h"
#include "dynamics/PhysicsContext.h"
#include "math/Vec3.h"

#include <string>

namespace BulletPhysics {
namespace dynamics {
namespace forces {

// base interface for forces
class IForce {
public:
    virtual ~IForce() = default;

    // apply force to physics body using context
    virtual void apply(IPhysicsBody& body, PhysicsContext& context, double dt) = 0;

    // check if this force should be active
    virtual bool isActive() const { return true; }

    // getters
    virtual const std::string& getName() const = 0;
    virtual const std::string& getSymbol() const = 0;
    virtual math::Vec3 getForce() const { return m_force; }

protected:
    math::Vec3 m_force{0.0, 0.0, 0.0};
};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysics
