/*
 * Environment.h
 */

#pragma once

#include "Constants.h"
#include "PhysicsBody.h"
#include "../PhysicsContext.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace environments {

// base interface for environment providers
class IEnvironment {
public:
    virtual ~IEnvironment() = default;

    // update physics context based on physics body state
    virtual void update(IPhysicsBody& body, PhysicsContext& context) = 0;

    // getters
    virtual const std::string& getName() const = 0;
    virtual int getPriority() const = 0;                // evaluation order: lower priority runs first (like z-index in graphics)
};

} // namespace environments
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
