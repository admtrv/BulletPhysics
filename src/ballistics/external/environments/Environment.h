/*
 * Environment.h
 */

#pragma once

#include "PhysicsBody.h"
#include "ballistics/external/PhysicsContext.h"

#include <string>

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

    virtual const std::string& getName() const = 0;
};

} // namespace environments
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
