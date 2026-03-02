/*
 * Integrator.h
 */

#pragma once

#include "dynamics/PhysicsBody.h"
#include "dynamics/PhysicsWorld.h"

namespace BulletPhysics {
namespace math {

class IIntegrator {
public:
    virtual ~IIntegrator() = default;
    virtual void step(dynamics::IPhysicsBody& body, dynamics::PhysicsWorld* world, double dt) = 0;
};

class EulerIntegrator final : public IIntegrator {
public:
    void step(dynamics::IPhysicsBody& body, dynamics::PhysicsWorld* world, double dt) override;
};

class MidpointIntegrator final : public IIntegrator {
public:
    void step(dynamics::IPhysicsBody& body, dynamics::PhysicsWorld* world, double dt) override;
};

class RK4Integrator final : public IIntegrator {
public:
    void step(dynamics::IPhysicsBody& body, dynamics::PhysicsWorld* world, double dt) override;
};

} // namespace math
} // namespace BulletPhysics
