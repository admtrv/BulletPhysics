/*
 * Integrator.h
 */

#pragma once

#include "PhysicsBody.h"
#include "ballistics/external/PhysicsWorld.h"

namespace BulletPhysics {
namespace math {

class IIntegrator {
public:
    virtual ~IIntegrator() = default;
    virtual void step(IPhysicsBody& body, ballistics::external::PhysicsWorld* world, double dt) = 0;
};

class EulerIntegrator final : public IIntegrator {
public:
    void step(IPhysicsBody& body, ballistics::external::PhysicsWorld* world, double dt) override;
};

class MidpointIntegrator final : public IIntegrator {
public:
    void step(IPhysicsBody& body, ballistics::external::PhysicsWorld* world, double dt) override;
};

class RK4Integrator final : public IIntegrator {
public:
    void step(IPhysicsBody& body, ballistics::external::PhysicsWorld* world, double dt) override;
};

} // namespace math
} // namespace BulletPhysics
