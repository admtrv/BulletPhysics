/*
 * PhysicsWorld.h
 */

#pragma once

#include "PhysicsContext.h"
#include "PhysicsBody.h"
#include "forces/Force.h"
#include "environment/Environment.h"

#include <vector>
#include <memory>
#include <string>

namespace BulletPhysics {
namespace dynamics {

// physics world manages forces and environment
class PhysicsWorld {
public:
    PhysicsWorld() = default;
    ~PhysicsWorld() = default;

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    PhysicsWorld(PhysicsWorld&&) = default;
    PhysicsWorld& operator=(PhysicsWorld&&) = default;

    // add force or environment
    void addForce(std::unique_ptr<forces::IForce> force);
    void addEnvironment(std::unique_ptr<environment::IEnvironment> environment);

    // remove all
    void clear();

    // apply all forces to physics body
    void applyForces(IPhysicsBody& body, double dt);

    // getters
    forces::IForce* getForce(const std::string& name);
    environment::IEnvironment* getEnvironment(const std::string& name);

    const std::vector<std::unique_ptr<forces::IForce>>& getForces() const { return m_forces; }
    const std::vector<std::unique_ptr<environment::IEnvironment>>& getEnvironments() const { return m_environments; }

    // counts
    size_t forceCount() const { return m_forces.size(); }
    size_t environmentCount() const { return m_environments.size(); }

private:
    std::vector<std::unique_ptr<forces::IForce>> m_forces;
    std::vector<std::unique_ptr<environment::IEnvironment>> m_environments;

    PhysicsContext m_context;
};

} // namespace dynamics
} // namespace BulletPhysics
