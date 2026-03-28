/*
 * PhysicsWorld.h
 */

#pragma once

#include "ballistics/external/forces/Force.h"
#include "ballistics/external/environments/Environment.h"
#include "geography/CoordinateMapping.h"

#include <vector>

namespace BulletPhysics {
namespace ballistics {
namespace external {

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
    void addEnvironment(std::unique_ptr<environments::IEnvironment> environment);

    // remove all
    void clear();

    // apply all forces to physics body
    void applyForces(IPhysicsBody& body);

    // getters
    forces::IForce* getForce(const std::string& name);
    environments::IEnvironment* getEnvironment(const std::string& name);

    const std::vector<std::unique_ptr<forces::IForce>>& getForces() const { return m_forces; }
    const std::vector<std::unique_ptr<environments::IEnvironment>>& getEnvironments() const { return m_environments; }

    // coordinate mapping
    void setCoordinateMapping(const geography::CoordinateMapping& mapping);
    const geography::CoordinateMapping& getCoordinateMapping() const;

    // counts
    size_t forceCount() const { return m_forces.size(); }
    size_t environmentCount() const { return m_environments.size(); }

private:
    std::vector<std::unique_ptr<forces::IForce>> m_forces;
    std::vector<std::unique_ptr<environments::IEnvironment>> m_environments;

    PhysicsContext m_context;
    geography::CoordinateMapping m_coordinateMapping = geography::mappings::EUN();
};

} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
