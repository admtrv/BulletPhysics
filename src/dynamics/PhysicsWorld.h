/*
 * PhysicsWorld.h
 */

#pragma once

#include "collision/Collision.h"
#include "dynamics/body/IslandManager.h"
#include "dynamics/body/RigidBody.h"
#include "dynamics/contact/ContactListener.h"
#include "dynamics/contact/ContactSolver.h"
#include "dynamics/PhysicsTimer.h"

#include <vector>

namespace BulletPhysics {
namespace dynamics {

inline const math::Vec3 DEFAULT_GRAVITY{0.0, -9.80665, 0.0};

// simulation world, holds bodies and advances them
class PhysicsWorld {
public:
    PhysicsWorld() = default;

    // simulation
    int update(double frameTime);   // runs as many fixed steps as frame time owes
    void step(double dt);           // one step, for own clock

    // contents, not owned
    void addBody(RigidBody* body, collision::collider::Collider* collider = nullptr);   // no collider means no contacts
    void removeBody(RigidBody* body);

    void clear();

    // queries
    bool raycast(const collision::Ray& ray, collision::RayHit& outHit, collision::collider::LayerMask mask = collision::collider::LAYER_ALL) const;

    // parameters
    const math::Vec3& getGravity() const { return m_gravity; }
    void setGravity(const math::Vec3& gravity) { m_gravity = gravity; }

    int getSolverIterations() const { return m_solverIterations; }   // more of them hold stacks better
    void setSolverIterations(int iterations) { m_solverIterations = (iterations > 0 ? iterations : 1); }

    PhysicsTimer& getTimer() { return m_timer; }
    const PhysicsTimer& getTimer() const { return m_timer; }

    void setContactListener(IContactListener* listener) { m_listener = listener; }   // not owned

    // getters
    const std::vector<RigidBody*>& getBodies() const { return m_bodies; }
    const std::vector<collision::Manifold>& getContacts() const { return m_manifolds; }

    // counts
    size_t getBodyCount() const { return m_bodies.size(); }

private:
    // step phases
    void integrate(double dt);
    void collide();

    // helpers
    void carryImpulses(const std::vector<collision::Manifold>& previous);
    void reportContacts(const std::vector<collision::Manifold>& previous) const;

    void syncColliders();

    // contents
    std::vector<RigidBody*> m_bodies;
    std::vector<collision::collider::Collider*> m_colliders;

    // parameters
    math::Vec3 m_gravity = DEFAULT_GRAVITY;
    int m_solverIterations = 20;

    PhysicsTimer m_timer;
    IContactListener* m_listener = nullptr;

    // machinery
    collision::Collision m_collision;
    ContactSolver m_solver;
    IslandManager m_islands;
    std::vector<collision::Manifold> m_manifolds;
};

} // namespace dynamics
} // namespace BulletPhysics
