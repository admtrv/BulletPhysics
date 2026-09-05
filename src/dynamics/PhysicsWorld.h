/*
 * PhysicsWorld.h
 */

#pragma once

#include "collision/Collision.h"
#include "dynamics/ContactSolver.h"
#include "dynamics/RigidBody.h"

#include <vector>

namespace BulletPhysics {
namespace dynamics {

inline const math::Vec3 DEFAULT_GRAVITY{0.0, -9.80665, 0.0};

// simulation world, holds bodies and advances them
class PhysicsWorld {
public:
    PhysicsWorld() = default;

    // simulation
    void step(double dt);

    // bodies, not owned
    // collider is optional, body without one passes through everything
    void addBody(RigidBody* body, collision::collider::Collider* collider = nullptr);
    void removeBody(RigidBody* body);

    void clear();

    // gravitation
    const math::Vec3& getGravity() const { return m_gravity; }
    void setGravity(const math::Vec3& gravity) { m_gravity = gravity; }

    // getters
    const std::vector<RigidBody*>& getBodies() const { return m_bodies; }
    const std::vector<collision::Manifold>& getContacts() const { return m_manifolds; }

    // counts
    size_t getBodyCount() const { return m_bodies.size(); }

private:
    // step phases
    void integrate(double dt);
    void collide();

    // contents
    std::vector<RigidBody*> m_bodies;
    std::vector<collision::collider::Collider*> m_colliders;

    // parameters
    math::Vec3 m_gravity = DEFAULT_GRAVITY;

    // machinery
    collision::Collision m_collision;
    ContactSolver m_solver;
    std::vector<collision::Manifold> m_manifolds;
};

} // namespace dynamics
} // namespace BulletPhysics
