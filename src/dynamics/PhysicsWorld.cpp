/*
 * PhysicsWorld.cpp
 */

#include "PhysicsWorld.h"

#include <algorithm>

namespace BulletPhysics {
namespace dynamics {

// simulation

void PhysicsWorld::step(double dt)
{
    integrate(dt);
    collide();
}

void PhysicsWorld::integrate(double dt)
{
    for (RigidBody* body : m_bodies)
    {
        if (!body->isDynamic())
        {
            continue;
        }

        // a = g + F / m
        const math::Vec3 acceleration = m_gravity + body->getAccumulatedForces() * body->getInverseMass();

        // semi-implicit euler, velocity first then position with the new velocity
        // stays stable when solver changes velocities between steps
        body->setVelocity(body->getVelocity() + acceleration * dt);
        body->setPosition(body->getPosition() + body->getVelocity() * dt);

        body->clearForces();
    }

    for (auto* collider : m_colliders)
    {
        collider->setPosition(collider->getBody()->getPosition());
    }
}

void PhysicsWorld::collide()
{
    m_collision.detect(m_manifolds);

    for (const auto& manifold : m_manifolds)
    {
        m_solver.resolve(manifold);

        // solver moved the bodies, keep colliders in sync for the next pair
        manifold.colliderA->setPosition(manifold.colliderA->getBody()->getPosition());
        manifold.colliderB->setPosition(manifold.colliderB->getBody()->getPosition());
    }
}

// contents

void PhysicsWorld::addBody(RigidBody* body, collision::collider::Collider* collider)
{
    if (!body)
    {
        return;
    }

    m_bodies.push_back(body);

    if (collider)
    {
        collider->setBody(body);
        collider->setPosition(body->getPosition());

        m_colliders.push_back(collider);
        m_collision.addCollider(collider);
    }
}

void PhysicsWorld::removeBody(RigidBody* body)
{
    auto it = std::find(m_bodies.begin(), m_bodies.end(), body);
    if (it != m_bodies.end())
    {
        m_bodies.erase(it);
    }

    // colliders go with their body
    for (auto colliderIt = m_colliders.begin(); colliderIt != m_colliders.end();)
    {
        if ((*colliderIt)->getBody() == body)
        {
            m_collision.removeCollider(*colliderIt);
            (*colliderIt)->setBody(nullptr);

            colliderIt = m_colliders.erase(colliderIt);
        }
        else
        {
            ++colliderIt;
        }
    }
}

void PhysicsWorld::clear()
{
    for (auto* collider : m_colliders)
    {
        collider->setBody(nullptr);
    }

    m_bodies.clear();
    m_colliders.clear();
    m_collision.clear();
    m_manifolds.clear();
}

} // namespace dynamics
} // namespace BulletPhysics
