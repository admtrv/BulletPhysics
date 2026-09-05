/*
 * PhysicsWorld.cpp
 */

#include "PhysicsWorld.h"

#include <algorithm>

namespace BulletPhysics {
namespace dynamics {

// motion bled off per second, keeps bodies from drifting forever
static constexpr double LINEAR_DAMPING = 0.05;
static constexpr double ANGULAR_DAMPING = 0.5;

// below this a body counts as standing still
static constexpr double SLEEP_LINEAR_SPEED = 0.05;
static constexpr double SLEEP_ANGULAR_SPEED = 0.05;

// simulation

int PhysicsWorld::update(double frameTime)
{
    const int steps = m_timer.consume(frameTime);

    for (int i = 0; i < steps; i++)
    {
        step(m_timer.getTimeStep());
    }

    return steps;
}

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

        const math::Vec3 acceleration = m_gravity + body->getAccumulatedForces() * body->getInverseMass();

        // semi-implicit euler
        body->setVelocity(body->getVelocity() + acceleration * dt);
        body->setPosition(body->getPosition() + body->getVelocity() * dt);

        const math::Vec3 angularAcceleration = body->getInverseInertia() * body->getAccumulatedTorque();

        body->setAngularVelocity(body->getAngularVelocity() + angularAcceleration * dt);

        // q' = q + 0.5 * w * q * dt, w as quaternion with zero scalar part
        const math::Vec3& angularVelocity = body->getAngularVelocity();
        const math::Quat spin{0.0, angularVelocity.x, angularVelocity.y, angularVelocity.z};

        body->setOrientation(body->getOrientation() + spin * body->getOrientation() * (0.5 * dt));

        // bleed off motion, nothing comes to rest otherwise
        body->setVelocity(body->getVelocity() * std::max(0.0, 1.0 - LINEAR_DAMPING * dt));
        body->setAngularVelocity(body->getAngularVelocity() * std::max(0.0, 1.0 - ANGULAR_DAMPING * dt));

        body->clearForces();
    }

    for (auto* collider : m_colliders)
    {
        collider->setPosition(collider->getBody()->getPosition());
        collider->setOrientation(collider->getBody()->getOrientation());
    }
}

void PhysicsWorld::collide()
{
    m_collision.detect(m_manifolds);

    // one pass leaves stacks sagging, contact sees only what came before
    for (int iteration = 0; iteration < m_solverIterations; iteration++)
    {
        for (const auto& manifold : m_manifolds)
        {
            m_solver.solveVelocity(manifold);
        }
    }

    for (const auto& manifold : m_manifolds)
    {
        m_solver.correctPosition(manifold);
    }

    // motion this slow is solver noise, body would creep forever
    for (const auto& manifold : m_manifolds)
    {
        for (auto* collider : {manifold.colliderA, manifold.colliderB})
        {
            RigidBody* body = collider->getBody();
            if (!body || !body->isDynamic())
            {
                continue;
            }

            if (body->getVelocity().length() < SLEEP_LINEAR_SPEED)
            {
                body->setVelocity({});
            }

            if (body->getAngularVelocity().length() < SLEEP_ANGULAR_SPEED)
            {
                body->setAngularVelocity({});
            }
        }
    }

    for (auto* collider : m_colliders)
    {
        collider->setPosition(collider->getBody()->getPosition());
        collider->setOrientation(collider->getBody()->getOrientation());
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
        collider->setOrientation(body->getOrientation());

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
