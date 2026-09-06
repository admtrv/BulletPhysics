/*
 * PhysicsWorld.cpp
 */

#include "PhysicsWorld.h"

#include <algorithm>

namespace BulletPhysics {
namespace dynamics {

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

    // after the solver, gravity leaves a resting body drifting until contacts cancel it
    for (RigidBody* body : m_bodies)
    {
        if (body->isDynamic())
        {
            body->updateSleep(dt);
        }
    }
}

void PhysicsWorld::integrate(double dt)
{
    for (RigidBody* body : m_bodies)
    {
        if (!body->isMovable() || body->isSleeping())
        {
            continue;
        }

        // kinematic bodies carry the velocity they were given, nothing acts on them
        if (body->isDynamic())
        {
            const math::Vec3 acceleration = m_gravity + body->getAccumulatedForces() * body->getInverseMass();
            body->setVelocity(body->getVelocity() + acceleration * dt);

            const math::Vec3 angularAcceleration = body->getInverseInertia() * body->getAccumulatedTorque();
            body->setAngularVelocity(body->getAngularVelocity() + angularAcceleration * dt);
        }

        // semi-implicit euler
        body->setPosition(body->getPosition() + body->getVelocity() * dt);

        // q' = q + 0.5 * w * q * dt, w as quaternion with zero scalar part
        const math::Vec3& angularVelocity = body->getAngularVelocity();
        const math::Quat spin{0.0, angularVelocity.x, angularVelocity.y, angularVelocity.z};

        body->setOrientation(body->getOrientation() + spin * body->getOrientation() * (0.5 * dt));

        if (body->isDynamic())
        {
            body->applyDamping(dt);
        }

        body->clearForces();
    }

    syncColliders();
}

void PhysicsWorld::collide()
{
    std::vector<collision::Manifold> previous;
    previous.swap(m_manifolds);

    m_collision.detect(m_manifolds);

    carryImpulses(previous);

    m_solver.prepare(m_manifolds);
    m_solver.warmStart(m_manifolds);

    // impulses accumulate over the passes, each contact sees what the others held
    for (int iteration = 0; iteration < m_solverIterations; iteration++)
    {
        for (auto& manifold : m_manifolds)
        {
            m_solver.solveVelocity(manifold);
        }
    }

    for (const auto& manifold : m_manifolds)
    {
        m_solver.correctPosition(manifold);
    }

    syncColliders();

    reportContacts(previous);
}

// helpers

// a pair is the same contact as long as both colliders match
static bool samePair(const collision::Manifold& a, const collision::Manifold& b)
{
    return a.colliderA == b.colliderA && a.colliderB == b.colliderB;
}

void PhysicsWorld::reportContacts(const std::vector<collision::Manifold>& previous) const
{
    if (!m_listener)
    {
        return;
    }

    for (const auto& manifold : m_manifolds)
    {
        const bool known = std::any_of(previous.begin(), previous.end(), [&manifold](const auto& old) {
            return samePair(old, manifold);
        });

        if (known)
        {
            m_listener->onContactStay(manifold);
        }
        else
        {
            m_listener->onContactBegin(manifold);
        }
    }

    for (const auto& old : previous)
    {
        const bool alive = std::any_of(m_manifolds.begin(), m_manifolds.end(), [&old](const auto& manifold) {
            return samePair(old, manifold);
        });

        if (!alive)
        {
            m_listener->onContactEnd(old.colliderA, old.colliderB);
        }
    }
}

void PhysicsWorld::syncColliders()
{
    for (auto* collider : m_colliders)
    {
        collider->setPosition(collider->getBody()->getPosition());
        collider->setOrientation(collider->getBody()->getOrientation());
    }
}

void PhysicsWorld::carryImpulses(const std::vector<collision::Manifold>& previous)
{
    for (auto& manifold : m_manifolds)
    {
        const auto match = std::find_if(previous.begin(), previous.end(), [&manifold](const auto& old) {
            return old.colliderA == manifold.colliderA && old.colliderB == manifold.colliderB;
        });

        if (match == previous.end())
        {
            continue;
        }

        // matched by what produced them, their order shifts as the bodies turn
        for (int i = 0; i < manifold.info.pointCount; i++)
        {
            auto& point = manifold.info.points[i];

            for (int j = 0; j < match->info.pointCount; j++)
            {
                const auto& old = match->info.points[j];

                if (point.feature == old.feature)
                {
                    point.normalImpulse = old.normalImpulse;
                    point.tangentImpulses[0] = old.tangentImpulses[0];
                    point.tangentImpulses[1] = old.tangentImpulses[1];

                    break;
                }
            }
        }
    }
}

// queries

bool PhysicsWorld::raycast(const collision::Ray& ray, collision::RayHit& outHit,
                           collision::collider::LayerMask mask) const
{
    outHit = {};

    double nearest = ray.maxDistance;

    for (auto* collider : m_colliders)
    {
        if ((mask & collider->getLayer()) == 0)
        {
            continue;
        }

        double distance = 0.0;
        if (!collider->raycast(ray, distance) || distance >= nearest)
        {
            continue;
        }

        nearest = distance;

        outHit.collider = collider;
        outHit.distance = distance;
        outHit.point = ray.pointAt(distance);
    }

    if (!outHit.collider)
    {
        return false;
    }

    outHit.normal = outHit.collider->normalAt(outHit.point);

    // always face the ray, a ray starting inside would get the far side otherwise
    if (outHit.normal.dot(ray.direction) > 0.0)
    {
        outHit.normal = outHit.normal * -1.0;
    }

    return true;
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

    const auto detached = std::remove_if(m_colliders.begin(), m_colliders.end(), [this, body](auto* collider) {
        if (collider->getBody() != body)
        {
            return false;
        }

        m_collision.removeCollider(collider);
        collider->setBody(nullptr);

        return true;
    });

    m_colliders.erase(detached, m_colliders.end());
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
