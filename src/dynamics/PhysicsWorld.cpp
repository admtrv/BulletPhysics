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
    sweepFast(dt);
    collide();

    // islands are rebuilt from this step contacts, a pile sleeps as one piece
    m_islands.build(m_bodies, m_manifolds);
    m_islands.updateSleep(dt);
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
            const math::Vec3 angularAcceleration = body->getInverseInertia() * body->getAccumulatedTorque();

            body->applyImpulse(acceleration * dt, angularAcceleration * dt);
        }

        body->advance(dt);

        if (body->isDynamic())
        {
            body->applyDamping(dt);
        }

        body->clearForces();
    }

    syncColliders();
}

void PhysicsWorld::sweepFast(double dt)
{
    for (auto* collider : m_colliders)
    {
        RigidBody* body = collider->getBody();

        if (!body->isContinuous() || !body->isMovable() || body->isSleeping())
        {
            continue;
        }

        const math::Vec3 travel = body->getVelocity() * dt;
        const double distance = travel.length();

        const double radius = collider->boundingRadius();

        // a step shorter than the shape cannot skip anything, the usual test sees it
        if (distance < radius)
        {
            continue;
        }

        // where it stood before this step, the sweep starts from there
        const math::Vec3 from = body->getPosition() - travel;

        const collision::Sweep query{from, travel * (1.0 / distance), distance, radius};

        collision::SweepHit hit;
        if (!sweep(query, hit, collider))
        {
            continue;
        }

        // pull it back to the touch, the solver takes the contact from there
        body->separate(from + query.direction * hit.distance - body->getPosition());

        collider->setPosition(body->getPosition());
    }
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

bool PhysicsWorld::sweep(const collision::Sweep& sweep, collision::SweepHit& outHit,
                         const collision::collider::Collider* ignore,
                         collision::collider::LayerMask mask) const
{
    outHit = {};

    double nearest = sweep.distance;

    for (auto* collider : m_colliders)
    {
        if (collider == ignore || (mask & collider->getLayer()) == 0 || collider->isTrigger())
        {
            continue;
        }

        // the pair still has to accept each other, a sweep is no way around the layers
        if (ignore && !ignore->collidesWith(*collider))
        {
            continue;
        }

        double distance = 0.0;
        if (!collider->sweep(sweep, distance) || distance >= nearest)
        {
            continue;
        }

        nearest = distance;

        outHit.collider = collider;
        outHit.distance = distance;
    }

    if (!outHit.collider)
    {
        return false;
    }

    // the sphere centre where it stopped, the surface is one radius closer
    const math::Vec3 centre = sweep.origin + sweep.direction * outHit.distance;

    outHit.normal = outHit.collider->normalAt(centre);

    if (outHit.normal.dot(sweep.direction) > 0.0)
    {
        outHit.normal = outHit.normal * -1.0;
    }

    outHit.point = centre + outHit.normal * -sweep.radius;

    return true;
}

// contents

void PhysicsWorld::addBody(RigidBody* body, collision::collider::Collider* collider)
{
    // a body listed twice would be integrated twice
    if (!body || std::find(m_bodies.begin(), m_bodies.end(), body) != m_bodies.end())
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
    // whatever leaned on it has to fall now
    m_islands.wake(body);

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
