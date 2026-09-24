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
    integrateForces(dt);
    integratePoses(dt);
    syncColliders();

    sweepFast(dt);

    detectContacts();
    solveVelocity();
    solvePosition();

    syncColliders();
    reportContacts();

    // islands rebuilt from this step contacts, pile sleeps as one piece
    m_islands.build(m_bodies, m_manifolds);
    m_islands.updateSleep(dt);
}

void PhysicsWorld::integrateForces(double dt)
{
    for (RigidBody* body : m_bodies)
    {
        // kinematic bodies carry given velocity, nothing acts on them
        if (!body->isDynamic() || body->isSleeping())
        {
            continue;
        }

        const math::Vec3 acceleration = m_gravity + body->getAccumulatedForces() * body->getInverseMass();
        const math::Vec3 angularAcceleration = body->getInverseInertia() * body->getAccumulatedTorque();

        body->applyImpulse(acceleration * dt, angularAcceleration * dt);
    }
}

void PhysicsWorld::integratePoses(double dt)
{
    for (RigidBody* body : m_bodies)
    {
        if (!body->isMovable() || body->isSleeping())
        {
            continue;
        }

        body->advance(dt);

        if (body->isDynamic())
        {
            body->applyDamping(dt);
        }

        body->clearForces();
    }
}

void PhysicsWorld::sweepFast(double dt)
{
    for (auto* collider : m_colliders)
    {
        RigidBody* body = collider->getBody();

        if (!body || !body->isContinuous() || !body->isMovable() || body->isSleeping())
        {
            continue;
        }

        const math::Vec3 travel = body->getVelocity() * dt;
        const double distance = travel.length();

        const double radius = collider->boundingRadius();

        // step shorter than narrowest way through cannot skip anything, usual test sees it
        if (distance < collider->thinnestExtent())
        {
            continue;
        }

        // where it stood before this step, sweep starts from there
        const math::Vec3 from = body->getPosition() - travel;

        const collision::Sweep query{from, travel * (1.0 / distance), distance, radius};

        collision::SweepHit hit;
        if (!sweep(query, hit, collider))
        {
            continue;
        }

        // pull it back to touch, solver takes contact from there
        body->separate(from + query.direction * hit.distance - body->getPosition());

        collider->setPosition(body->getPosition());
    }
}

void PhysicsWorld::detectContacts()
{
    // detect fills manifolds anew, so whatever stood there becomes last step
    m_previous.swap(m_manifolds);

    m_collision.detect(m_manifolds);

    carryImpulses();
}

void PhysicsWorld::solveVelocity()
{
    m_solver.prepare(m_manifolds);
    m_solver.warmStart(m_manifolds);

    // impulses accumulate over passes, each contact sees what others held
    for (int iteration = 0; iteration < m_solverIterations; iteration++)
    {
        for (auto& manifold : m_manifolds)
        {
            m_solver.solveVelocity(manifold);
        }
    }
}

void PhysicsWorld::solvePosition()
{
    for (const auto& manifold : m_manifolds)
    {
        m_solver.correctPosition(manifold);
    }
}

// helpers

// pair is same contact as long as both colliders match
static bool samePair(const collision::Manifold& a, const collision::Manifold& b)
{
    return a.colliderA == b.colliderA && a.colliderB == b.colliderB;
}

void PhysicsWorld::reportContacts() const
{
    if (!m_listener)
    {
        return;
    }

    for (const auto& manifold : m_manifolds)
    {
        const bool known = std::any_of(m_previous.begin(), m_previous.end(), [&manifold](const auto& old) {
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

    for (const auto& old : m_previous)
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
        const RigidBody* body = collider->getBody();

        // one without body stands where put, nothing carries it
        if (body)
        {
            collider->place(body->getPosition(), body->getOrientation());
        }
    }
}

void PhysicsWorld::carryImpulses()
{
    for (auto& manifold : m_manifolds)
    {
        const auto match = std::find_if(m_previous.begin(), m_previous.end(), [&manifold](const auto& old) {
            return old.colliderA == manifold.colliderA && old.colliderB == manifold.colliderB;
        });

        if (match == m_previous.end())
        {
            continue;
        }

        // matched by what produced them, order shifts as bodies turn
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

    // always face ray, one starting inside gets far side otherwise
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

        // pair still has to accept each other, sweep is no way around layers
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

    // sphere centre where it stopped, surface is one radius closer
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
    // body listed twice integrates twice
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
        m_listed.insert(collider);
        m_collision.addCollider(collider);
    }
}

void PhysicsWorld::addCollider(collision::collider::Collider* collider)
{
    if (!collider || !m_listed.insert(collider).second)
    {
        return;
    }

    m_colliders.push_back(collider);
    m_collision.addCollider(collider);
}

void PhysicsWorld::removeCollider(collision::collider::Collider* collider)
{
    if (m_listed.erase(collider) == 0)
    {
        return;
    }

    m_collision.removeCollider(collider);
    m_colliders.erase(std::find(m_colliders.begin(), m_colliders.end(), collider));
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
        m_listed.erase(collider);
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
    m_listed.clear();
    m_collision.clear();
    m_manifolds.clear();
}

} // namespace dynamics
} // namespace BulletPhysics
