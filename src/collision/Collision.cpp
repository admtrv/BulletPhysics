/*
 * Collision.cpp
 */

#include "Collision.h"

#include "dynamics/body/RigidBody.h"

#include <algorithm>

namespace BulletPhysics {
namespace collision {

using namespace collider;

void Collision::addCollider(Collider* collider)
{
    if (collider)
    {
        m_colliders.push_back(collider);
    }
}

void Collision::removeCollider(Collider* collider)
{
    auto it = std::find(m_colliders.begin(), m_colliders.end(), collider);
    if (it != m_colliders.end())
    {
        m_colliders.erase(it);
    }
}

void Collision::clear()
{
    m_colliders.clear();
}

// what shape can do about overlap, one held in place does nothing
static collider::Mobility mobilityOf(const Collider& collider)
{
    const dynamics::RigidBody* body = collider.getBody();

    collider::Mobility mobility;

    for (int axis = 0; axis < 3; axis++)
    {
        mobility.axes[axis] = body && body->canMoveAlong(axis);
    }

    return mobility;
}

void Collision::detect(std::vector<Manifold>& manifolds)
{
    manifolds.clear();

    for (size_t i = 0; i < m_colliders.size(); i++)
    {
        for (size_t j = i + 1; j < m_colliders.size(); j++)
        {
            Collider* a = m_colliders[i];
            Collider* b = m_colliders[j];

            if (!a->collidesWith(*b))
            {
                continue;
            }

            const collider::PairMobility mobility{mobilityOf(*a), mobilityOf(*b)};

            CollisionInfo info;
            info.mobility = mobility;

            if (a->testCollision(*b, info))
            {
                manifolds.push_back({a, b, info});
                continue;
            }

            // failed test leaves marks behind, so other side starts clean
            info = CollisionInfo{};
            info.mobility = {mobility.second, mobility.first};

            if (b->testCollision(*a, info))
            {
                manifolds.push_back({b, a, info});
            }
        }
    }
}

} // namespace collision
} // namespace BulletPhysics
