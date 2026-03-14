/*
 * Collision.cpp
 */

#include "Collision.h"

namespace BulletPhysics {
namespace builtin {
namespace collision {

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

void Collision::detect(std::vector<Manifold>& manifolds)
{
    manifolds.clear();

    for (size_t i = 0; i < m_colliders.size(); i++)
    {
        for (size_t j = i + 1; j < m_colliders.size(); j++)
        {
            Collider* a = m_colliders[i];
            Collider* b = m_colliders[j];

            CollisionInfo info;
            if (a->testCollision(*b, info))
            {
                manifolds.push_back({a, b, info});
            }
            else if (b->testCollision(*a, info))
            {
                manifolds.push_back({b, a, info});
            }
        }
    }
}

} // namespace collision
} // namespace builtin
} // namespace BulletPhysics
