/*
 * Collision.h
 */

#pragma once

#include "collision/collider/Collider.h"

#include <vector>

namespace BulletPhysics {
namespace collision {

struct Manifold {
    collider::Collider* colliderA;
    collider::Collider* colliderB;
    collider::CollisionInfo info;
};

class Collision {
public:
    void addCollider(collider::Collider* collider);
    void removeCollider(collider::Collider* collider);
    void clear();

    void detect(std::vector<Manifold>& manifolds);

private:
    std::vector<collider::Collider*> m_colliders;
};

} // namespace collision
} // namespace BulletPhysics
