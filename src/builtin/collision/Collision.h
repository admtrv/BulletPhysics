/*
 * Collision.h
 */

#pragma once

#include "builtin/collision/collider/Collider.h"

#include <algorithm>
#include <vector>

namespace BulletPhysics {
namespace builtin {
namespace collision {

using namespace collider;

struct Manifold {
    Collider* colliderA;
    Collider* colliderB;
    CollisionInfo info;
};

class Collision {
public:
    void addCollider(Collider* collider);
    void removeCollider(Collider* collider);
    void clear();

    void detect(std::vector<Manifold>& manifolds);

private:
    std::vector<Collider*> m_colliders;
};

} // namespace collision
} // namespace builtin
} // namespace BulletPhysics
