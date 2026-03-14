/*
 * GroundCollider.cpp
 */

#include "GroundCollider.h"
#include "BoxCollider.h"

namespace BulletPhysics {
namespace builtin {
namespace collision {
namespace collider {

GroundCollider::GroundCollider(double groundLevel) : m_position{0.0, groundLevel, 0.0} {}

void GroundCollider::setPosition(const math::Vec3& pos)
{
    m_position.y = pos.y;
}

void GroundCollider::setGroundY(double level)
{
    m_position.y = level;
}

bool GroundCollider::testCollision(const Collider& other, CollisionInfo& outInfo) const
{
    switch (other.getShape()) {
        case CollisionShape::Box: {
            return testCollisionWithBox(dynamic_cast<const BoxCollider&>(other), outInfo);
        }
        case CollisionShape::Ground: {
            // two ground planes don't collide
            return false;
        }
        default:
            return false;
    }
}

bool GroundCollider::testPoint(const math::Vec3& point) const
{
    return point.y <= m_position.y;
}

bool GroundCollider::testCollisionWithBox(const BoxCollider& box, CollisionInfo& outInfo) const
{
    // box will test collision with us
    return false;
}

} // namespace collider
} // namespace collision
} // namespace builtin
} // namespace BulletPhysics
