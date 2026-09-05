/*
 * GroundCollider.cpp
 */

#include "GroundCollider.h"
#include "BoxCollider.h"
#include "SphereCollider.h"

namespace BulletPhysics {
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
            return testCollisionWithBox(static_cast<const BoxCollider&>(other), outInfo);
        }
        case CollisionShape::Sphere: {
            return testCollisionWithSphere(static_cast<const SphereCollider&>(other), outInfo);
        }
        case CollisionShape::Ground: {
            // two ground planes don't collide
            return false;
        }
        default:
            return false;
    }
}

bool GroundCollider::testCollisionWithBox(const BoxCollider& box, CollisionInfo& outInfo) const
{
    // reuse the box test, then flip the normal so it points from the ground upwards
    if (!box.testCollisionWithGround(*this, outInfo))
    {
        return false;
    }

    outInfo.normal = outInfo.normal * -1.0;
    return true;
}

bool GroundCollider::testCollisionWithSphere(const SphereCollider& sphere, CollisionInfo& outInfo) const
{
    // reuse the sphere test, then flip the normal so it points from the ground upwards
    if (!sphere.testCollisionWithGround(*this, outInfo))
    {
        return false;
    }

    outInfo.normal = outInfo.normal * -1.0;
    return true;
}

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
