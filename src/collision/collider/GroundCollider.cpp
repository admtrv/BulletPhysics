/*
 * GroundCollider.cpp
 */

#include "GroundCollider.h"
#include "BoxCollider.h"
#include "SphereCollider.h"

#include <cmath>

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
            return false;
        }
        default:
            return false;
    }
}

// contact

bool GroundCollider::testCollisionWithBox(const BoxCollider& box, CollisionInfo& outInfo) const
{
    // box does the test, flip normal to point up from ground
    if (!box.testCollisionWithGround(*this, outInfo))
    {
        return false;
    }

    outInfo.normal = outInfo.normal * -1.0;
    return true;
}

bool GroundCollider::testCollisionWithSphere(const SphereCollider& sphere, CollisionInfo& outInfo) const
{
    // sphere does the test, flip normal to point up from ground
    if (!sphere.testCollisionWithGround(*this, outInfo))
    {
        return false;
    }

    outInfo.normal = outInfo.normal * -1.0;
    return true;
}

// queries

bool GroundCollider::raycast(const Ray& ray, double& outDistance) const
{
    // parallel rays never reach the plane
    if (std::abs(ray.direction.y) < 1e-9)
    {
        return false;
    }

    const double distance = (m_position.y - ray.origin.y) / ray.direction.y;

    if (distance < 0.0 || distance > ray.maxDistance)
    {
        return false;
    }

    outDistance = distance;
    return true;
}

bool GroundCollider::sweep(const Sweep& sweep, double& outDistance) const
{
    // a sphere touches the plane once its centre is one radius above it
    const double height = sweep.origin.y - (m_position.y + sweep.radius);

    // already touching at the start, that is for the usual test to resolve
    if (height <= 0.0)
    {
        return false;
    }

    // moving level or away never brings it down
    if (sweep.direction.y > -1e-9)
    {
        return false;
    }

    const double distance = height / -sweep.direction.y;

    if (distance > sweep.distance)
    {
        return false;
    }

    outDistance = distance;
    return true;
}

double GroundCollider::thickness(const Ray& ray) const
{
    // solid all the way down, nothing gets through
    return 1e30;
}

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
