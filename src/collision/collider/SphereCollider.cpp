/*
 * SphereCollider.cpp
 */

#include "SphereCollider.h"

#include "BoxCollider.h"
#include "GroundCollider.h"

#include <algorithm>

namespace BulletPhysics {
namespace collision {
namespace collider {

SphereCollider::SphereCollider(double radius) : m_radius(radius > 0.0 ? radius : 0.5) {}

void SphereCollider::setRadius(double radius)
{
    m_radius = (radius > 0.0 ? radius : m_radius);
}

bool SphereCollider::testCollision(const Collider& other, CollisionInfo& outInfo) const
{
    switch (other.getShape())
    {
        case CollisionShape::Sphere:
            return testCollisionWithSphere(static_cast<const SphereCollider&>(other), outInfo);

        case CollisionShape::Box:
            return testCollisionWithBox(static_cast<const BoxCollider&>(other), outInfo);

        case CollisionShape::Ground:
            return testCollisionWithGround(static_cast<const GroundCollider&>(other), outInfo);

        default:
            return false;
    }
}

bool SphereCollider::testCollisionWithSphere(const SphereCollider& sphere, CollisionInfo& outInfo) const
{
    const math::Vec3 diff = sphere.m_position - m_position;

    const double distance = diff.length();
    const double contactDistance = m_radius + sphere.m_radius;

    if (distance >= contactDistance)
    {
        return false;
    }

    // concentric spheres have no meaningful direction, push along a fixed axis
    outInfo.normal = (distance > 1e-9) ? diff * (1.0 / distance) : math::Vec3{0.0, 1.0, 0.0};
    outInfo.penetration = contactDistance - distance;

    return true;
}

bool SphereCollider::testCollisionWithBox(const BoxCollider& box, CollisionInfo& outInfo) const
{
    const math::Vec3 half = box.getSize() * 0.5;
    const math::Vec3* axes = box.getAxes();

    const math::Vec3 toSphere = m_position - box.getPosition();

    // closest point on the box, found by clamping the centre in the box's own frame
    math::Vec3 closest = box.getPosition();
    double local[3];

    for (int i = 0; i < 3; i++)
    {
        const double extent = (i == 0) ? half.x : (i == 1) ? half.y : half.z;

        local[i] = std::clamp(toSphere.dot(axes[i]), -extent, extent);
        closest += axes[i] * local[i];
    }

    math::Vec3 diff = closest - m_position;
    double distance = diff.length();

    if (distance > m_radius)
    {
        return false;
    }

    if (distance > 1e-9)
    {
        outInfo.normal = diff * (1.0 / distance);
        outInfo.penetration = m_radius - distance;
        return true;
    }

    // centre sits inside the box, leave along the face it is closest to
    int shallowest = 0;
    double smallestGap = (half.x - std::abs(local[0]));

    for (int i = 1; i < 3; i++)
    {
        const double extent = (i == 1) ? half.y : half.z;
        const double gap = extent - std::abs(local[i]);

        if (gap < smallestGap)
        {
            smallestGap = gap;
            shallowest = i;
        }
    }

    const double sign = (local[shallowest] >= 0.0) ? 1.0 : -1.0;

    outInfo.normal = axes[shallowest] * sign;
    outInfo.penetration = m_radius + smallestGap;

    return true;
}

bool SphereCollider::testCollisionWithGround(const GroundCollider& ground, CollisionInfo& outInfo) const
{
    const double lowest = m_position.y - m_radius;
    const double groundY = ground.getGroundY();

    if (lowest >= groundY)
    {
        return false;
    }

    // the sphere is the first collider, so the normal points down into the ground
    outInfo.normal = math::Vec3{0.0, -1.0, 0.0};
    outInfo.penetration = groundY - lowest;

    return true;
}

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
