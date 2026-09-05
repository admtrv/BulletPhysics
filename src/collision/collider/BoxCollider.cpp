/*
 * BoxCollider.cpp
 */

#include "BoxCollider.h"
#include "GroundCollider.h"
#include "SphereCollider.h"

namespace BulletPhysics {
namespace collision {
namespace collider {

BoxCollider::BoxCollider(const math::Vec3& size) : m_size(size) {}

void BoxCollider::setAxes(const math::Vec3& axisX, const math::Vec3& axisY, const math::Vec3& axisZ)
{
    m_axes[0] = axisX;
    m_axes[1] = axisY;
    m_axes[2] = axisZ;
}

bool BoxCollider::testCollision(const Collider& other, CollisionInfo& outInfo) const
{
    switch (other.getShape()) {
        case CollisionShape::Box: {
            return testCollisionWithBox(static_cast<const BoxCollider&>(other), outInfo);
        }
        case CollisionShape::Sphere: {
            // reuse the sphere's own test, then flip the normal to point away from us
            if (!static_cast<const SphereCollider&>(other).testCollisionWithBox(*this, outInfo))
            {
                return false;
            }

            outInfo.normal = outInfo.normal * -1.0;
            return true;
        }
        case CollisionShape::Ground: {
            return testCollisionWithGround(static_cast<const GroundCollider&>(other), outInfo);
        }
        default:
            return false;
    }
}

bool BoxCollider::testCollisionWithBox(const BoxCollider& other, CollisionInfo& outInfo) const
{
    math::Vec3 half1 = m_size * 0.5;
    math::Vec3 half2 = other.m_size * 0.5;

    math::Vec3 diff = other.m_position - m_position;

    if (std::abs(diff.x) > half1.x + half2.x || std::abs(diff.y) > half1.y + half2.y || std::abs(diff.z) > half1.z + half2.z)
    {
        return false;
    }

    // find axis of minimum penetration
    double minPenetration = std::abs(diff.x) - (half1.x + half2.x);
    outInfo.normal = diff.x > 0.0 ? math::Vec3{1.0, 0.0, 0.0} : math::Vec3{-1.0, 0.0, 0.0};
    outInfo.penetration = -minPenetration;

    double penY = std::abs(diff.y) - (half1.y + half2.y);
    if (-penY < outInfo.penetration)
    {
        outInfo.penetration = -penY;
        outInfo.normal = diff.y > 0.0 ? math::Vec3{0.0, 1.0, 0.0} : math::Vec3{0.0, -1.0, 0.0};
    }

    double penZ = std::abs(diff.z) - (half1.z + half2.z);
    if (-penZ < outInfo.penetration)
    {
        outInfo.penetration = -penZ;
        outInfo.normal = diff.z > 0.0 ? math::Vec3{0.0, 0.0, 1.0} : math::Vec3{0.0, 0.0, -1.0};
    }

    return true;
}

bool BoxCollider::testCollisionWithGround(const GroundCollider& ground, CollisionInfo& outInfo) const
{
    double groundY = ground.getGroundY();
    math::Vec3 half = m_size * 0.5;

    // find lowest vertex
    double lowestY = m_position.y;

    // project half-extents onto each axis and find y-components
    for (int i = 0; i < 8; i++)
    {
        // generate all 8 corner combinations
        double sx = (i & 1) ? half.x : -half.x;
        double sy = (i & 2) ? half.y : -half.y;
        double sz = (i & 4) ? half.z : -half.z;

        // vertex = center + sx*axisX + sy*axisY + sz*axisZ
        double vertexY = m_position.y + sx * m_axes[0].y + sy * m_axes[1].y + sz * m_axes[2].y;

        if (vertexY < lowestY)
        {
            lowestY = vertexY;
        }
    }

    if (lowestY < groundY)
    {
        // the box is the first collider, so the normal points down into the ground
        outInfo.normal = math::Vec3{0.0, -1.0, 0.0};
        outInfo.penetration = groundY - lowestY;
        return true;
    }

    return false;
}

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
