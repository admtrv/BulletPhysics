/*
 * BoxCollider.cpp
 */

#include "BoxCollider.h"
#include "GroundCollider.h"

namespace BulletPhysics {
namespace builtin {
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
            return testCollisionWithBox(dynamic_cast<const BoxCollider&>(other), outInfo);
        }
        case CollisionShape::Ground: {
            return testCollisionWithGround(dynamic_cast<const GroundCollider&>(other), outInfo);
        }
        default:
            return false;
    }
}

bool BoxCollider::testPoint(const math::Vec3& point) const
{
    math::Vec3 half = m_size * 0.5;
    math::Vec3 diff = point - m_position;

    return std::abs(diff.x) <= half.x && std::abs(diff.y) <= half.y && std::abs(diff.z) <= half.z;
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
        outInfo.normal = math::Vec3{0.0, 1.0, 0.0};
        outInfo.penetration = groundY - lowestY;
        return true;
    }

    return false;
}

double BoxCollider::computeThickness(const math::Vec3& rayOrigin, const math::Vec3& rayDir) const
{
    math::Vec3 dir = rayDir.normalized();
    math::Vec3 halfSize = m_size * 0.5;

    // ray-AABB intersection (slab method)
    double tMin = -1e30;
    double tMax = 1e30;

    // x axis
    if (std::abs(dir.x) > 1e-9)
    {
        double t1 = (m_position.x - halfSize.x - rayOrigin.x) / dir.x;
        double t2 = (m_position.x + halfSize.x - rayOrigin.x) / dir.x;

        if (t1 > t2)
            std::swap(t1, t2);

        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
    }
    else
    {
        if (rayOrigin.x < m_position.x - halfSize.x || rayOrigin.x > m_position.x + halfSize.x)
            return 1e6;
    }

    // y axis
    if (std::abs(dir.y) > 1e-9)
    {
        double t1 = (m_position.y - halfSize.y - rayOrigin.y) / dir.y;
        double t2 = (m_position.y + halfSize.y - rayOrigin.y) / dir.y;

        if (t1 > t2)
            std::swap(t1, t2);

        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
    }
    else
    {
        if (rayOrigin.y < m_position.y - halfSize.y || rayOrigin.y > m_position.y + halfSize.y)
            return 1e6;
    }

    // z axis
    if (std::abs(dir.z) > 1e-9)
    {
        double t1 = (m_position.z - halfSize.z - rayOrigin.z) / dir.z;
        double t2 = (m_position.z + halfSize.z - rayOrigin.z) / dir.z;

        if (t1 > t2)
            std::swap(t1, t2);

        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
    }
    else
    {
        if (rayOrigin.z < m_position.z - halfSize.z || rayOrigin.z > m_position.z + halfSize.z)
            return 1e6;
    }

    if (tMax < tMin || tMax < 0.0)
    {
        return 1e6;
    }

    double entry = std::max(tMin, 0.0);
    return tMax - entry;
}

} // namespace collider
} // namespace collision
} // namespace builtin
} // namespace BulletPhysics
