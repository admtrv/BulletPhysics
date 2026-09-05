/*
 * BoxCollider.cpp
 */

#include "BoxCollider.h"
#include "GroundCollider.h"
#include "SphereCollider.h"

namespace BulletPhysics {
namespace collision {
namespace collider {

// corners slightly above surface still count, otherwise the box rocks
static constexpr double CONTACT_MARGIN = 0.01;

BoxCollider::BoxCollider(const math::Vec3& size) : m_size(size) {}

void BoxCollider::setAxes(const math::Vec3& axisX, const math::Vec3& axisY, const math::Vec3& axisZ)
{
    m_axes[0] = axisX;
    m_axes[1] = axisY;
    m_axes[2] = axisZ;
}

void BoxCollider::setOrientation(const math::Quat& orientation)
{
    m_axes[0] = orientation.rotate({1.0, 0.0, 0.0});
    m_axes[1] = orientation.rotate({0.0, 1.0, 0.0});
    m_axes[2] = orientation.rotate({0.0, 0.0, 1.0});
}

bool BoxCollider::testCollision(const Collider& other, CollisionInfo& outInfo) const
{
    switch (other.getShape()) {
        case CollisionShape::Box: {
            return testCollisionWithBox(static_cast<const BoxCollider&>(other), outInfo);
        }
        case CollisionShape::Sphere: {
            // sphere does the test, flip normal to point away from us
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

    outInfo.pointCount = 0;
    outInfo.addPoint({
        std::clamp(other.m_position.x, m_position.x - half1.x, m_position.x + half1.x),
        std::clamp(other.m_position.y, m_position.y - half1.y, m_position.y + half1.y),
        std::clamp(other.m_position.z, m_position.z - half1.z, m_position.z + half1.z)
    });

    return true;
}

bool BoxCollider::testCollisionWithGround(const GroundCollider& ground, CollisionInfo& outInfo) const
{
    double groundY = ground.getGroundY();
    math::Vec3 half = m_size * 0.5;

    double lowestY = m_position.y;

    // every corner under the plane, flat face gives four and stops rocking
    outInfo.pointCount = 0;

    for (int i = 0; i < 8; i++)
    {
        double sx = (i & 1) ? half.x : -half.x;
        double sy = (i & 2) ? half.y : -half.y;
        double sz = (i & 4) ? half.z : -half.z;

        const math::Vec3 vertex = m_position + m_axes[0] * sx + m_axes[1] * sy + m_axes[2] * sz;

        if (vertex.y < lowestY)
        {
            lowestY = vertex.y;
        }

        if (vertex.y < groundY + CONTACT_MARGIN)
        {
            outInfo.addPoint({vertex.x, groundY, vertex.z});
        }
    }

    if (outInfo.pointCount > 0)
    {
        // box is first collider, normal points down into ground
        outInfo.normal = math::Vec3{0.0, -1.0, 0.0};
        outInfo.penetration = groundY - lowestY;

        return true;
    }

    return false;
}

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
