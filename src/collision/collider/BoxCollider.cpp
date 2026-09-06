/*
 * BoxCollider.cpp
 */

#include "BoxCollider.h"
#include "GroundCollider.h"
#include "SphereCollider.h"

#include <algorithm>

namespace BulletPhysics {
namespace collision {
namespace collider {

static constexpr double CONTACT_MARGIN = 0.01;   // corners just above surface still count, else the box rocks

static constexpr int CORNERS_PER_BOX = 8;
static constexpr int FEATURE_CLAMPED = 16;       // clamped point, no corner behind it

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

bool BoxCollider::raycast(const Ray& ray, double& outDistance) const
{
    const math::Vec3 half = m_size * 0.5;
    const math::Vec3 toCentre = m_position - ray.origin;

    // slab method run in the box own axes, so a turned box needs no special case
    double entry = -1e30;
    double exit = 1e30;

    for (int i = 0; i < 3; i++)
    {
        const double extent = (i == 0) ? half.x : (i == 1) ? half.y : half.z;

        const double along = ray.direction.dot(m_axes[i]);
        const double centre = toCentre.dot(m_axes[i]);

        if (std::abs(along) < 1e-9)
        {
            // ray runs parallel to this pair of faces, it must start between them
            if (std::abs(centre) > extent)
            {
                return false;
            }

            continue;
        }

        double near = (centre - extent) / along;
        double far = (centre + extent) / along;

        if (near > far)
        {
            std::swap(near, far);
        }

        entry = std::max(entry, near);
        exit = std::min(exit, far);

        if (entry > exit)
        {
            return false;
        }
    }

    // a ray starting inside leaves through the far face
    const double distance = (entry >= 0.0) ? entry : exit;

    if (distance < 0.0 || distance > ray.maxDistance)
    {
        return false;
    }

    outDistance = distance;
    return true;
}

math::Vec3 BoxCollider::normalAt(const math::Vec3& point) const
{
    const math::Vec3 half = m_size * 0.5;
    const math::Vec3 local = point - m_position;

    // the face the point sits closest to owns the normal
    int nearest = 0;
    double smallestGap = 1e30;

    for (int i = 0; i < 3; i++)
    {
        const double extent = (i == 0) ? half.x : (i == 1) ? half.y : half.z;
        const double gap = extent - std::abs(local.dot(m_axes[i]));

        if (gap < smallestGap)
        {
            smallestGap = gap;
            nearest = i;
        }
    }

    return m_axes[nearest] * (local.dot(m_axes[nearest]) >= 0.0 ? 1.0 : -1.0);
}

// half width of the box measured along an arbitrary direction
static double projectedRadius(const math::Vec3& half, const math::Vec3* axes, const math::Vec3& direction)
{
    return std::abs(direction.dot(axes[0])) * half.x
         + std::abs(direction.dot(axes[1])) * half.y
         + std::abs(direction.dot(axes[2])) * half.z;
}

bool BoxCollider::testCollisionWithBox(const BoxCollider& other, CollisionInfo& outInfo) const
{
    const math::Vec3 half = m_size * 0.5;
    const math::Vec3 otherHalf = other.m_size * 0.5;

    const math::Vec3 diff = other.m_position - m_position;

    // separating axis theorem, a gap on any axis means no contact
    math::Vec3 axes[15];
    int axisCount = 0;

    // face normals of both boxes
    for (int i = 0; i < 3; i++)
    {
        axes[axisCount++] = m_axes[i];
        axes[axisCount++] = other.m_axes[i];
    }

    // and every pair of edge directions
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            // parallel edges give a degenerate axis, their face normals cover it
            const math::Vec3 axis = m_axes[i].cross(other.m_axes[j]).normalized();

            if (axis.length() > 0.5)
            {
                axes[axisCount++] = axis;
            }
        }
    }

    double leastOverlap = 1e30;
    math::Vec3 leastAxis{};

    for (int i = 0; i < axisCount; i++)
    {
        const math::Vec3& axis = axes[i];

        const double reach = projectedRadius(half, m_axes, axis) + projectedRadius(otherHalf, other.m_axes, axis);
        const double distance = std::abs(diff.dot(axis));

        if (distance > reach)
        {
            return false;
        }

        const double overlap = reach - distance;
        if (overlap < leastOverlap)
        {
            leastOverlap = overlap;
            leastAxis = axis;
        }
    }

    // point the axis from this box towards the other one
    outInfo.normal = (diff.dot(leastAxis) < 0.0) ? leastAxis * -1.0 : leastAxis;
    outInfo.penetration = leastOverlap;

    // face lying on a face gives four points, which is what holds a stack up
    outInfo.pointCount = 0;

    collectCorners(other, outInfo.normal, outInfo.penetration, outInfo, 0);

    // the other box holds the face when its corners point away from the contact
    if (outInfo.pointCount == 0)
    {
        other.collectCorners(*this, outInfo.normal * -1.0, outInfo.penetration, outInfo, CORNERS_PER_BOX);
    }

    // edge crossing an edge leaves no corner inside, fall back to the deepest one
    if (outInfo.pointCount == 0)
    {
        math::Vec3 deepest = other.m_position;

        for (int i = 0; i < 3; i++)
        {
            const double extent = (i == 0) ? otherHalf.x : (i == 1) ? otherHalf.y : otherHalf.z;
            const double side = (other.m_axes[i].dot(outInfo.normal) > 0.0) ? -extent : extent;

            deepest += other.m_axes[i] * side;
        }

        const math::Vec3 local = deepest - m_position;
        math::Vec3 point = m_position;

        for (int i = 0; i < 3; i++)
        {
            const double extent = (i == 0) ? half.x : (i == 1) ? half.y : half.z;

            point += m_axes[i] * std::clamp(local.dot(m_axes[i]), -extent, extent);
        }

        outInfo.addPoint(point, FEATURE_CLAMPED);
    }

    return true;
}

// corners of source near the contact plane, a deeper one belongs to the far side
void BoxCollider::collectCorners(const BoxCollider& source, const math::Vec3& normal, double penetration,
                                 CollisionInfo& outInfo, int featureBase) const
{
    const math::Vec3 half = m_size * 0.5;
    const math::Vec3 sourceHalf = source.m_size * 0.5;

    // how deep a corner may sit and still count as touching
    const double reach = penetration + CONTACT_MARGIN;

    for (int i = 0; i < CORNERS_PER_BOX; i++)
    {
        const double sx = (i & 1) ? sourceHalf.x : -sourceHalf.x;
        const double sy = (i & 2) ? sourceHalf.y : -sourceHalf.y;
        const double sz = (i & 4) ? sourceHalf.z : -sourceHalf.z;

        const math::Vec3 corner = source.m_position
                                + source.m_axes[0] * sx + source.m_axes[1] * sy + source.m_axes[2] * sz;

        const math::Vec3 local = corner - m_position;

        // distance past this box's surface along the contact normal
        const double depth = projectedRadius(half, m_axes, normal) - local.dot(normal);

        if (depth < -CONTACT_MARGIN || depth > reach)
        {
            continue;
        }

        // and within the face
        bool inside = true;

        for (int axis = 0; axis < 3 && inside; axis++)
        {
            const double extent = (axis == 0) ? half.x : (axis == 1) ? half.y : half.z;

            inside = std::abs(local.dot(m_axes[axis])) <= extent + CONTACT_MARGIN;
        }

        if (inside)
        {
            outInfo.addPoint(corner, featureBase + i);
        }
    }
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
            outInfo.addPoint({vertex.x, groundY, vertex.z}, i);
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
