/*
 * CylinderCollider.cpp
 */

#include "CylinderCollider.h"

#include "BoxCollider.h"
#include "GroundCollider.h"
#include "SphereCollider.h"

#include <algorithm>
#include <cmath>

namespace BulletPhysics {
namespace collision {
namespace collider {

constexpr double EPSILON = 1e-9;
constexpr int RIM_POINTS = 4;       // enough to keep a resting cap from rocking

CylinderCollider::CylinderCollider(double radius, double height)
    : m_radius(radius > 0.0 ? radius : 0.5), m_height(height > 0.0 ? height : 1.0) {}

void CylinderCollider::setRadius(double radius)
{
    m_radius = (radius > 0.0 ? radius : m_radius);
}

void CylinderCollider::setHeight(double height)
{
    m_height = (height > 0.0 ? height : m_height);
}

void CylinderCollider::setOrientation(const math::Quat& orientation)
{
    m_axis = orientation.rotate({0.0, 1.0, 0.0});
}

double CylinderCollider::boundingRadius() const
{
    const double half = m_height * 0.5;
    return std::sqrt(m_radius * m_radius + half * half);
}

// contact

bool CylinderCollider::testCollision(const Collider& other, CollisionInfo& outInfo) const
{
    switch (other.getShape())
    {
        case CollisionShape::Cylinder:
            return testCollisionWithCylinder(static_cast<const CylinderCollider&>(other), outInfo);

        case CollisionShape::Box:
            return testCollisionWithBox(static_cast<const BoxCollider&>(other), outInfo);

        case CollisionShape::Sphere:
            return testCollisionWithSphere(static_cast<const SphereCollider&>(other), outInfo);

        case CollisionShape::Ground:
            return testCollisionWithGround(static_cast<const GroundCollider&>(other), outInfo);

        default:
            return false;
    }
}

bool CylinderCollider::testCollisionWithSphere(const SphereCollider& sphere, CollisionInfo& outInfo) const
{
    const math::Vec3 centre = sphere.getPosition();
    const math::Vec3 closest = closestPoint(centre);

    math::Vec3 diff = centre - closest;
    double distance = diff.length();

    // centre inside the solid, push it out the cheapest way
    if (distance < EPSILON)
    {
        const math::Vec3 local = centre - m_position;

        const double along = local.dot(m_axis);
        const math::Vec3 radial = local - m_axis * along;
        const double radialLength = radial.length();

        const double toCap = m_height * 0.5 - std::abs(along);
        const double toSide = m_radius - radialLength;

        if (toCap < toSide)
        {
            outInfo.normal = m_axis * (along >= 0.0 ? 1.0 : -1.0);
            outInfo.penetration = toCap + sphere.getRadius();
        }
        else
        {
            outInfo.normal = radialLength > EPSILON ? radial * (1.0 / radialLength) : math::Vec3{1.0, 0.0, 0.0};
            outInfo.penetration = toSide + sphere.getRadius();
        }

        outInfo.pointCount = 0;
        outInfo.addPoint(centre, 0);

        return true;
    }

    if (distance >= sphere.getRadius())
    {
        return false;
    }

    outInfo.normal = diff * (1.0 / distance);
    outInfo.penetration = sphere.getRadius() - distance;

    outInfo.pointCount = 0;
    outInfo.addPoint(closest, 0);

    return true;
}

bool CylinderCollider::testCollisionWithGround(const GroundCollider& ground, CollisionInfo& outInfo) const
{
    static const math::Vec3 UP{0.0, 1.0, 0.0};

    const double groundY = ground.getGroundY();
    const double half = m_height * 0.5;

    // lowest reach is the cap centre dropped by however much the rim tilts below it
    const double tilt = m_axis.dot(UP);
    const math::Vec3 radial = UP - m_axis * tilt;
    const double spread = radial.length() * m_radius;

    const double lowest = m_position.y - std::abs(tilt) * half - spread;

    if (lowest >= groundY)
    {
        return false;
    }

    // cylinder is first collider, normal points down into ground
    outInfo.normal = math::Vec3{0.0, -1.0, 0.0};
    outInfo.penetration = groundY - lowest;
    outInfo.pointCount = 0;

    const double side = tilt >= 0.0 ? -1.0 : 1.0;
    const math::Vec3 capCentre = m_position + m_axis * (side * half);

    // a cap lying flat rests on its whole rim, anything else touches at one spot
    if (std::abs(tilt) > 1.0 - EPSILON)
    {
        math::Vec3 first = m_axis.cross({1.0, 0.0, 0.0});

        if (first.length() < EPSILON)
        {
            first = m_axis.cross({0.0, 0.0, 1.0});
        }

        first = first.normalized();
        const math::Vec3 second = m_axis.cross(first);

        for (int i = 0; i < RIM_POINTS; i++)
        {
            const double angle = 2.0 * M_PI * i / RIM_POINTS;
            const math::Vec3 offset = (first * std::cos(angle) + second * std::sin(angle)) * m_radius;

            outInfo.addPoint({capCentre.x + offset.x, groundY, capCentre.z + offset.z}, i);
        }

        return true;
    }

    const math::Vec3 lowestPoint = capCentre - radial.normalized() * m_radius;
    outInfo.addPoint({lowestPoint.x, groundY, lowestPoint.z}, 0);

    return true;
}

bool CylinderCollider::testCollisionWithBox(const BoxCollider& box, CollisionInfo& outInfo) const
{
    const math::Vec3 half = box.getSize() * 0.5;
    const math::Vec3* axes = box.getAxes();

    // box face normals and the cylinder axis, plus where the two sets meet edge on
    math::Vec3 candidates[8];
    int count = 0;

    for (int i = 0; i < 3; i++)
    {
        candidates[count++] = axes[i];
    }

    candidates[count++] = m_axis;

    for (int i = 0; i < 3; i++)
    {
        const math::Vec3 cross = m_axis.cross(axes[i]);

        if (cross.length() > EPSILON)
        {
            candidates[count++] = cross.normalized();
        }
    }

    const math::Vec3 diff = box.getPosition() - m_position;

    double leastOverlap = 1e30;
    math::Vec3 leastAxis{};

    for (int i = 0; i < count; i++)
    {
        const math::Vec3& axis = candidates[i];

        const double boxReach = std::abs(axis.dot(axes[0])) * half.x
                              + std::abs(axis.dot(axes[1])) * half.y
                              + std::abs(axis.dot(axes[2])) * half.z;

        // cylinder spans its half height along the axis and its radius across
        const double along = axis.dot(m_axis);
        const double ownReach = std::abs(along) * m_height * 0.5 + std::sqrt(std::max(0.0, 1.0 - along * along)) * m_radius;

        const double gap = std::abs(diff.dot(axis)) - (boxReach + ownReach);

        if (gap >= 0.0)
        {
            return false;
        }

        if (-gap < leastOverlap && outInfo.allows(axis))
        {
            leastOverlap = -gap;
            leastAxis = diff.dot(axis) >= 0.0 ? axis : axis * -1.0;
        }
    }

    // every way out was frozen, so there is nothing to answer with
    if (leastOverlap > 1e29)
    {
        return false;
    }

    outInfo.normal = leastAxis;
    outInfo.penetration = leastOverlap;

    outInfo.pointCount = 0;
    outInfo.addPoint(closestPoint(box.getPosition()), 0);

    return true;
}

bool CylinderCollider::testCollisionWithCylinder(const CylinderCollider& cylinder, CollisionInfo& outInfo) const
{
    const math::Vec3 diff = cylinder.m_position - m_position;

    math::Vec3 candidates[3];
    int count = 0;

    candidates[count++] = m_axis;
    candidates[count++] = cylinder.m_axis;

    const math::Vec3 cross = m_axis.cross(cylinder.m_axis);

    if (cross.length() > EPSILON)
    {
        candidates[count++] = cross.normalized();
    }
    else if (diff.length() > EPSILON)
    {
        // axes are parallel, the line between centres tells them apart sideways
        const math::Vec3 radial = diff - m_axis * diff.dot(m_axis);

        if (radial.length() > EPSILON)
        {
            candidates[count++] = radial.normalized();
        }
    }

    double leastOverlap = 1e30;
    math::Vec3 leastAxis{};

    for (int i = 0; i < count; i++)
    {
        const math::Vec3& axis = candidates[i];

        const double ownAlong = axis.dot(m_axis);
        const double ownReach = std::abs(ownAlong) * m_height * 0.5 + std::sqrt(std::max(0.0, 1.0 - ownAlong * ownAlong)) * m_radius;

        const double otherAlong = axis.dot(cylinder.m_axis);
        const double otherReach = std::abs(otherAlong) * cylinder.m_height * 0.5 + std::sqrt(std::max(0.0, 1.0 - otherAlong * otherAlong)) * cylinder.m_radius;

        const double gap = std::abs(diff.dot(axis)) - (ownReach + otherReach);

        if (gap >= 0.0)
        {
            return false;
        }

        if (-gap < leastOverlap && outInfo.allows(axis))
        {
            leastOverlap = -gap;
            leastAxis = diff.dot(axis) >= 0.0 ? axis : axis * -1.0;
        }
    }

    if (leastOverlap > 1e29)
    {
        return false;
    }

    outInfo.normal = leastAxis;
    outInfo.penetration = leastOverlap;

    outInfo.pointCount = 0;
    outInfo.addPoint(closestPoint(cylinder.m_position), 0);

    return true;
}

// queries

bool CylinderCollider::raycast(const Ray& ray, double& outDistance) const
{
    double entry = 0.0;
    double exit = 0.0;

    if (!span(ray.origin, ray.direction, m_radius, m_height * 0.5, entry, exit))
    {
        return false;
    }

    const double hit = entry >= 0.0 ? entry : exit;

    if (hit < 0.0 || hit > ray.maxDistance)
    {
        return false;
    }

    outDistance = hit;
    return true;
}

bool CylinderCollider::sweep(const Sweep& sweep, double& outDistance) const
{
    double entry = 0.0;
    double exit = 0.0;

    // a swept sphere becomes a point once the solid grows by its radius
    if (!span(sweep.origin, sweep.direction, m_radius + sweep.radius, m_height * 0.5 + sweep.radius, entry, exit))
    {
        return false;
    }

    if (entry > sweep.distance || exit < 0.0)
    {
        return false;
    }

    outDistance = std::max(0.0, entry);
    return true;
}

double CylinderCollider::thickness(const Ray& ray) const
{
    double entry = 0.0;
    double exit = 0.0;

    if (!span(ray.origin, ray.direction, m_radius, m_height * 0.5, entry, exit))
    {
        return 0.0;
    }

    return std::max(0.0, exit - std::max(0.0, entry));
}

// shape

math::Vec3 CylinderCollider::normalAt(const math::Vec3& point) const
{
    const math::Vec3 local = point - m_position;

    const double along = local.dot(m_axis);
    const math::Vec3 radial = local - m_axis * along;
    const double radialLength = radial.length();

    // a point past the cap belongs to it, one beside the body belongs to the side
    if (std::abs(along) >= m_height * 0.5 - EPSILON)
    {
        return m_axis * (along >= 0.0 ? 1.0 : -1.0);
    }

    return radialLength > EPSILON ? radial * (1.0 / radialLength) : m_axis;
}

math::Vec3 CylinderCollider::closestPoint(const math::Vec3& point) const
{
    const math::Vec3 local = point - m_position;

    const double half = m_height * 0.5;
    const double along = std::clamp(local.dot(m_axis), -half, half);

    const math::Vec3 radial = local - m_axis * local.dot(m_axis);
    const double radialLength = radial.length();

    math::Vec3 offset{};

    if (radialLength > EPSILON)
    {
        offset = radial * (std::min(radialLength, m_radius) / radialLength);
    }

    return m_position + m_axis * along + offset;
}

// helpers

// where a line enters and leaves the solid, caps and side each cut it in turn
bool CylinderCollider::span(const math::Vec3& origin, const math::Vec3& direction, double radius, double halfHeight, double& outEntry, double& outExit) const
{
    const math::Vec3 local = origin - m_position;

    double entry = -1e30;
    double exit = 1e30;

    // caps, a pair of parallel planes across the axis
    const double alongDir = direction.dot(m_axis);
    const double alongOrigin = local.dot(m_axis);

    if (std::abs(alongDir) < EPSILON)
    {
        if (std::abs(alongOrigin) > halfHeight)
        {
            return false;
        }
    }
    else
    {
        double near = (-halfHeight - alongOrigin) / alongDir;
        double far = (halfHeight - alongOrigin) / alongDir;

        if (near > far)
        {
            std::swap(near, far);
        }

        entry = std::max(entry, near);
        exit = std::min(exit, far);
    }

    // side, a circle once both are flattened onto the plane across the axis
    const math::Vec3 flatDir = direction - m_axis * alongDir;
    const math::Vec3 flatOrigin = local - m_axis * alongOrigin;

    const double a = flatDir.dot(flatDir);
    const double b = 2.0 * flatOrigin.dot(flatDir);
    const double c = flatOrigin.dot(flatOrigin) - radius * radius;

    if (a < EPSILON)
    {
        if (c > 0.0)
        {
            return false;
        }
    }
    else
    {
        const double discriminant = b * b - 4.0 * a * c;

        if (discriminant < 0.0)
        {
            return false;
        }

        const double root = std::sqrt(discriminant);

        entry = std::max(entry, (-b - root) / (2.0 * a));
        exit = std::min(exit, (-b + root) / (2.0 * a));
    }

    if (entry > exit || exit < 0.0)
    {
        return false;
    }

    outEntry = entry;
    outExit = exit;

    return true;
}

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
