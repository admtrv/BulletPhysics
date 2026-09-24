/*
 * Collider.cpp
 */

#include "Collider.h"

#include <algorithm>

namespace BulletPhysics {
namespace collision {
namespace collider {

// mobility

math::Vec3 Mobility::project(const math::Vec3& direction) const
{
    math::Vec3 allowed{};

    for (int axis = 0; axis < 3; axis++)
    {
        allowed[axis] = axes[axis] ? direction[axis] : 0.0;
    }

    return allowed;
}

math::Vec3 PairMobility::project(const math::Vec3& direction) const
{
    Mobility together;

    for (int axis = 0; axis < 3; axis++)
    {
        together.axes[axis] = relieves(axis);
    }

    return together.project(direction);
}

double PairMobility::costOf(const math::Vec3& direction, double penetration) const
{
    const double share = project(direction).length();

    return share > AXIS_EPSILON ? penetration / share : -1.0;
}

// contact

void CollisionInfo::setContact(const math::Vec3& normal, double penetration)
{
    this->normal = normal;
    this->penetration = penetration;

    take(normal, penetration);
}

void CollisionInfo::offerAxis(const math::Vec3& axis, double penetration, const math::Vec3& towards)
{
    const math::Vec3 facing = axis.dot(towards) < 0.0 ? axis * -1.0 : axis;

    if (penetration < this->penetration)
    {
        normal = facing;
        this->penetration = penetration;
    }

    take(facing, penetration);
}

void CollisionInfo::reverse()
{
    normal = normal * -1.0;
    correction = correction * -1.0;
}

void CollisionInfo::addPoint(const math::Vec3& position, int feature)
{
    if (pointCount >= MAX_CONTACT_POINTS)
    {
        return;
    }

    ContactPoint& point = points[pointCount];

    point.position = position;
    point.feature = feature;
    point.normalImpulse = 0.0;
    point.tangentImpulses[0] = 0.0;
    point.tangentImpulses[1] = 0.0;
    point.targetSpeed = 0.0;

    pointCount++;
}

// stretched until it clears whole overlap, since pair leaves along it at angle
void CollisionInfo::take(const math::Vec3& axis, double penetration)
{
    const double cost = mobility.costOf(axis, penetration);

    if (cost < 0.0 || cost >= separation)
    {
        return;
    }

    const math::Vec3 allowed = mobility.project(axis);

    correction = allowed * (cost / std::max(allowed.length(), AXIS_EPSILON));
    separation = cost;
}

// collider

void Collider::place(const math::Vec3& position, const math::Quat& rotation)
{
    setPosition(position + rotation.rotate(m_localPosition));
    setOrientation(rotation * m_localRotation);
}

bool Collider::collidesWith(const Collider& other) const
{
    return (m_mask & other.m_layer) != 0 && (other.m_mask & m_layer) != 0;
}

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
