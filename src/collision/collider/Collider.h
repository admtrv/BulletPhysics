/*
 * Collider.h
 */

#pragma once

#include "math/Quat.h"
#include "math/Vec3.h"

namespace BulletPhysics {
namespace dynamics {

class RigidBody;
}

namespace collision {
namespace collider {

enum class CollisionShape {
    Box,
    Sphere,
    Ground,
};

// face on face needs several points, single one lets the box rock
inline constexpr int MAX_CONTACT_POINTS = 4;

struct CollisionInfo {
    double penetration = 0.0;   // overlap depth along normal
    math::Vec3 normal{};        // unit vector, from first collider to second

    math::Vec3 points[MAX_CONTACT_POINTS]{};   // world contact points
    int pointCount = 0;

    void addPoint(const math::Vec3& point)
    {
        if (pointCount < MAX_CONTACT_POINTS)
        {
            points[pointCount++] = point;
        }
    }
};

class Collider {
public:
    virtual ~Collider() = default;

    virtual CollisionShape getShape() const = 0;

    virtual const math::Vec3& getPosition() const = 0;
    virtual void setPosition(const math::Vec3& pos) = 0;

    // shapes looking same from every side ignore it
    virtual void setOrientation(const math::Quat& orientation) {}

    // outInfo normal points from this collider to other
    virtual bool testCollision(const Collider& other, CollisionInfo& outInfo) const = 0;

    dynamics::RigidBody* getBody() const { return m_body; }
    void setBody(dynamics::RigidBody* body) { m_body = body; }

private:
    dynamics::RigidBody* m_body = nullptr;
};

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
