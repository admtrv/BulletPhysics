/*
 * Collider.h
 */

#pragma once

#include "collision/PhysicsMaterial.h"
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

// one point of the contact patch
struct ContactPoint {
    math::Vec3 position{};

    // what produced the point, matching it across steps carries the impulse over
    int feature = 0;

    // impulse held last step, friction runs along two fixed directions
    double normalImpulse = 0.0;
    double tangentImpulses[2]{};

    // separating speed to reach, measured before the passes eat the approach
    double targetSpeed = 0.0;
};

struct CollisionInfo {
    double penetration = 0.0;   // overlap depth along normal
    math::Vec3 normal{};        // unit vector, from first collider to second

    ContactPoint points[MAX_CONTACT_POINTS]{};
    int pointCount = 0;

    void addPoint(const math::Vec3& position, int feature)
    {
        if (pointCount < MAX_CONTACT_POINTS)
        {
            points[pointCount].position = position;
            points[pointCount].feature = feature;
            points[pointCount].normalImpulse = 0.0;
            points[pointCount].tangentImpulses[0] = 0.0;
            points[pointCount].tangentImpulses[1] = 0.0;
            points[pointCount].targetSpeed = 0.0;

            pointCount++;
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

    // surface
    const PhysicsMaterial& getMaterial() const { return m_material; }
    void setMaterial(const PhysicsMaterial& material) { m_material = material; }

    dynamics::RigidBody* getBody() const { return m_body; }
    void setBody(dynamics::RigidBody* body) { m_body = body; }

private:
    dynamics::RigidBody* m_body = nullptr;
    PhysicsMaterial m_material;
};

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
