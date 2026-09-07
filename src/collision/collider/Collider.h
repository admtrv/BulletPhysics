/*
 * Collider.h
 */

#pragma once

#include "collision/PhysicsMaterial.h"
#include "collision/Query.h"
#include "math/Quat.h"
#include "math/Vec3.h"

#include <cstdint>

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

// pair collides only when both sides accept the other
using LayerMask = uint32_t;

inline constexpr LayerMask LAYER_DEFAULT = 1u << 0;
inline constexpr LayerMask LAYER_ALL = ~0u;

// face on face needs several points, single one lets the box rock
inline constexpr int MAX_CONTACT_POINTS = 4;

struct ContactPoint {
    math::Vec3 position{};
    int feature = 0;            // what produced it, matched across steps

    double normalImpulse = 0.0;
    double tangentImpulses[2]{};
    double targetSpeed = 0.0;   // separating speed to reach
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

    // position
    virtual const math::Vec3& getPosition() const = 0;
    virtual void setPosition(const math::Vec3& pos) = 0;

    // orientation
    virtual void setOrientation(const math::Quat& orientation) {}

    // contact
    virtual bool testCollision(const Collider& other, CollisionInfo& outInfo) const = 0;   // normal points from this collider to other

    // queries
    virtual bool raycast(const Ray& ray, double& outDistance) const = 0;
    virtual bool sweep(const Sweep& sweep, double& outDistance) const = 0;
    virtual double thickness(const Ray& ray) const = 0;

    // shape
    virtual double boundingRadius() const = 0;
    virtual math::Vec3 normalAt(const math::Vec3& point) const = 0;

    // surface
    const PhysicsMaterial& getMaterial() const { return m_material; }
    void setMaterial(const PhysicsMaterial& material) { m_material = material; }

    // trigger
    bool isTrigger() const { return m_trigger; }
    void setTrigger(bool trigger) { m_trigger = trigger; }

    // layers
    LayerMask getLayer() const { return m_layer; }
    void setLayer(LayerMask layer) { m_layer = layer; }

    LayerMask getMask() const { return m_mask; }
    void setMask(LayerMask mask) { m_mask = mask; }

    bool collidesWith(const Collider& other) const
    {
        return (m_mask & other.m_layer) != 0 && (other.m_mask & m_layer) != 0;
    }

    // body
    dynamics::RigidBody* getBody() const { return m_body; }
    void setBody(dynamics::RigidBody* body) { m_body = body; }

private:
    dynamics::RigidBody* m_body = nullptr;
    PhysicsMaterial m_material;

    bool m_trigger = false;

    LayerMask m_layer = LAYER_DEFAULT;
    LayerMask m_mask = LAYER_ALL;
};

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
