/*
 * Collider.h
 */

#pragma once

#include "collision/PhysicsMaterial.h"
#include "collision/Query.h"
#include "math/Mat3.h"
#include "math/Quat.h"
#include "math/Vec3.h"

#include <cmath>
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
    Cylinder,
    Ground,
};

// pair collides only when both sides accept the other
using LayerMask = uint32_t;

inline constexpr LayerMask LAYER_DEFAULT = 1u << 0;
inline constexpr LayerMask LAYER_ALL = ~0u;

// face on face needs several points, single one lets the box rock
inline constexpr int MAX_CONTACT_POINTS = 4;

// depth a flat shape is given, zero would let fast bodies pass through it
inline constexpr double THICKNESS_2D = 0.01;

constexpr double AXIS_EPSILON = 1e-6;       // a direction carrying less than this does not count

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

    bool freedom[3] = {true, true, true};       // where the pair can part, filled in before the test

    bool allows(const math::Vec3& axis) const
    {
        return (freedom[0] || std::abs(axis.x) < AXIS_EPSILON)
            && (freedom[1] || std::abs(axis.y) < AXIS_EPSILON)
            && (freedom[2] || std::abs(axis.z) < AXIS_EPSILON);
    }

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
    virtual bool isOrientable() const { return true; }   // a shape with no facing ignores rotation

    // pose relative to whatever carries it, so a shape sits off centre or turned
    const math::Vec3& getLocalPosition() const { return m_localPosition; }
    void setLocalPosition(const math::Vec3& position) { m_localPosition = position; }

    const math::Quat& getLocalRotation() const { return m_localRotation; }
    void setLocalRotation(const math::Quat& rotation) { m_localRotation = rotation; }

    // the carrier moved, shape follows it keeping its own offset
    void place(const math::Vec3& position, const math::Quat& rotation)
    {
        setPosition(position + rotation.rotate(m_localPosition));
        setOrientation(rotation * m_localRotation);
    }

    // contact
    virtual bool testCollision(const Collider& other, CollisionInfo& outInfo) const = 0;   // normal points from this collider to other

    // queries
    virtual bool raycast(const Ray& ray, double& outDistance) const = 0;
    virtual bool sweep(const Sweep& sweep, double& outDistance) const = 0;
    virtual double thickness(const Ray& ray) const = 0;

    // shape
    virtual double boundingRadius() const = 0;
    virtual double thinnestExtent() const { return boundingRadius(); }   // narrowest way through, what a step may skip over
    virtual math::Vec3 normalAt(const math::Vec3& point) const = 0;

    // how hard the shape is to spin, zero for one that never turns
    virtual math::Mat3 inverseInertia(double mass) const = 0;

    // surface
    const PhysicsMaterial& getMaterial() const { return m_material; }
    PhysicsMaterial& getMaterial() { return m_material; }
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

    math::Vec3 m_localPosition{};
    math::Quat m_localRotation{};

    bool m_trigger = false;

    LayerMask m_layer = LAYER_DEFAULT;
    LayerMask m_mask = LAYER_ALL;
};

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
