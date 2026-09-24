/*
 * Collider.h
 */

#pragma once

#include "collision/PhysicsMaterial.h"
#include "collision/Query.h"
#include "math/Mat3.h"
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
    Cylinder,
    Ground,
};

// layers, pair collides only when both sides accept other
using LayerMask = uint32_t;

inline constexpr LayerMask LAYER_DEFAULT = 1u << 0;
inline constexpr LayerMask LAYER_ALL = ~0u;

inline constexpr int MAX_CONTACT_POINTS = 4;    // single point lets box rock on its face
inline constexpr double THICKNESS_2D = 0.01;    // zero lets fast bodies pass through flat shape

inline constexpr double AXIS_EPSILON = 1e-6;
inline constexpr double UNREACHABLE = 1e30;     // stands in until way out found

// mobility

// axes body still moves along
struct Mobility {
    bool axes[3] = {true, true, true};

    math::Vec3 project(const math::Vec3& direction) const;
};

// way out costs what pair travels along it, not depth of overlap there
struct PairMobility {
    Mobility first;
    Mobility second;

    bool relieves(int axis) const { return first.axes[axis] || second.axes[axis]; }

    math::Vec3 project(const math::Vec3& direction) const;
    double costOf(const math::Vec3& direction, double penetration) const;   // negative when pair cannot clear it at all
};

// contact

struct ContactPoint {
    math::Vec3 position{};
    int feature = 0;            // what produced it, matched across steps

    double normalImpulse = 0.0;
    double tangentImpulses[2]{};
    double targetSpeed = 0.0;
};

struct CollisionInfo {
    // overlap
    double penetration = UNREACHABLE;
    math::Vec3 normal{};        // from first collider to second

    // way apart, normal cut back to what pair can travel
    math::Vec3 correction{};
    double separation = UNREACHABLE;

    ContactPoint points[MAX_CONTACT_POINTS]{};
    int pointCount = 0;

    PairMobility mobility;      // filled in before test

    bool correctable() const { return separation < UNREACHABLE; }

    // filled by test, contact takes shallowest overlap, way out need not match it

    void setContact(const math::Vec3& normal, double penetration);
    void offerAxis(const math::Vec3& axis, double penetration, const math::Vec3& towards);
    void reverse();

    void addPoint(const math::Vec3& position, int feature);

private:
    void take(const math::Vec3& axis, double penetration);
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
    virtual bool isOrientable() const { return true; }   // shape with no facing ignores rotation

    // pose relative to carrier
    const math::Vec3& getLocalPosition() const { return m_localPosition; }
    void setLocalPosition(const math::Vec3& position) { m_localPosition = position; }

    const math::Quat& getLocalRotation() const { return m_localRotation; }
    void setLocalRotation(const math::Quat& rotation) { m_localRotation = rotation; }

    void place(const math::Vec3& position, const math::Quat& rotation);

    // contact
    virtual bool testCollision(const Collider& other, CollisionInfo& outInfo) const = 0;   // normal points from this collider to other

    // queries
    virtual bool raycast(const Ray& ray, double& outDistance) const = 0;
    virtual bool sweep(const Sweep& sweep, double& outDistance) const = 0;
    virtual double thickness(const Ray& ray) const = 0;

    // shape
    virtual double boundingRadius() const = 0;
    virtual double thinnestExtent() const { return boundingRadius(); }   // what step skips over
    virtual math::Vec3 normalAt(const math::Vec3& point) const = 0;
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

    bool collidesWith(const Collider& other) const;

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
