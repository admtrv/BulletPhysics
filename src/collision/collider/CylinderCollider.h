/*
 * CylinderCollider.h
 */

#pragma once

#include "Collider.h"

#include "dynamics/body/Inertia.h"

#include <algorithm>

namespace BulletPhysics {
namespace collision {
namespace collider {

class BoxCollider;
class SphereCollider;
class GroundCollider;

// round shape with flat caps, standing along its local y
class CylinderCollider : public Collider {
public:
    explicit CylinderCollider(double radius = 0.5, double height = 1.0);

    CollisionShape getShape() const override { return CollisionShape::Cylinder; }

    // position
    const math::Vec3& getPosition() const override { return m_position; }
    void setPosition(const math::Vec3& pos) override { m_position = pos; }

    // orientation
    void setOrientation(const math::Quat& orientation) override;
    const math::Vec3& getAxis() const { return m_axis; }        // which way it stands, once turned

    // size
    double getRadius() const { return m_radius; }
    void setRadius(double radius);

    double getHeight() const { return m_height; }
    void setHeight(double height);

    // contact
    bool testCollision(const Collider& other, CollisionInfo& outInfo) const override;
    bool testCollisionWithCylinder(const CylinderCollider& cylinder, CollisionInfo& outInfo) const;
    bool testCollisionWithBox(const BoxCollider& box, CollisionInfo& outInfo) const;
    bool testCollisionWithSphere(const SphereCollider& sphere, CollisionInfo& outInfo) const;
    bool testCollisionWithGround(const GroundCollider& ground, CollisionInfo& outInfo) const;

    // queries
    bool raycast(const Ray& ray, double& outDistance) const override;
    bool sweep(const Sweep& sweep, double& outDistance) const override;
    double thickness(const Ray& ray) const override;

    // shape
    double boundingRadius() const override;
    double thinnestExtent() const override { return std::min(m_radius, m_height * 0.5); }
    math::Mat3 inverseInertia(double mass) const override { return dynamics::inertia::cylinder(mass, m_radius, m_height); }
    math::Vec3 normalAt(const math::Vec3& point) const override;

    // closest point of solid to point outside it
    math::Vec3 closestPoint(const math::Vec3& point) const;

private:
    bool span(const math::Vec3& origin, const math::Vec3& direction, double radius, double halfHeight, double& outEntry, double& outExit) const;

    math::Vec3 m_position{};
    math::Vec3 m_axis{0.0, 1.0, 0.0};
    double m_radius;
    double m_height;
};

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
