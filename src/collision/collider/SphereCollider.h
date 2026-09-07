/*
 * SphereCollider.h
 */

#pragma once

#include "Collider.h"

namespace BulletPhysics {
namespace collision {
namespace collider {

class BoxCollider;
class GroundCollider;

class SphereCollider : public Collider {
public:
    explicit SphereCollider(double radius = 0.5);

    CollisionShape getShape() const override { return CollisionShape::Sphere; }

    // podition
    const math::Vec3& getPosition() const override { return m_position; }
    void setPosition(const math::Vec3& pos) override { m_position = pos; }

    // orientation
    const math::Quat& getOrientation() const { return m_orientation; }
    void setOrientation(const math::Quat& orientation) override { m_orientation = orientation; }

    // radius
    double getRadius() const { return m_radius; }
    void setRadius(double radius);

    // contact
    bool testCollision(const Collider& other, CollisionInfo& outInfo) const override;
    bool testCollisionWithSphere(const SphereCollider& sphere, CollisionInfo& outInfo) const;
    bool testCollisionWithBox(const BoxCollider& box, CollisionInfo& outInfo) const;
    bool testCollisionWithGround(const GroundCollider& ground, CollisionInfo& outInfo) const;

    // queries
    bool raycast(const Ray& ray, double& outDistance) const override;
    bool sweep(const Sweep& sweep, double& outDistance) const override;
    double thickness(const Ray& ray) const override;

    // shape
    double boundingRadius() const override { return m_radius; }
    math::Vec3 normalAt(const math::Vec3& point) const override;

private:
    bool span(const math::Vec3& origin, const math::Vec3& direction, double margin, double& outEntry, double& outExit) const;

    math::Vec3 m_position{};
    math::Quat m_orientation{};
    double m_radius;
};

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
