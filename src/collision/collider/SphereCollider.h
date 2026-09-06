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

    const math::Vec3& getPosition() const override { return m_position; }
    void setPosition(const math::Vec3& pos) override { m_position = pos; }

    // a sphere collides the same whichever way it faces, kept for debug drawing
    const math::Quat& getOrientation() const { return m_orientation; }
    void setOrientation(const math::Quat& orientation) override { m_orientation = orientation; }

    double getRadius() const { return m_radius; }
    void setRadius(double radius);

    bool testCollision(const Collider& other, CollisionInfo& outInfo) const override;
    bool raycast(const Ray& ray, double& outDistance) const override;
    math::Vec3 normalAt(const math::Vec3& point) const override;
    bool testCollisionWithSphere(const SphereCollider& sphere, CollisionInfo& outInfo) const;
    bool testCollisionWithBox(const BoxCollider& box, CollisionInfo& outInfo) const;
    bool testCollisionWithGround(const GroundCollider& ground, CollisionInfo& outInfo) const;

private:
    math::Vec3 m_position{};
    math::Quat m_orientation{};
    double m_radius;
};

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
