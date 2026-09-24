/*
 * GroundCollider.h
 */

#pragma once

#include "Collider.h"

namespace BulletPhysics {
namespace collision {
namespace collider {

class BoxCollider;
class SphereCollider;

// infinite ground plane
class GroundCollider : public Collider {
public:
    explicit GroundCollider(double groundY = 0.0);

    CollisionShape getShape() const override { return CollisionShape::Ground; }

    // position
    const math::Vec3& getPosition() const override { return m_position; }
    void setPosition(const math::Vec3& pos) override;   // keeps height only
    bool isOrientable() const override { return false; }

    // ground y
    double getGroundY() const { return m_position.y; }
    void setGroundY(double level) { m_position.y = level; }

    // contact
    bool testCollision(const Collider& other, CollisionInfo& outInfo) const override;
    bool testCollisionWithBox(const BoxCollider& box, CollisionInfo& outInfo) const;
    bool testCollisionWithSphere(const SphereCollider& sphere, CollisionInfo& outInfo) const;

    // queries
    bool raycast(const Ray& ray, double& outDistance) const override;
    bool sweep(const Sweep& sweep, double& outDistance) const override;
    double thickness(const Ray& ray) const override;

    // shape
    double boundingRadius() const override { return 1e30; }   // plane has no bound
    math::Mat3 inverseInertia(double) const override { return math::Mat3::zero(); }   // never turns
    math::Vec3 normalAt(const math::Vec3& point) const override { return {0.0, 1.0, 0.0}; }

private:
    math::Vec3 m_position{};

    friend class BoxCollider;
    friend class SphereCollider;
};

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
