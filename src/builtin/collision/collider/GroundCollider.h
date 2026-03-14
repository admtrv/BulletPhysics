/*
 * GroundCollider.h
 */

#pragma once

#include "Collider.h"

namespace BulletPhysics {
namespace builtin {
namespace collision {
namespace collider {

class BoxCollider;

// represents an infinite ground plane (similar to WorldBoundaryShape2D in Godot)
class GroundCollider : public Collider {
public:
    explicit GroundCollider(double groundY = 0.0);

    CollisionShape getShape() const override { return CollisionShape::Ground; }
    const math::Vec3& getPosition() const override { return m_position; }
    void setPosition(const math::Vec3& pos) override;

    double getGroundY() const { return m_position.y; }
    void setGroundY(double level);

    bool testCollision(const Collider& other, CollisionInfo& outInfo) const override;
    bool testPoint(const math::Vec3& point) const override;

    bool testCollisionWithBox(const BoxCollider& box, CollisionInfo& outInfo) const;

private:
    math::Vec3 m_position{};

    friend class BoxCollider;
};

} // namespace collider
} // namespace collision
} // namespace builtin
} // namespace BulletPhysics
