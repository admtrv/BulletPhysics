/*
 * BoxCollider.h
 */

#pragma once

#include "Collider.h"
#include "GroundCollider.h"

#include <algorithm>
#include <cmath>

namespace BulletPhysics {
namespace collision {

class GroundCollider;

// Oriented Bounding Box (OBB) collider
class BoxCollider : public Collider {
public:
    explicit BoxCollider(const math::Vec3& size = math::Vec3{1.0f, 1.0f, 1.0f});

    CollisionShape getShape() const override { return CollisionShape::Box; }
    const math::Vec3& getPosition() const override { return m_position; }
    void setPosition(const math::Vec3& pos) override { m_position = pos; }

    const math::Vec3& getSize() const { return m_size; }
    void setSize(const math::Vec3& size) { m_size = size; }

    // orientation as 3x3 rotation matrix (column vectors: x, y, z axes)
    void setAxes(const math::Vec3& axisX, const math::Vec3& axisY, const math::Vec3& axisZ);
    const math::Vec3* getAxes() const { return m_axes; }

    bool testCollision(const Collider& other, CollisionInfo& outInfo) const override;
    bool testPoint(const math::Vec3& point) const override;
    float computeThickness(const math::Vec3& rayOrigin, const math::Vec3& rayDir) const override;

    bool testCollisionWithBox(const BoxCollider& box, CollisionInfo& outInfo) const;
    bool testCollisionWithGround(const GroundCollider& ground, CollisionInfo& outInfo) const;

private:
    math::Vec3 m_position{};
    math::Vec3 m_size;
    math::Vec3 m_axes[3] {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    friend class GroundCollider;
};

} // namespace collision
} // namespace BulletPhysics
