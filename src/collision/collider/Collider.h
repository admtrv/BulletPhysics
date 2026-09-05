/*
 * Collider.h
 */

#pragma once

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

struct CollisionInfo {
    double penetration = 0.0;   // overlap depth along normal
    math::Vec3 normal{};        // unit vector, points from first collider to second
};

class Collider {
public:
    virtual ~Collider() = default;

    virtual CollisionShape getShape() const = 0;

    virtual const math::Vec3& getPosition() const = 0;
    virtual void setPosition(const math::Vec3& pos) = 0;

    // outInfo normal points from this collider towards other
    virtual bool testCollision(const Collider& other, CollisionInfo& outInfo) const = 0;

    // owning body, assigned when world takes the collider in
    dynamics::RigidBody* getBody() const { return m_body; }
    void setBody(dynamics::RigidBody* body) { m_body = body; }

private:
    dynamics::RigidBody* m_body = nullptr;
};

} // namespace collider
} // namespace collision
} // namespace BulletPhysics
