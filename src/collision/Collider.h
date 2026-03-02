/*
 * Collider.h
 */

#pragma once

#include "math/Vec3.h"
#include "collision/terminal/Material.h"

#include <optional>

namespace BulletPhysics {
namespace collision {

enum class CollisionShape {
    Box,
    Ground,
};

struct CollisionInfo {
    double penetration = 0.0;
    math::Vec3 normal{};
};

class Collider {
public:
    virtual ~Collider() = default;

    virtual CollisionShape getShape() const = 0;
    virtual const math::Vec3& getPosition() const = 0;
    virtual void setPosition(const math::Vec3& pos) = 0;

    virtual bool testCollision(const Collider& other, CollisionInfo& outInfo) const = 0;
    virtual bool testPoint(const math::Vec3& point) const = 0;

    // ray thickness
    virtual double computeThickness(const math::Vec3& rayOrigin, const math::Vec3& rayDir) const { return 1e6; }

    void setMaterial(const terminal::Material& mat) { m_material = mat; }
    const std::optional<terminal::Material>& getMaterial() const { return m_material; }

private:
    std::optional<terminal::Material> m_material;
};

} // namespace collision
} // namespace BulletPhysics
