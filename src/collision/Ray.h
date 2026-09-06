/*
 * Ray.h
 */

#pragma once

#include "math/Vec3.h"

namespace BulletPhysics {
namespace collision {

namespace collider {
class Collider;
}

struct Ray {
    math::Vec3 origin{};
    math::Vec3 direction{0.0, 0.0, -1.0};   // expected to be unit length
    double maxDistance = 1e30;

    math::Vec3 pointAt(double distance) const { return origin + direction * distance; }
};

struct RayHit {
    collider::Collider* collider = nullptr;

    math::Vec3 point{};
    math::Vec3 normal{};        // surface normal at the hit, faces the ray
    double distance = 0.0;
};

} // namespace collision
} // namespace BulletPhysics
