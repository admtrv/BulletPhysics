/*
 * Query.h
 */

#pragma once

#include "math/Vec3.h"

namespace BulletPhysics {
namespace collision {

namespace collider {
class Collider;
}

// ray

struct Ray {
    math::Vec3 origin{};
    math::Vec3 direction{0.0, 0.0, -1.0};   // expected to be unit length
    double maxDistance = 1e30;

    math::Vec3 pointAt(double distance) const { return origin + direction * distance; }
};

struct RayHit {
    collider::Collider* collider = nullptr;

    math::Vec3 point{};
    math::Vec3 normal{};        // surface normal at hit, faces ray
    double distance = 0.0;
};

// sweep

// sphere carried along path, what fast body sweeps through between steps
struct Sweep {
    math::Vec3 origin{};
    math::Vec3 direction{0.0, -1.0, 0.0};   // expected to be unit length
    double distance = 0.0;
    double radius = 0.0;
};

struct SweepHit {
    collider::Collider* collider = nullptr;

    math::Vec3 point{};         // on surface, not sphere centre
    math::Vec3 normal{};        // surface normal at hit, faces sweep
    double distance = 0.0;      // travelled before touching
};

} // namespace collision
} // namespace BulletPhysics
