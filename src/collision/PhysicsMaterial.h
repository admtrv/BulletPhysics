/*
 * PhysicsMaterial.h
 */

#pragma once

#include <algorithm>
#include <cmath>

namespace BulletPhysics {
namespace collision {

// surface response of a collider
struct PhysicsMaterial {
    double friction = 0.5;      // coulomb coefficient, 0 slides freely
    double restitution = 0.0;   // bounciness, 1 keeps all speed

    // pair of surfaces acts as one
    static PhysicsMaterial combine(const PhysicsMaterial& a, const PhysicsMaterial& b)
    {
        return {
            std::sqrt(a.friction * b.friction),
            std::max(a.restitution, b.restitution)
        };
    }
};

namespace materials {

inline PhysicsMaterial Rubber() { return {0.9, 0.8}; }
inline PhysicsMaterial Wood() { return {0.5, 0.2}; }
inline PhysicsMaterial Metal() { return {0.4, 0.3}; }
inline PhysicsMaterial Ice() { return {0.05, 0.05}; }

} // namespace materials
} // namespace collision
} // namespace BulletPhysics
