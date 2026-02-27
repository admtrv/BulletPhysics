/*
 * Impact.h
 */

#pragma once

#include "Material.h"
#include "math/Vec3.h"
#include "dynamics/PhysicsBody.h"

namespace BulletPhysics {
namespace collision {
namespace terminal {

// input

struct ImpactInfo {
    math::Vec3 normal;            // surface normal at impact point
    Material material;            // material properties of target
    float thickness;              // m, effective thickness of obstacle along velocity direction
};

// output

enum class ImpactOutcome {
    Ricochet,
    Penetration,
    Embed
};

struct ImpactResult {
    ImpactOutcome outcome;

    math::Vec3 residualVelocity;  // m/s, post-impact velocity (zero for Embed)
    float energyAbsorbed;         // J, transferred to material
    float penetrationDepth;       // m, how far into material
};

class Impact {
public:
    static ImpactResult resolve(const dynamics::projectile::IProjectileBody& projectile, const ImpactInfo& info);
};

} // namespace terminal
} // namespace collision
} // namespace BulletPhysics
