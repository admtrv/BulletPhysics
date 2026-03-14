/*
 * Impact.h
 */

#pragma once

#include "Material.h"
#include "PhysicsBody.h"

#include <algorithm>

namespace BulletPhysics {
namespace ballistics {
namespace terminal {

// input

struct ImpactInfo {
    math::Vec3 normal;      // surface normal at impact point
    Material material;      // material properties of target
    double thickness;       // m, effective thickness of obstacle along velocity direction
};

// output

enum class ImpactOutcome {
    Ricochet,
    Penetration,
    Embed
};

struct ImpactResult {
    ImpactOutcome outcome;

    math::Vec3 residualVelocity;    // m/s, post-impact velocity (zero for Embed)
    double energyAbsorbed;          // J, transferred to material
    double penetrationDepth;        // m, how far into material
};

class Impact {
public:
    static ImpactResult resolve(const projectile::IProjectileBody& projectile, const ImpactInfo& info);
};

} // namespace terminal
} // namespace ballistics
} // namespace BulletPhysics
