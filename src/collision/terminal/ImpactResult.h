/*
 * ImpactResult.h
 */

#pragma once

#include "math/Vec3.h"

namespace BulletPhysics {
namespace collision {
namespace terminal {

enum class ImpactOutcome {
    Ricochet,
    Penetration,
    Embed
};

struct ImpactResult {
    ImpactOutcome outcome;
    math::Vec3 residualVelocity;  // post-impact velocity (zero for Embed)
    float energyAbsorbed;         // Joules transferred to material
    float penetrationDepth;       // m — how far into material
};

} // namespace terminal
} // namespace collision
} // namespace BulletPhysics
