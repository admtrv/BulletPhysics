/*
 * ImpactResolver.h
 */

#pragma once

#include "Material.h"
#include "ImpactResult.h"
#include "collision/Collider.h"
#include "dynamics/PhysicsBody.h"
#include "Constants.h"
#include "collision/BoxCollider.h"
#include "math/Angles.h"
#include "math/Constants.h"

#include <cmath>
#include <algorithm>
#include <cstdio>
#include <vector>

namespace BulletPhysics {
namespace collision {
namespace terminal {

class ImpactResolver {
public:
    // single-layer impact resolution
    static ImpactResult resolve(const dynamics::projectile::IProjectileBody& projectile, const CollisionInfo& info, const Collider& collider);

private:
    // critical grazing angle for ricochet (radians from surface), Wijk
    static float criticalAngle(float mass, float speed, float diameter, float yieldStrength);

    // penetration depth (meters), energy-based model
    static float maxPenetrationDepth(float mass, float speed, float area, float penetrationResistance);

    // residual velocity after penetration, energy-based model
    static float residualSpeed(float speed, float thickness, float maxPenetration);

    // compute path length of ray through collider along velocity direction
    static float computeEffectiveThickness(const math::Vec3& position, const math::Vec3& velocity, const Collider* collider);
};

} // namespace terminal
} // namespace collision
} // namespace BulletPhysics
