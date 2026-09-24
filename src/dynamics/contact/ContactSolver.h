/*
 * ContactSolver.h
 */

#pragma once

#include "collision/Collision.h"

#include <vector>

namespace BulletPhysics {
namespace dynamics {

class ContactSolver {
public:
    void prepare(std::vector<collision::Manifold>& manifolds) const;
    void warmStart(std::vector<collision::Manifold>& manifolds) const;    // reapplies what contacts held last step

    void solveVelocity(collision::Manifold& manifold) const;
    void correctPosition(const collision::Manifold& manifold) const;
};

} // namespace dynamics
} // namespace BulletPhysics
