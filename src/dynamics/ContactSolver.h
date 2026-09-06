/*
 * ContactSolver.h
 */

#pragma once

#include "collision/Collision.h"

#include <vector>

namespace BulletPhysics {
namespace dynamics {

// resolves contacts, impulses carry into the next step or no stack ever settles
class ContactSolver {
public:
    // separating speed each contact aims for, measured before any pass
    void prepare(std::vector<collision::Manifold>& manifolds) const;

    // reapply what the contacts held last step
    void warmStart(std::vector<collision::Manifold>& manifolds) const;

    void solveVelocity(collision::Manifold& manifold) const;
    void correctPosition(const collision::Manifold& manifold) const;
};

} // namespace dynamics
} // namespace BulletPhysics
