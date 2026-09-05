/*
 * ContactSolver.h
 */

#pragma once

#include "collision/Collision.h"

namespace BulletPhysics {
namespace dynamics {

// resolves contacts so bodies stop moving into each other
class ContactSolver {
public:
    void resolve(const collision::Manifold& manifold) const;
};

} // namespace dynamics
} // namespace BulletPhysics
