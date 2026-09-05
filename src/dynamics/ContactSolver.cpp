/*
 * ContactSolver.cpp
 */

#include "ContactSolver.h"

#include "dynamics/RigidBody.h"

namespace BulletPhysics {
namespace dynamics {

void ContactSolver::resolve(const collision::Manifold& manifold) const
{
    RigidBody* bodyA = manifold.colliderA->getBody();
    RigidBody* bodyB = manifold.colliderB->getBody();

    if (!bodyA || !bodyB)
    {
        return;
    }

    // static bodies report zero inverse mass, pair of them has nothing to resolve
    const double inverseMassA = bodyA->getInverseMass();
    const double inverseMassB = bodyB->getInverseMass();

    const double inverseMassSum = inverseMassA + inverseMassB;
    if (inverseMassSum <= 0.0)
    {
        return;
    }

    // normal points from A to B
    const math::Vec3& normal = manifold.info.normal;

    const math::Vec3 velocityA = bodyA->getVelocity();
    const math::Vec3 velocityB = bodyB->getVelocity();

    const double approachSpeed = (velocityB - velocityA).dot(normal);

    // cancel motion that drives bodies into each other, skip if they already separate
    if (approachSpeed < 0.0)
    {
        const math::Vec3 impulse = normal * (-approachSpeed / inverseMassSum);

        bodyA->setVelocity(velocityA - impulse * inverseMassA);
        bodyB->setVelocity(velocityB + impulse * inverseMassB);
    }

    // and push apart, killing velocity still leaves them overlapping
    const math::Vec3 separation = normal * (manifold.info.penetration / inverseMassSum);

    bodyA->setPosition(bodyA->getPosition() - separation * inverseMassA);
    bodyB->setPosition(bodyB->getPosition() + separation * inverseMassB);
}

} // namespace dynamics
} // namespace BulletPhysics
