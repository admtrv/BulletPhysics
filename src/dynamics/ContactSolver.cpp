/*
 * ContactSolver.cpp
 */

#include "ContactSolver.h"

#include "dynamics/RigidBody.h"

namespace BulletPhysics {
namespace dynamics {

// overlap left alone, full correction makes resting bodies twitch
static constexpr double PENETRATION_SLOP = 0.002;

// share of overlap removed per step
static constexpr double CORRECTION_RATE = 0.4;

void ContactSolver::solveVelocity(const collision::Manifold& manifold) const
{
    RigidBody* bodyA = manifold.colliderA->getBody();
    RigidBody* bodyB = manifold.colliderB->getBody();

    if (!bodyA || !bodyB || manifold.info.pointCount == 0)
    {
        return;
    }

    // static bodies have zero inverse mass, pair of them resolves to nothing
    const double inverseMassA = bodyA->getInverseMass();
    const double inverseMassB = bodyB->getInverseMass();

    const double inverseMassSum = inverseMassA + inverseMassB;
    if (inverseMassSum <= 0.0)
    {
        return;
    }

    const math::Mat3& inverseInertiaA = bodyA->getInverseInertia();
    const math::Mat3& inverseInertiaB = bodyB->getInverseInertia();

    const math::Vec3& normal = manifold.info.normal;

    const double share = 1.0 / manifold.info.pointCount;

    for (int i = 0; i < manifold.info.pointCount; i++)
    {
        const math::Vec3& point = manifold.info.points[i];

        const math::Vec3 armA = point - bodyA->getPosition();
        const math::Vec3 armB = point - bodyB->getPosition();

        // contact point moves with its body, not with the centre
        const double approachSpeed = (bodyB->getVelocityAt(point) - bodyA->getVelocityAt(point)).dot(normal);

        if (approachSpeed >= 0.0)
        {
            continue;
        }

        // effective mass, contact further from centre turns easier
        const math::Vec3 angularA = (inverseInertiaA * armA.cross(normal)).cross(armA);
        const math::Vec3 angularB = (inverseInertiaB * armB.cross(normal)).cross(armB);

        const double effectiveMass = inverseMassSum + normal.dot(angularA + angularB);
        if (effectiveMass <= 0.0)
        {
            continue;
        }

        const math::Vec3 impulse = normal * (-approachSpeed / effectiveMass * share);

        bodyA->setVelocity(bodyA->getVelocity() - impulse * inverseMassA);
        bodyB->setVelocity(bodyB->getVelocity() + impulse * inverseMassB);

        bodyA->setAngularVelocity(bodyA->getAngularVelocity() - inverseInertiaA * armA.cross(impulse));
        bodyB->setAngularVelocity(bodyB->getAngularVelocity() + inverseInertiaB * armB.cross(impulse));
    }
}

void ContactSolver::correctPosition(const collision::Manifold& manifold) const
{
    RigidBody* bodyA = manifold.colliderA->getBody();
    RigidBody* bodyB = manifold.colliderB->getBody();

    if (!bodyA || !bodyB)
    {
        return;
    }

    const double inverseMassA = bodyA->getInverseMass();
    const double inverseMassB = bodyB->getInverseMass();

    const double inverseMassSum = inverseMassA + inverseMassB;
    if (inverseMassSum <= 0.0)
    {
        return;
    }

    // killing velocity still leaves overlap, positions only
    const double excess = manifold.info.penetration - PENETRATION_SLOP;
    if (excess <= 0.0)
    {
        return;
    }

    const math::Vec3 separation = manifold.info.normal * (excess * CORRECTION_RATE / inverseMassSum);

    bodyA->setPosition(bodyA->getPosition() - separation * inverseMassA);
    bodyB->setPosition(bodyB->getPosition() + separation * inverseMassB);
}

} // namespace dynamics
} // namespace BulletPhysics
