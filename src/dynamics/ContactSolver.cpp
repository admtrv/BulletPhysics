/*
 * ContactSolver.cpp
 */

#include "ContactSolver.h"

#include "dynamics/RigidBody.h"

#include <algorithm>

namespace BulletPhysics {
namespace dynamics {

static constexpr double PENETRATION_SLOP = 0.002;       // overlap left alone, full correction twitches
static constexpr double CORRECTION_RATE = 0.8;          // share of overlap removed per step
static constexpr double RESTITUTION_THRESHOLD = 1.0;    // below this a contact stops bouncing
static constexpr double WAKE_IMPULSE = 0.5;             // hit strong enough to wake a parked body

// everything a contact needs to be solved, gathered once per manifold
struct ContactFrame {
    RigidBody* a = nullptr;
    RigidBody* b = nullptr;

    math::Vec3 normal{};
    math::Vec3 tangents[2]{};

    collision::PhysicsMaterial material;

    bool valid() const { return a && b && (a->getInverseMass() + b->getInverseMass()) > 0.0; }

    // relative velocity of the two bodies where they touch
    math::Vec3 relativeVelocityAt(const math::Vec3& point) const
    {
        return b->getVelocityAt(point) - a->getVelocityAt(point);
    }

    // resistance along the direction, contact further from centre turns easier
    double effectiveMass(const math::Vec3& direction, const math::Vec3& armA, const math::Vec3& armB) const
    {
        const math::Vec3 angularA = (a->getInverseInertia() * armA.cross(direction)).cross(armA);
        const math::Vec3 angularB = (b->getInverseInertia() * armB.cross(direction)).cross(armB);

        return a->getInverseMass() + b->getInverseMass() + direction.dot(angularA + angularB);
    }

    void applyImpulse(const math::Vec3& armA, const math::Vec3& armB, const math::Vec3& impulse) const
    {
        // only a real hit wakes a parked body, resting weight would keep it up
        if (impulse.length() > WAKE_IMPULSE)
        {
            a->wake();
            b->wake();
        }

        a->setVelocity(a->getVelocity() - impulse * a->getInverseMass());
        b->setVelocity(b->getVelocity() + impulse * b->getInverseMass());

        a->setAngularVelocity(a->getAngularVelocity() - a->getInverseInertia() * armA.cross(impulse));
        b->setAngularVelocity(b->getAngularVelocity() + b->getInverseInertia() * armB.cross(impulse));
    }
};

static ContactFrame frameOf(const collision::Manifold& manifold)
{
    ContactFrame frame;

    // a trigger only reports the touch, nothing is pushed apart
    if (manifold.colliderA->isTrigger() || manifold.colliderB->isTrigger())
    {
        return frame;
    }

    frame.a = manifold.colliderA->getBody();
    frame.b = manifold.colliderB->getBody();
    frame.normal = manifold.info.normal;

    frame.material = collision::PhysicsMaterial::combine(
        manifold.colliderA->getMaterial(),
        manifold.colliderB->getMaterial());

    // tangents from the normal alone, sliding velocity is noise on a resting patch
    const math::Vec3 reference = (std::abs(frame.normal.x) < 0.9) ? math::Vec3{1.0, 0.0, 0.0} : math::Vec3{0.0, 1.0, 0.0};

    frame.tangents[0] = frame.normal.cross(reference).normalized();
    frame.tangents[1] = frame.normal.cross(frame.tangents[0]);

    return frame;
}

void ContactSolver::prepare(std::vector<collision::Manifold>& manifolds) const
{
    for (auto& manifold : manifolds)
    {
        const ContactFrame frame = frameOf(manifold);
        if (!frame.valid())
        {
            continue;
        }

        for (int i = 0; i < manifold.info.pointCount; i++)
        {
            collision::collider::ContactPoint& point = manifold.info.points[i];

            const double approachSpeed = frame.relativeVelocityAt(point.position).dot(frame.normal);

            // slow contact would bounce off its own noise
            point.targetSpeed = (-approachSpeed > RESTITUTION_THRESHOLD) ? -approachSpeed * frame.material.restitution : 0.0;
        }
    }
}

void ContactSolver::warmStart(std::vector<collision::Manifold>& manifolds) const
{
    for (auto& manifold : manifolds)
    {
        const ContactFrame frame = frameOf(manifold);
        if (!frame.valid())
        {
            continue;
        }

        for (int i = 0; i < manifold.info.pointCount; i++)
        {
            const collision::collider::ContactPoint& point = manifold.info.points[i];

            const math::Vec3 armA = point.position - frame.a->getPosition();
            const math::Vec3 armB = point.position - frame.b->getPosition();

            frame.applyImpulse(armA, armB, frame.normal * point.normalImpulse
                + frame.tangents[0] * point.tangentImpulses[0]
                + frame.tangents[1] * point.tangentImpulses[1]);
        }
    }
}

void ContactSolver::solveVelocity(collision::Manifold& manifold) const
{
    const ContactFrame frame = frameOf(manifold);
    if (!frame.valid())
    {
        return;
    }

    for (int i = 0; i < manifold.info.pointCount; i++)
    {
        collision::collider::ContactPoint& point = manifold.info.points[i];

        const math::Vec3 armA = point.position - frame.a->getPosition();
        const math::Vec3 armB = point.position - frame.b->getPosition();

        const double normalMass = frame.effectiveMass(frame.normal, armA, armB);
        if (normalMass <= 0.0)
        {
            continue;
        }

        const double approachSpeed = frame.relativeVelocityAt(point.position).dot(frame.normal);

        // clamp the total, a single step may well be negative
        const double held = point.normalImpulse;
        point.normalImpulse = std::max(held + (point.targetSpeed - approachSpeed) / normalMass, 0.0);

        frame.applyImpulse(armA, armB, frame.normal * (point.normalImpulse - held));

        if (frame.material.friction <= 0.0)
        {
            continue;
        }

        // coulomb limit, surface holds only so much before it slips
        const double limit = frame.material.friction * point.normalImpulse;

        for (int axis = 0; axis < 2; axis++)
        {
            const math::Vec3& tangent = frame.tangents[axis];

            const double tangentMass = frame.effectiveMass(tangent, armA, armB);
            if (tangentMass <= 0.0)
            {
                continue;
            }

            const double slidingSpeed = frame.relativeVelocityAt(point.position).dot(tangent);

            const double heldTangent = point.tangentImpulses[axis];
            point.tangentImpulses[axis] = std::clamp(heldTangent - slidingSpeed / tangentMass, -limit, limit);

            frame.applyImpulse(armA, armB, tangent * (point.tangentImpulses[axis] - heldTangent));
        }
    }
}

void ContactSolver::correctPosition(const collision::Manifold& manifold) const
{
    const ContactFrame frame = frameOf(manifold);
    if (!frame.valid())
    {
        return;
    }

    const double excess = manifold.info.penetration - PENETRATION_SLOP;
    if (excess <= 0.0)
    {
        return;
    }

    const double inverseMassSum = frame.a->getInverseMass() + frame.b->getInverseMass();
    const math::Vec3 separation = frame.normal * (excess * CORRECTION_RATE / inverseMassSum);

    frame.a->setPosition(frame.a->getPosition() - separation * frame.a->getInverseMass());
    frame.b->setPosition(frame.b->getPosition() + separation * frame.b->getInverseMass());
}

} // namespace dynamics
} // namespace BulletPhysics
