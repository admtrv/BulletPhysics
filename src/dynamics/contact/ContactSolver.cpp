/*
 * ContactSolver.cpp
 */

#include "ContactSolver.h"

#include "dynamics/body/RigidBody.h"

#include <algorithm>

namespace BulletPhysics {
namespace dynamics {

static constexpr double PENETRATION_SLOP = 0.002;       // overlap left alone, full correction twitches
static constexpr double CORRECTION_RATE = 0.8;          // share of overlap removed per step
static constexpr double RESTITUTION_THRESHOLD = 1.0;    // below this contact stops bouncing
static constexpr double WAKE_IMPULSE = 0.5;             // hit strong enough to wake parked body

// everything contact needs to be solved, gathered once per manifold
struct ContactFrame {
    RigidBody* a = nullptr;
    RigidBody* b = nullptr;

    math::Vec3 normal{};
    math::Vec3 tangents[2]{};

    collision::PhysicsMaterial material;

    // nothing to solve when neither side can give anywhere
    bool valid() const
    {
        if (!a || !b)
        {
            return false;
        }

        for (int axis = 0; axis < 3; axis++)
        {
            if (a->canMoveAlong(axis) || b->canMoveAlong(axis) || a->canTurnAround(axis) || b->canTurnAround(axis))
            {
                return true;
            }
        }

        return false;
    }

    math::Vec3 relativeVelocityAt(const math::Vec3& point) const
    {
        return b->getVelocityAt(point) - a->getVelocityAt(point);
    }

    // frozen axis carries no impulse and must not lighten contact either
    double effectiveMass(const math::Vec3& direction, const math::Vec3& armA, const math::Vec3& armB) const
    {
        const math::Vec3 linear = (a->getLinearMobility() + b->getLinearMobility()) * direction;

        const math::Vec3 angularA = (a->getAngularMobility() * armA.cross(direction)).cross(armA);
        const math::Vec3 angularB = (b->getAngularMobility() * armB.cross(direction)).cross(armB);

        return direction.dot(linear + angularA + angularB);
    }

    void applyImpulse(const math::Vec3& armA, const math::Vec3& armB, const math::Vec3& impulse) const
    {
        // only real hit wakes parked body, resting weight keeps it up otherwise
        if (impulse.length() > WAKE_IMPULSE)
        {
            a->wake();
            b->wake();
        }

        a->applyImpulse(a->getLinearMobility() * impulse * -1.0, a->getAngularMobility() * armA.cross(impulse) * -1.0);
        b->applyImpulse(b->getLinearMobility() * impulse, b->getAngularMobility() * armB.cross(impulse));
    }
};

static ContactFrame frameOf(const collision::Manifold& manifold)
{
    ContactFrame frame;

    // trigger only reports touch, pushes nothing apart
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

    // tangents from normal alone, sliding velocity is noise on resting patch
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

            // slow contact bounces off its own noise
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

        // clamp total, single step often comes out negative
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

    // pinned pair still collides, just nowhere to push it
    if (!manifold.info.correctable())
    {
        return;
    }

    const double excess = manifold.info.penetration - PENETRATION_SLOP;
    if (excess <= 0.0)
    {
        return;
    }

    const math::Vec3 wanted = manifold.info.correction * (excess / manifold.info.penetration * CORRECTION_RATE);

    // each axis goes to bodies free to travel it, so what one cannot do other covers
    math::Vec3 toA{};
    math::Vec3 toB{};

    for (int axis = 0; axis < 3; axis++)
    {
        const double weightA = manifold.info.mobility.first.axes[axis] ? frame.a->getInverseMass() : 0.0;
        const double weightB = manifold.info.mobility.second.axes[axis] ? frame.b->getInverseMass() : 0.0;

        const double total = weightA + weightB;

        if (total <= 0.0)
        {
            continue;
        }

        toA[axis] = -wanted[axis] * weightA / total;
        toB[axis] = wanted[axis] * weightB / total;
    }

    frame.a->separate(toA);
    frame.b->separate(toB);
}

} // namespace dynamics
} // namespace BulletPhysics
