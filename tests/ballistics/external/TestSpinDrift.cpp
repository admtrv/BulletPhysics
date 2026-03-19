/*
 * TestSpinDrift.cpp
 */

#include <gtest/gtest.h>

#include "ballistics/external/forces/SpinDrift.h"
#include "ballistics/external/PhysicsContext.h"
#include "builtin/bodies/RigidBody.h"

namespace BulletPhysics {
namespace tests {

using namespace ballistics::external;
using namespace builtin::bodies;

class TestSpinDrift : public ::testing::Test {
protected:
    PhysicsContext context;

    void SetUp() override
    {
        context.reset();
    }
};

// helpers

double SpinDriftX(ProjectileRigidBody& body, PhysicsContext& context)
{
    body.clearForces();

    forces::Lift lift;
    forces::Magnus magnus;

    lift.apply(body, context);
    magnus.apply(body, context);

    return body.getAccumulatedForces().x;
}

// right twist deflects right

TEST_F(TestSpinDrift, RightTwistDeflectsRight)
{
    auto specs = projectile::ProjectileSpecs::create(0.0095, 0.00762)
        .withDragModel(forces::drag::DragCurveModel::G7)
        .withMuzzle(800.0, projectile::Direction::RIGHT, 12.0);

    ProjectileRigidBody body(specs);
    body.setVelocity({0.0, 0.0, 800.0}); // north

    double fx = SpinDriftX(body, context);

    ASSERT_GT(fx, 0.0);
}

// left twist deflects left

TEST_F(TestSpinDrift, LeftTwistDeflectsLeft)
{
    auto specs = projectile::ProjectileSpecs::create(0.0095, 0.00762)
        .withDragModel(forces::drag::DragCurveModel::G7)
        .withMuzzle(800.0, projectile::Direction::LEFT, 12.0);

    ProjectileRigidBody body(specs);
    body.setVelocity({0.0, 0.0, 800.0}); // north

    double fx = SpinDriftX(body, context);

    ASSERT_LT(fx, 0.0);
}

} // namespace tests
} // namespace BulletPhysics
