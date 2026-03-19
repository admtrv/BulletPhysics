/*
 * TestCoriolis.cpp
 */

#include <gtest/gtest.h>

#include "ballistics/external/forces/Coriolis.h"
#include "ballistics/external/PhysicsContext.h"
#include "builtin/bodies/RigidBody.h"
#include "math/Angles.h"

namespace BulletPhysics {
namespace tests {

using namespace ballistics::external;
using namespace builtin::bodies;

class TestCoriolis : public ::testing::Test {
protected:
    forces::Coriolis coriolis;
    RigidBody body;
    PhysicsContext context;

    void SetUp() override
    {
        body.setMass(1.0);
        body.setPosition({0.0, 0.0, 0.0});
        context.reset();
    }
};

// helpers

void apply(forces::Coriolis& coriolis, RigidBody& body, PhysicsContext& context)
{
    body.clearForces();
    coriolis.apply(body, context);
}

// northern hemisphere: projectile moving north deflects east

TEST_F(TestCoriolis, NorthernHemisphereNorthwardDeflectsEast)
{
    context.latitude = math::deg2rad(45.0);
    body.setVelocity({0.0, 0.0, 800.0});

    apply(coriolis, body, context);

    ASSERT_GT(body.getAccumulatedForces().x, 0.0);
}

// northern hemisphere: projectile moving east deflects south

TEST_F(TestCoriolis, NorthernHemisphereEastwardDeflectsSouth)
{
    context.latitude = math::deg2rad(45.0);
    body.setVelocity({800.0, 0.0, 0.0}); // east

    apply(coriolis, body, context);

    ASSERT_LT(body.getAccumulatedForces().z, 0.0);
}

// southern hemisphere: projectile moving north deflects west

TEST_F(TestCoriolis, SouthernHemisphereNorthwardDeflectsWest)
{
    context.latitude = math::deg2rad(-45.0);
    body.setVelocity({0.0, 0.0, 800.0}); // north

    apply(coriolis, body, context);

    ASSERT_LT(body.getAccumulatedForces().x, 0.0);
}

// southern hemisphere: projectile moving east deflects north

TEST_F(TestCoriolis, SouthernHemisphereEastwardDeflectsNorth)
{
    context.latitude = math::deg2rad(-45.0);
    body.setVelocity({800.0, 0.0, 0.0}); // east

    apply(coriolis, body, context);

    ASSERT_GT(body.getAccumulatedForces().z, 0.0);
}

// equator: horizontal motion has no horizontal deflection

TEST_F(TestCoriolis, EquatorNorthwardDeflectsEast)
{
    context.latitude = math::deg2rad(0.0);
    body.setVelocity({0.0, 0.0, 800.0}); // north

    apply(coriolis, body, context);

    ASSERT_NEAR(body.getAccumulatedForces().x, 0.0, 1e-12);
}

// equator: eastward motion deflects upward

TEST_F(TestCoriolis, EquatorEastwardDeflectsUp)
{
    context.latitude = math::deg2rad(0.0);
    body.setVelocity({800.0, 0.0, 0.0}); // east

    apply(coriolis, body, context);

    ASSERT_GT(body.getAccumulatedForces().y, 0.0);
}

// pole: maximum horizontal deflection

TEST_F(TestCoriolis, NorthPoleNorthwardDeflectsEast)
{
    context.latitude = math::deg2rad(90.0);
    body.setVelocity({0.0, 0.0, 800.0}); // north

    apply(coriolis, body, context);

    ASSERT_GT(body.getAccumulatedForces().x, 0.0);
}

// opposite hemispheres produce opposite horizontal deflection

TEST_F(TestCoriolis, OppositeHemispheresOppositeDeflection)
{
    body.setVelocity({0.0, 0.0, 800.0}); // north

    context.latitude = math::deg2rad(45.0);
    apply(coriolis, body, context);
    double forceNorth = body.getAccumulatedForces().x;

    context.latitude = math::deg2rad(-45.0);
    apply(coriolis, body, context);
    double forceSouth = body.getAccumulatedForces().x;

    ASSERT_GT(forceNorth, 0.0);
    ASSERT_LT(forceSouth, 0.0);
    ASSERT_NEAR(forceNorth, -forceSouth, 1e-12);
}

} // namespace tests
} // namespace BulletPhysics
