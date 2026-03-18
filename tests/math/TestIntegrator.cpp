/*
 * TestIntegrator.cpp
 */

#include <gtest/gtest.h>

#include "math/Integrator.h"
#include "Constants.h"
#include "builtin/bodies/RigidBody.h"
#include "ballistics/external/PhysicsWorld.h"
#include "ballistics/external/forces/Gravity.h"

#include <cmath>
#include <memory>

namespace BulletPhysics {
namespace tests {

class TestIntegrator : public ::testing::Test {
protected:
    builtin::bodies::RigidBody body;
    ballistics::external::PhysicsWorld world;

    void SetUp() override
    {
        body.setMass(1.0);
        body.setPosition({0.0, 100.0, 0.0});
        body.setVelocity({0.0, 0.0, 0.0});

        world.clear();
        world.addForce(std::make_unique<ballistics::external::forces::Gravity>());
    }
};

// helpers

void simulate(math::IIntegrator& integrator, IPhysicsBody& body, ballistics::external::PhysicsWorld& world, double dt, int steps)
{
    for (int i = 0; i < steps; i++)
    {
        integrator.step(body, &world, dt);
    }
}

double position(double t)
{
    double g = -constants::GRAVITY.y;
    return 100.0 - 0.5 * g * t * t;
}

double velocity(double t)
{
    return constants::GRAVITY.y * t;
}

TEST_F(TestIntegrator, EulerFreeFall)
{
    math::EulerIntegrator euler;

    simulate(euler, body, world, 0.001, 1000);

    ASSERT_NEAR(body.getPosition().y, position(1.0), 0.1);
    ASSERT_NEAR(body.getVelocity().y, velocity(1.0), 0.1);
}

TEST_F(TestIntegrator, MidpointFreeFall)
{
    math::MidpointIntegrator midpoint;

    simulate(midpoint, body, world, 0.001, 1000);

    ASSERT_NEAR(body.getPosition().y, position(1.0), 0.01);
    ASSERT_NEAR(body.getVelocity().y, velocity(1.0), 0.01);
}

TEST_F(TestIntegrator, RK4FreeFall)
{
    math::RK4Integrator rk4;

    simulate(rk4, body, world, 0.001, 1000);

    ASSERT_NEAR(body.getPosition().y, position(1.0), 0.001);
    ASSERT_NEAR(body.getVelocity().y, velocity(1.0), 0.001);
}

TEST_F(TestIntegrator, FreeFallNoHorizontalMotion)
{
    math::RK4Integrator rk4;

    simulate(rk4, body, world, 0.01, 100);

    ASSERT_DOUBLE_EQ(body.getPosition().x, 0.0);
    ASSERT_DOUBLE_EQ(body.getPosition().z, 0.0);
    ASSERT_DOUBLE_EQ(body.getVelocity().x, 0.0);
    ASSERT_DOUBLE_EQ(body.getVelocity().z, 0.0);
}

TEST_F(TestIntegrator, NullWorldUniformMotion)
{
    body.setPosition({0.0, 0.0, 0.0});
    body.setVelocity({10.0, 0.0, 0.0});

    math::RK4Integrator rk4;
    for (int i = 0; i < 100; i++)
    {
        rk4.step(body, nullptr, 0.01);
    }

    ASSERT_NEAR(body.getPosition().x, 10.0, 1e-10);
    ASSERT_DOUBLE_EQ(body.getVelocity().x, 10.0);
}

} // namespace tests
} // namespace BulletPhysics
