/*
 * TestPhysicsWorld.cpp
 */

#include <gtest/gtest.h>

#include "ballistics/external/PhysicsWorld.h"
#include "ballistics/external/environments/Atmosphere.h"
#include "ballistics/external/environments/Humidity.h"
#include "builtin/bodies/RigidBody.h"

namespace BulletPhysics {
namespace tests {

using namespace ballistics::external;

class TestPhysicsWorld : public ::testing::Test {
protected:
    PhysicsWorld world;
};

// environments sorted by priority on insertion

TEST_F(TestPhysicsWorld, EnvironmentsSortedByPriority)
{
    world.addEnvironment(std::make_unique<environments::Humidity>());       // priority 10
    world.addEnvironment(std::make_unique<environments::Atmosphere>());     // priority 0

    const auto& envs = world.getEnvironments();

    ASSERT_EQ(envs.size(), 2u);
    ASSERT_EQ(envs[0]->getName(), "Atmosphere");
    ASSERT_EQ(envs[1]->getName(), "Humidity");
}

TEST_F(TestPhysicsWorld, EnvironmentsSortedRegardlessOrder)
{
    world.addEnvironment(std::make_unique<environments::Atmosphere>());
    world.addEnvironment(std::make_unique<environments::Humidity>());

    const auto& envs = world.getEnvironments();

    ASSERT_EQ(envs.size(), 2u);
    ASSERT_EQ(envs[0]->getName(), "Atmosphere");
    ASSERT_EQ(envs[1]->getName(), "Humidity");
}

} // namespace tests
} // namespace BulletPhysics
