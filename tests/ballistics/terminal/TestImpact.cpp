/*
 * TestImpact.cpp
 */

#include <gtest/gtest.h>

#include "ballistics/terminal/Impact.h"
#include "ballistics/terminal/Material.h"
#include "builtin/bodies/RigidBody.h"

namespace BulletPhysics {
namespace tests {

using namespace ballistics::terminal;
using namespace builtin::bodies;

class TestImpact : public ::testing::Test {
protected:
    projectile::ProjectileSpecs specs = projectile::presets::Nato762();
};

// helpers

ImpactResult resolve(ProjectileRigidBody& body, const math::Vec3& normal, const Material& material, double thickness)
{
    ImpactInfo info{normal, material, thickness};
    return Impact::resolve(body, info);
}

// ricochet: grazing angle on steel at shallow angle

TEST_F(TestImpact, SteelShallowAngleRicochet)
{
    ProjectileRigidBody body(specs);
    body.setVelocity({800.0, -20.0, 0.0}); // nearly parallel to surface

    auto result = resolve(body, {0.0, 1.0, 0.0}, materials::Steel(), 0.01);

    ASSERT_EQ(result.outcome, ImpactOutcome::Ricochet);
    ASSERT_GT(result.residualVelocity.length(), 0.0);
    ASSERT_GT(result.energyAbsorbed, 0.0);
}

// penetration: thin wood at steep angle

TEST_F(TestImpact, ThinWoodPenetration)
{
    ProjectileRigidBody body(specs);
    body.setVelocity({0.0, -800.0, 0.0}); // straight down

    auto result = resolve(body, {0.0, 1.0, 0.0}, materials::Wood(), 0.02);

    ASSERT_EQ(result.outcome, ImpactOutcome::Penetration);
    ASSERT_GT(result.residualVelocity.length(), 0.0);
    ASSERT_LT(result.residualVelocity.length(), 800.0);
}

// embed: thick steel stops projectile

TEST_F(TestImpact, ThickSteelEmbed)
{
    ProjectileRigidBody body(specs);
    body.setVelocity({0.0, -800.0, 0.0}); // straight into thick steel

    auto result = resolve(body, {0.0, 1.0, 0.0}, materials::Steel(), 0.5);

    ASSERT_EQ(result.outcome, ImpactOutcome::Embed);
    ASSERT_DOUBLE_EQ(result.residualVelocity.length(), 0.0);
}


} // namespace tests
} // namespace BulletPhysics
