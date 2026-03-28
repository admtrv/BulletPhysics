/*
 * TestCoordinateMapping.cpp
 */

#include <gtest/gtest.h>

#include "geography/CoordinateMapping.h"

namespace BulletPhysics {
namespace tests {

using namespace geography;

class TestCoordinateMapping : public ::testing::Test {};

// EUN (identity)

TEST_F(TestCoordinateMapping, EUNIsIdentity)
{
    auto m = mappings::EUN();
    EXPECT_TRUE(m.isIdentity());
}

TEST_F(TestCoordinateMapping, EUNRoundTrip)
{
    auto m = mappings::EUN();
    math::Vec3 v{1.0, 2.0, 3.0};

    auto internal = m.toInternal(v);
    EXPECT_DOUBLE_EQ(internal.x, 1.0);
    EXPECT_DOUBLE_EQ(internal.y, 2.0);
    EXPECT_DOUBLE_EQ(internal.z, 3.0);

    auto external = m.toExternal(internal);
    EXPECT_DOUBLE_EQ(external.x, v.x);
    EXPECT_DOUBLE_EQ(external.y, v.y);
    EXPECT_DOUBLE_EQ(external.z, v.z);
}

// Godot (x=East, y=Up, z=-North)

TEST_F(TestCoordinateMapping, GodotToInternal)
{
    auto m = mappings::Godot();

    // Godot: (1, 2, -3) -> EUN: East=1, Up=2, North=3
    auto eun = m.toInternal({1.0, 2.0, -3.0});

    EXPECT_DOUBLE_EQ(eun.x, 1.0);
    EXPECT_DOUBLE_EQ(eun.y, 2.0);
    EXPECT_DOUBLE_EQ(eun.z, 3.0);
}

TEST_F(TestCoordinateMapping, GodotToExternal)
{
    auto m = mappings::Godot();

    // EUN: East=1, Up=2, North=3 -> Godot: (1, 2, -3)
    auto godot = m.toExternal({1.0, 2.0, 3.0});

    EXPECT_DOUBLE_EQ(godot.x, 1.0);
    EXPECT_DOUBLE_EQ(godot.y, 2.0);
    EXPECT_DOUBLE_EQ(godot.z, -3.0);
}

TEST_F(TestCoordinateMapping, GodotRoundTrip)
{
    auto m = mappings::Godot();
    math::Vec3 v{5.0, -1.0, 7.0};

    auto result = m.toExternal(m.toInternal(v));
    EXPECT_DOUBLE_EQ(result.x, v.x);
    EXPECT_DOUBLE_EQ(result.y, v.y);
    EXPECT_DOUBLE_EQ(result.z, v.z);
}

// Unreal (x=North, y=East, z=Up)

TEST_F(TestCoordinateMapping, UnrealToInternal)
{
    auto m = mappings::Unreal();

    // Unreal: x=North=3, y=East=1, z=Up=2 -> EUN: (1, 2, 3)
    auto eun = m.toInternal({3.0, 1.0, 2.0});

    EXPECT_DOUBLE_EQ(eun.x, 1.0);
    EXPECT_DOUBLE_EQ(eun.y, 2.0);
    EXPECT_DOUBLE_EQ(eun.z, 3.0);
}

TEST_F(TestCoordinateMapping, UnrealToExternal)
{
    auto m = mappings::Unreal();

    // EUN: (1, 2, 3) -> Unreal: x=3, y=1, z=2
    auto unreal = m.toExternal({1.0, 2.0, 3.0});

    EXPECT_DOUBLE_EQ(unreal.x, 3.0);
    EXPECT_DOUBLE_EQ(unreal.y, 1.0);
    EXPECT_DOUBLE_EQ(unreal.z, 2.0);
}

TEST_F(TestCoordinateMapping, UnrealRoundTrip)
{
    auto m = mappings::Unreal();
    math::Vec3 v{-2.0, 4.0, 6.0};

    auto result = m.toExternal(m.toInternal(v));
    EXPECT_DOUBLE_EQ(result.x, v.x);
    EXPECT_DOUBLE_EQ(result.y, v.y);
    EXPECT_DOUBLE_EQ(result.z, v.z);
}

// OpenGL same as Godot

TEST_F(TestCoordinateMapping, OpenGLSameAsGodot)
{
    auto gl = mappings::OpenGL();
    auto godot = mappings::Godot();
    math::Vec3 v{3.0, -1.0, 5.0};

    auto gl_int = gl.toInternal(v);
    auto godot_int = godot.toInternal(v);

    EXPECT_DOUBLE_EQ(gl_int.x, godot_int.x);
    EXPECT_DOUBLE_EQ(gl_int.y, godot_int.y);
    EXPECT_DOUBLE_EQ(gl_int.z, godot_int.z);
}

// Custom mapping

TEST_F(TestCoordinateMapping, CustomMappingRoundTrip)
{
    // East=NEG_Z, Up=POS_X, North=POS_Y
    CoordinateMapping m(Axis::NEG_Z, Axis::POS_X, Axis::POS_Y);

    math::Vec3 user{2.0, 3.0, -1.0};
    auto eun = m.toInternal(user);

    EXPECT_DOUBLE_EQ(eun.x, 1.0);  // East = -user.z
    EXPECT_DOUBLE_EQ(eun.y, 2.0);  // Up = user.x
    EXPECT_DOUBLE_EQ(eun.z, 3.0);  // North = user.y

    auto back = m.toExternal(eun);
    EXPECT_DOUBLE_EQ(back.x, user.x);
    EXPECT_DOUBLE_EQ(back.y, user.y);
    EXPECT_DOUBLE_EQ(back.z, user.z);
}

// Zero vector

TEST_F(TestCoordinateMapping, ZeroVectorUnchanged)
{
    auto presets = {mappings::EUN(), mappings::Godot(), mappings::Unreal()};
    math::Vec3 zero{0, 0, 0};

    for (auto& m : presets)
    {
        auto result = m.toInternal(zero);
        EXPECT_DOUBLE_EQ(result.x, 0.0);
        EXPECT_DOUBLE_EQ(result.y, 0.0);
        EXPECT_DOUBLE_EQ(result.z, 0.0);
    }
}

} // namespace tests
} // namespace BulletPhysics
