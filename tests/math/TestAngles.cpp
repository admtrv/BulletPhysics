/*
 * TestAngles.cpp
 */

#include <gtest/gtest.h>

#include "math/Angles.h"
#include "math/Constants.h"

#include <cmath>

namespace BulletPhysics {
namespace tests {

using namespace math;

class TestAngles : public ::testing::Test {};

// deg2rad

TEST_F(TestAngles, Deg2RadZero)
{
    ASSERT_DOUBLE_EQ(deg2rad(0.0), 0.0);
}

TEST_F(TestAngles, Deg2Rad90)
{
    ASSERT_DOUBLE_EQ(deg2rad(90.0), constants::PI / 2.0);
}

TEST_F(TestAngles, Deg2Rad180)
{
    ASSERT_DOUBLE_EQ(deg2rad(180.0), constants::PI);
}

TEST_F(TestAngles, Deg2Rad360)
{
    ASSERT_DOUBLE_EQ(deg2rad(360.0), 2.0 * constants::PI);
}

TEST_F(TestAngles, Deg2RadNegative)
{
    ASSERT_DOUBLE_EQ(deg2rad(-90.0), -constants::PI / 2.0);
}

TEST_F(TestAngles, Deg2Rad45)
{
    ASSERT_DOUBLE_EQ(deg2rad(45.0), constants::PI / 4.0);
}

// rad2deg

TEST_F(TestAngles, Rad2DegZero)
{
    ASSERT_DOUBLE_EQ(rad2deg(0.0), 0.0);
}

TEST_F(TestAngles, Rad2DegPiHalf)
{
    ASSERT_DOUBLE_EQ(rad2deg(constants::PI / 2.0), 90.0);
}

TEST_F(TestAngles, Rad2DegPi)
{
    ASSERT_DOUBLE_EQ(rad2deg(constants::PI), 180.0);
}

TEST_F(TestAngles, Rad2DegTwoPi)
{
    ASSERT_DOUBLE_EQ(rad2deg(2.0 * constants::PI), 360.0);
}

TEST_F(TestAngles, Rad2DegNegative)
{
    ASSERT_DOUBLE_EQ(rad2deg(-constants::PI / 2.0), -90.0);
}

// roundtrip

TEST_F(TestAngles, RoundtripDegRadDeg)
{
    ASSERT_NEAR(rad2deg(deg2rad(123.456)), 123.456, 1e-12);
}

TEST_F(TestAngles, RoundtripRadDegRad)
{
    ASSERT_NEAR(deg2rad(rad2deg(1.234)), 1.234, 1e-12);
}

} // namespace tests
} // namespace BulletPhysics
