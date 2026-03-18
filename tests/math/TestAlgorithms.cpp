/*
 * TestAlgorithms.cpp
 */

#include <gtest/gtest.h>

#include "math/Algorithms.h"

namespace BulletPhysics {
namespace tests {

using math::lerp;

class TestAlgorithms : public ::testing::Test {};

// lerp double

TEST_F(TestAlgorithms, LerpAtStart)
{
    ASSERT_DOUBLE_EQ(lerp(0.0, 10.0, 0.0), 0.0);
}

TEST_F(TestAlgorithms, LerpAtEnd)
{
    ASSERT_DOUBLE_EQ(lerp(0.0, 10.0, 1.0), 10.0);
}

TEST_F(TestAlgorithms, LerpAtMidpoint)
{
    ASSERT_DOUBLE_EQ(lerp(0.0, 10.0, 0.5), 5.0);
}

TEST_F(TestAlgorithms, LerpQuarter)
{
    ASSERT_DOUBLE_EQ(lerp(0.0, 100.0, 0.25), 25.0);
}

TEST_F(TestAlgorithms, LerpNegativeRange)
{
    ASSERT_DOUBLE_EQ(lerp(-10.0, 10.0, 0.5), 0.0);
}

TEST_F(TestAlgorithms, LerpSameValues)
{
    ASSERT_DOUBLE_EQ(lerp(5.0, 5.0, 0.5), 5.0);
}

TEST_F(TestAlgorithms, LerpReversedRange)
{
    ASSERT_DOUBLE_EQ(lerp(10.0, 0.0, 0.25), 7.5);
}

// lerp int

TEST_F(TestAlgorithms, LerpInt)
{
    ASSERT_EQ(lerp(0, 10, 0.5), 5);
}

TEST_F(TestAlgorithms, LerpIntTruncation)
{
    ASSERT_EQ(lerp(0, 10, 0.33), 3);
}

// lerp float

TEST_F(TestAlgorithms, LerpFloat)
{
    ASSERT_FLOAT_EQ(lerp(0.0f, 1.0f, 0.5), 0.5f);
}

} // namespace tests
} // namespace BulletPhysics
