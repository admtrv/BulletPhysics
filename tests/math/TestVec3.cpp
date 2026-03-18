/*
 * TestVec3.cpp
 */

#include <gtest/gtest.h>

#include "math/Vec3.h"

#include <cmath>

namespace BulletPhysics {
namespace tests {

using math::Vec3;

class TestVec3 : public ::testing::Test {
protected:
    Vec3 zero;
    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{4.0, 5.0, 6.0};
    Vec3 unitX{1.0, 0.0, 0.0};
    Vec3 unitY{0.0, 1.0, 0.0};
    Vec3 unitZ{0.0, 0.0, 1.0};
};

// helpers

void AssertEq(const Vec3& v, double x, double y, double z)
{
    ASSERT_DOUBLE_EQ(v.x, x);
    ASSERT_DOUBLE_EQ(v.y, y);
    ASSERT_DOUBLE_EQ(v.z, z);
}

void AssertNear(const Vec3& v, double x, double y, double z, double eps = 1e-12)
{
    ASSERT_NEAR(v.x, x, eps);
    ASSERT_NEAR(v.y, y, eps);
    ASSERT_NEAR(v.z, z, eps);
}

// construction

TEST_F(TestVec3, DefaultConstructor)
{
    AssertEq(zero, 0.0, 0.0, 0.0);
}

TEST_F(TestVec3, ParameterizedConstructor)
{
    AssertEq(a, 1.0, 2.0, 3.0);
}

TEST_F(TestVec3, NegativeComponents)
{
    AssertEq(Vec3(-1.5, -2.5, -3.5), -1.5, -2.5, -3.5);
}

// addition

TEST_F(TestVec3, Addition)
{
    AssertEq(a + b, 5.0, 7.0, 9.0);
}

TEST_F(TestVec3, AdditionWithNegative)
{
    AssertEq(a + Vec3(-1.0, -2.0, -3.0), 0.0, 0.0, 0.0);
}

TEST_F(TestVec3, AdditionWithZero)
{
    AssertEq(a + zero, 1.0, 2.0, 3.0);
}

TEST_F(TestVec3, AddAssign)
{
    a += b;
    AssertEq(a, 5.0, 7.0, 9.0);
}

// subtraction

TEST_F(TestVec3, Subtraction)
{
    AssertEq(b - a, 3.0, 3.0, 3.0);
}

TEST_F(TestVec3, SubtractSelf)
{
    AssertEq(a - a, 0.0, 0.0, 0.0);
}

TEST_F(TestVec3, SubAssign)
{
    b -= a;
    AssertEq(b, 3.0, 3.0, 3.0);
}

// scalar multiplication

TEST_F(TestVec3, ScalarMultiply)
{
    AssertEq(a * 2.0, 2.0, 4.0, 6.0);
}

TEST_F(TestVec3, ScalarMultiplyLeft)
{
    AssertEq(2.0 * a, 2.0, 4.0, 6.0);
}

TEST_F(TestVec3, ScalarMultiplyZero)
{
    AssertEq(a * 0.0, 0.0, 0.0, 0.0);
}

TEST_F(TestVec3, ScalarMultiplyNegative)
{
    AssertEq(a * -1.0, -1.0, -2.0, -3.0);
}

TEST_F(TestVec3, MulAssign)
{
    a *= 3.0;
    AssertEq(a, 3.0, 6.0, 9.0);
}

// scalar division

TEST_F(TestVec3, ScalarDivide)
{
    AssertEq(Vec3(2.0, 4.0, 6.0) / 2.0, 1.0, 2.0, 3.0);
}

TEST_F(TestVec3, ScalarDivideByOne)
{
    AssertEq(a / 1.0, 1.0, 2.0, 3.0);
}

TEST_F(TestVec3, ScalarDivideNegative)
{
    AssertEq(Vec3(2.0, 4.0, 6.0) / -2.0, -1.0, -2.0, -3.0);
}

TEST_F(TestVec3, DivideByZeroProducesInf)
{
    Vec3 c = a / 0.0;
    ASSERT_TRUE(std::isinf(c.x));
    ASSERT_TRUE(std::isinf(c.y));
    ASSERT_TRUE(std::isinf(c.z));
}

// length

TEST_F(TestVec3, LengthUnitVectors)
{
    ASSERT_DOUBLE_EQ(unitX.length(), 1.0);
    ASSERT_DOUBLE_EQ(unitY.length(), 1.0);
    ASSERT_DOUBLE_EQ(unitZ.length(), 1.0);
}

TEST_F(TestVec3, LengthZero)
{
    ASSERT_DOUBLE_EQ(zero.length(), 0.0);
}

TEST_F(TestVec3, Length345)
{
    ASSERT_DOUBLE_EQ(Vec3(3.0, 4.0, 0.0).length(), 5.0);
}

TEST_F(TestVec3, LengthGeneral)
{
    ASSERT_DOUBLE_EQ(a.length(), std::sqrt(14.0));
}

// normalize

TEST_F(TestVec3, NormalizeAlongAxis)
{
    AssertEq(Vec3(5.0, 0.0, 0.0).normalized(), 1.0, 0.0, 0.0);
}

TEST_F(TestVec3, NormalizeGeneral)
{
    Vec3 n = Vec3(3.0, 4.0, 0.0).normalized();
    ASSERT_DOUBLE_EQ(n.length(), 1.0);
    AssertEq(n, 0.6, 0.8, 0.0);
}

TEST_F(TestVec3, NormalizeZeroVector)
{
    AssertEq(zero.normalized(), 0.0, 0.0, 0.0);
}

TEST_F(TestVec3, NormalizeNearZeroVector)
{
    AssertEq(Vec3(1e-5, 0.0, 0.0).normalized(), 0.0, 0.0, 0.0);
}

TEST_F(TestVec3, NormalizePreservesDirection)
{
    double len = std::sqrt(14.0);
    AssertNear(a.normalized(), 1.0 / len, 2.0 / len, 3.0 / len);
}

// dot product

TEST_F(TestVec3, DotProduct)
{
    ASSERT_DOUBLE_EQ(a.dot(b), 32.0);
}

TEST_F(TestVec3, DotProductOrthogonal)
{
    ASSERT_DOUBLE_EQ(unitX.dot(unitY), 0.0);
    ASSERT_DOUBLE_EQ(unitY.dot(unitZ), 0.0);
    ASSERT_DOUBLE_EQ(unitX.dot(unitZ), 0.0);
}

TEST_F(TestVec3, DotProductParallel)
{
    ASSERT_DOUBLE_EQ(unitX.dot(Vec3(3.0, 0.0, 0.0)), 3.0);
}

TEST_F(TestVec3, DotProductAntiparallel)
{
    ASSERT_DOUBLE_EQ(unitX.dot(Vec3(-1.0, 0.0, 0.0)), -1.0);
}

TEST_F(TestVec3, DotProductWithZero)
{
    ASSERT_DOUBLE_EQ(a.dot(zero), 0.0);
}

TEST_F(TestVec3, DotProductCommutative)
{
    ASSERT_DOUBLE_EQ(a.dot(b), b.dot(a));
}

// cross product

TEST_F(TestVec3, CrossProductBasis)
{
    AssertEq(unitX.cross(unitY), 0.0, 0.0, 1.0);
}

TEST_F(TestVec3, CrossProductAnticommutative)
{
    Vec3 ab = a.cross(b);
    Vec3 ba = b.cross(a);
    AssertEq(ab, -ba.x, -ba.y, -ba.z);
}

TEST_F(TestVec3, CrossProductSelfIsZero)
{
    AssertEq(a.cross(a), 0.0, 0.0, 0.0);
}

TEST_F(TestVec3, CrossProductParallelIsZero)
{
    AssertNear(a.cross(a * 5.0), 0.0, 0.0, 0.0);
}

TEST_F(TestVec3, CrossProductOrthogonalToInputs)
{
    Vec3 c = a.cross(b);
    ASSERT_NEAR(c.dot(a), 0.0, 1e-12);
    ASSERT_NEAR(c.dot(b), 0.0, 1e-12);
}

TEST_F(TestVec3, CrossProductGeneral)
{
    AssertEq(a.cross(b), -3.0, 6.0, -3.0);
}

TEST_F(TestVec3, CrossProductWithZero)
{
    AssertEq(a.cross(zero), 0.0, 0.0, 0.0);
}

} // namespace tests
} // namespace BulletPhysics
