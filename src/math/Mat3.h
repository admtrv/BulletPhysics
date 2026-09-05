/*
 * Mat3.h
 */

#pragma once

#include "Vec3.h"

namespace BulletPhysics {
namespace math {

// 3x3 matrix, row major
struct Mat3 {
    Vec3 rows[3];

    Mat3();                                                     // identity
    Mat3(const Vec3& row0, const Vec3& row1, const Vec3& row2);

    static Mat3 diagonal(double x, double y, double z);
    static Mat3 zero();

    Mat3 operator*(const Mat3& rhs) const;
    Vec3 operator*(const Vec3& v) const;
    Mat3 operator*(double scalar) const;

    Mat3 transposed() const;
};

} // namespace math
} // namespace BulletPhysics
