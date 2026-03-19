/*
 * Vec3.h
 */

#pragma once

#include <cmath>

namespace BulletPhysics {
namespace math {

struct Vec3 {
    double x, y, z;

    Vec3();
    Vec3(double X, double Y, double Z);

    Vec3 operator+(const Vec3& rhs) const;
    Vec3 operator-(const Vec3& rhs) const;
    Vec3 operator*(double scalar) const;
    Vec3 operator/(double scalar) const;

    Vec3& operator+=(const Vec3& rhs);
    Vec3& operator-=(const Vec3& rhs);
    Vec3& operator*=(double scalar);

    double length() const;
    Vec3 normalized() const;
    double dot(const Vec3& rhs) const;
    Vec3 cross(const Vec3& rhs) const;
    Vec3 crossEUN(const Vec3& rhs) const;
};

Vec3 operator*(double scalar, const Vec3& vec);

} // namespace math
} // namespace BulletPhysics
