/*
 * Vec3.cpp
 */

#include "Vec3.h"
#include <cmath>

namespace BulletPhysics {
namespace math {

Vec3::Vec3() : x(0.0), y(0.0), z(0.0) {}
Vec3::Vec3(double X, double Y, double Z) : x(X), y(Y), z(Z) {}

Vec3 Vec3::operator+(const Vec3& rhs) const
{
    return {x + rhs.x, y + rhs.y, z + rhs.z};
}
Vec3 Vec3::operator-(const Vec3& rhs) const
{
    return {x - rhs.x, y - rhs.y, z - rhs.z};
}
Vec3 Vec3::operator*(double scalar) const
{
    return {x * scalar, y * scalar, z * scalar};
}
Vec3 Vec3::operator/(double scalar) const
{
    return {x / scalar, y / scalar, z / scalar};
}

Vec3& Vec3::operator+=(const Vec3& rhs)
{
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
}
Vec3& Vec3::operator-=(const Vec3& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
}
Vec3& Vec3::operator*=(double scalar)
{
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

Vec3 operator*(double scalar, const Vec3& vec)
{
    return vec * scalar;
}

double Vec3::length() const
{
    return std::sqrt(x * x + y * y + z * z);
}

Vec3 Vec3::normalized() const
{
    double len = length();
    if (len > 0.0001)
    {
        return *this / len;
    }
    return {0.0, 0.0, 0.0};
}

double Vec3::dot(const Vec3& rhs) const
{
    return x * rhs.x + y * rhs.y + z * rhs.z;
}

Vec3 Vec3::cross(const Vec3& rhs) const
{
    return {
        y * rhs.z - z * rhs.y,
        z * rhs.x - x * rhs.z,
        x * rhs.y - y * rhs.x
    };
}

} // namespace math
} // namespace BulletPhysics
