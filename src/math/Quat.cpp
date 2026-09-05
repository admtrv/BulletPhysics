/*
 * Quat.cpp
 */

#include "Quat.h"

#include <cmath>

namespace BulletPhysics {
namespace math {

Quat::Quat() : w(1.0), x(0.0), y(0.0), z(0.0) {}

Quat::Quat(double W, double X, double Y, double Z) : w(W), x(X), y(Y), z(Z) {}

Quat Quat::fromAxisAngle(const Vec3& axis, double angleRad)
{
    const double length = axis.length();
    if (length < 1e-12)
    {
        return {};
    }

    const double half = angleRad * 0.5;
    const double s = std::sin(half) / length;

    return {std::cos(half), axis.x * s, axis.y * s, axis.z * s};
}

Quat Quat::operator*(const Quat& rhs) const
{
    return {
        w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
        w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
        w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
        w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
    };
}

Quat Quat::operator*(double scalar) const
{
    return {w * scalar, x * scalar, y * scalar, z * scalar};
}

Quat Quat::operator+(const Quat& rhs) const
{
    return {w + rhs.w, x + rhs.x, y + rhs.y, z + rhs.z};
}

Quat& Quat::operator+=(const Quat& rhs)
{
    w += rhs.w;
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;

    return *this;
}

void Quat::normalize()
{
    const double length = std::sqrt(w * w + x * x + y * y + z * z);
    if (length < 1e-12)
    {
        *this = Quat{};
        return;
    }

    const double inv = 1.0 / length;

    w *= inv;
    x *= inv;
    y *= inv;
    z *= inv;
}

Quat Quat::normalized() const
{
    Quat result = *this;
    result.normalize();
    return result;
}

Quat Quat::conjugated() const
{
    return {w, -x, -y, -z};
}

Vec3 Quat::rotate(const Vec3& v) const
{
    // v' = v + 2w(q x v) + 2(q x (q x v)), cheaper than building the matrix
    const Vec3 q{x, y, z};
    const Vec3 t = q.cross(v) * 2.0;

    return v + t * w + q.cross(t);
}

Mat3 Quat::toMat3() const
{
    const double xx = x * x, yy = y * y, zz = z * z;
    const double xy = x * y, xz = x * z, yz = y * z;
    const double wx = w * x, wy = w * y, wz = w * z;

    return {
        {1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)},
        {2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)},
        {2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)}
    };
}

} // namespace math
} // namespace BulletPhysics
