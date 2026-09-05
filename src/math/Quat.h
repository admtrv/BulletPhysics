/*
 * Quat.h
 */

#pragma once

#include "Mat3.h"
#include "Vec3.h"

namespace BulletPhysics {
namespace math {

// unit quaternion, w is scalar part
struct Quat {
    double w, x, y, z;

    Quat();                                             // identity
    Quat(double W, double X, double Y, double Z);

    static Quat fromAxisAngle(const Vec3& axis, double angleRad);

    Quat operator*(const Quat& rhs) const;
    Quat operator*(double scalar) const;
    Quat operator+(const Quat& rhs) const;
    Quat& operator+=(const Quat& rhs);

    void normalize();
    Quat normalized() const;

    Quat conjugated() const;

    Vec3 rotate(const Vec3& v) const;

    Mat3 toMat3() const;
};

} // namespace math
} // namespace BulletPhysics
