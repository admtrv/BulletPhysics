/*
 * Mat3.cpp
 */

#include "Mat3.h"

namespace BulletPhysics {
namespace math {

Mat3::Mat3() : rows{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}} {}

Mat3::Mat3(const Vec3& row0, const Vec3& row1, const Vec3& row2) : rows{row0, row1, row2} {}

Mat3 Mat3::diagonal(double x, double y, double z)
{
    return {{x, 0.0, 0.0}, {0.0, y, 0.0}, {0.0, 0.0, z}};
}

Mat3 Mat3::zero()
{
    return {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
}

Mat3 Mat3::operator*(const Mat3& rhs) const
{
    const Mat3 t = rhs.transposed();

    return {
        {rows[0].dot(t.rows[0]), rows[0].dot(t.rows[1]), rows[0].dot(t.rows[2])},
        {rows[1].dot(t.rows[0]), rows[1].dot(t.rows[1]), rows[1].dot(t.rows[2])},
        {rows[2].dot(t.rows[0]), rows[2].dot(t.rows[1]), rows[2].dot(t.rows[2])}
    };
}

Vec3 Mat3::operator*(const Vec3& v) const
{
    return {rows[0].dot(v), rows[1].dot(v), rows[2].dot(v)};
}

Mat3 Mat3::operator*(double scalar) const
{
    return {rows[0] * scalar, rows[1] * scalar, rows[2] * scalar};
}

Mat3 Mat3::transposed() const
{
    return {
        {rows[0].x, rows[1].x, rows[2].x},
        {rows[0].y, rows[1].y, rows[2].y},
        {rows[0].z, rows[1].z, rows[2].z}
    };
}

} // namespace math
} // namespace BulletPhysics
