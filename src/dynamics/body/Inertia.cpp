/*
 * Inertia.cpp
 */

#include "Inertia.h"

namespace BulletPhysics {
namespace dynamics {
namespace inertia {

// solid box: I = m/12 * diag(y^2+z^2, x^2+z^2, x^2+y^2)
math::Mat3 box(double mass, const math::Vec3& size)
{
    if (mass <= 0.0)
    {
        return math::Mat3::zero();
    }

    const double x2 = size.x * size.x;
    const double y2 = size.y * size.y;
    const double z2 = size.z * size.z;

    const double factor = 12.0 / mass;

    return math::Mat3::diagonal(factor / (y2 + z2), factor / (x2 + z2), factor / (x2 + y2));
}

// solid sphere: I = 2/5 * m * r^2
math::Mat3 sphere(double mass, double radius)
{
    if (mass <= 0.0 || radius <= 0.0)
    {
        return math::Mat3::zero();
    }

    const double inverse = 2.5 / (mass * radius * radius);

    return math::Mat3::diagonal(inverse, inverse, inverse);
}

// solid cylinder: I = m/12 * (3r^2 + h^2) across axis, m/2 * r^2 along it
math::Mat3 cylinder(double mass, double radius, double height)
{
    if (mass <= 0.0 || radius <= 0.0)
    {
        return math::Mat3::zero();
    }

    const double r2 = radius * radius;
    const double across = mass * (3.0 * r2 + height * height) / 12.0;
    const double along = mass * r2 * 0.5;

    return math::Mat3::diagonal(1.0 / across, 1.0 / along, 1.0 / across);
}

} // namespace inertia
} // namespace dynamics
} // namespace BulletPhysics
