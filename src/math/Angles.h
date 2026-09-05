/*
 * Angles.h
 */

#pragma once

#include "Constants.h"

namespace BulletPhysics {
namespace math {

constexpr double PI = 3.14159265358979323846;

inline double deg2rad(double deg)
{
    return deg * (constants::PI / 180.0);
}
inline double rad2deg(double rad)
{
    return rad * (180.0 / constants::PI);
}

} // namespace math
} // namespace BulletPhysics
