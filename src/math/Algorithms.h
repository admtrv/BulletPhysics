/*
 * Algorithms.h
 */

#pragma once

namespace BulletPhysics {
namespace math {

// linear interpolation: y = y1 + t * (y2 - y1), where t in [0, 1]
template<typename T>
T lerp(T y1, T y2, double t)
{
    return y1 + static_cast<T>(t * (y2 - y1));
}

} // namespace math
} // namespace BulletPhysics
