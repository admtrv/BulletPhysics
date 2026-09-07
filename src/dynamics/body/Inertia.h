/*
 * Inertia.h
 */

#pragma once

#include "math/Mat3.h"

namespace BulletPhysics {
namespace dynamics {
namespace inertia {

// inverse inertia tensors in body space

math::Mat3 box(double mass, const math::Vec3& size);
math::Mat3 sphere(double mass, double radius);

} // namespace inertia
} // namespace dynamics
} // namespace BulletPhysics
