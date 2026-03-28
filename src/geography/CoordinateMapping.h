/*
 * CoordinateMapping.h
 */

#pragma once

#include "math/Vec3.h"

namespace BulletPhysics {
namespace geography {

enum class Axis {
    POS_X, NEG_X,
    POS_Y, NEG_Y,
    POS_Z, NEG_Z
};

// maps between user coordinate system and internal ENU (x=East, y=North, z=Up)
class CoordinateMapping {
public:
    // which user axis corresponds to East, North, Up
    CoordinateMapping(Axis userEast, Axis userNorth, Axis userUp);

    math::Vec3 toInternal(const math::Vec3& v) const;
    math::Vec3 toExternal(const math::Vec3& v) const;

    bool isIdentity() const;

    // active mapping
    static void set(const CoordinateMapping& mapping);
    static const CoordinateMapping& get();

private:
    static CoordinateMapping s_active;
    static double readAxis(const math::Vec3& v, Axis a);
    static void writeAxis(math::Vec3& v, Axis a, double val);

    Axis m_userEast;
    Axis m_userNorth;
    Axis m_userUp;
};

// presets
namespace mappings {

// x=East, y=North, z=Up (native)
inline CoordinateMapping ENU() {
    return {Axis::POS_X, Axis::POS_Y, Axis::POS_Z};
}

// x=East, y=Up, z=-North (right-handed, -Z forward)
inline CoordinateMapping OpenGL() {
    return {Axis::POS_X, Axis::NEG_Z, Axis::POS_Y};
}

// x=East, y=-Up, z=-North (right-handed, Y down)
inline CoordinateMapping Vulkan() {
    return {Axis::POS_X, Axis::NEG_Z, Axis::NEG_Y};
}

// x=East, y=Up, z=-North (right-handed, -Z forward)
inline CoordinateMapping Godot() {
    return {Axis::POS_X, Axis::NEG_Z, Axis::POS_Y};
}

// x=North, y=East, z=Up (left-handed, X forward)
inline CoordinateMapping Unreal() {
    return {Axis::POS_Y, Axis::POS_X, Axis::POS_Z};
}

// x=East, y=Up, z=North (left-handed, Z forward)
inline CoordinateMapping Unity() {
    return {Axis::POS_X, Axis::POS_Z, Axis::POS_Y};
}

} // namespace mappings
} // namespace geography
} // namespace BulletPhysics
