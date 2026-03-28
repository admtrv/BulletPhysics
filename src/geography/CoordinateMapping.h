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

// maps between user coordinate system and internal EUN (x=East, y=Up, z=North)
class CoordinateMapping {
public:
    // which user axis corresponds to East, Up, North
    CoordinateMapping(Axis userEast, Axis userUp, Axis userNorth);

    math::Vec3 toInternal(const math::Vec3& v) const;
    math::Vec3 toExternal(const math::Vec3& v) const;

    bool isIdentity() const;

private:
    static double readAxis(const math::Vec3& v, Axis a);
    static void writeAxis(math::Vec3& v, Axis a, double val);

    Axis m_userEast;
    Axis m_userUp;
    Axis m_userNorth;
};

// presets
namespace mappings {

// x=East, y=Up, z=North (native)
inline CoordinateMapping EUN() {
    return {Axis::POS_X, Axis::POS_Y, Axis::POS_Z};
}

// x=East, y=Up, z=-North (right-handed, -Z forward)
inline CoordinateMapping Godot() {
    return {Axis::POS_X, Axis::POS_Y, Axis::NEG_Z};
}

// x=East, y=Up, z=-North (right-handed, -Z forward)
inline CoordinateMapping OpenGL() {
    return {Axis::POS_X, Axis::POS_Y, Axis::NEG_Z};
}

// x=North, y=East, z=Up (left-handed, X forward)
inline CoordinateMapping Unreal() {
    return {Axis::POS_Y, Axis::POS_Z, Axis::POS_X};
}

} // namespace mappings
} // namespace geography
} // namespace BulletPhysics
