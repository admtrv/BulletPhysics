/*
 * CoordinateMapping.cpp
 */

#include "CoordinateMapping.h"

namespace BulletPhysics {
namespace geography {

CoordinateMapping::CoordinateMapping(Axis userEast, Axis userUp, Axis userNorth)
    : m_userEast(userEast), m_userUp(userUp), m_userNorth(userNorth) {}

double CoordinateMapping::readAxis(const math::Vec3& v, Axis a)
{
    switch (a)
    {
        case Axis::POS_X:
            return v.x;
        case Axis::NEG_X:
            return -v.x;
        case Axis::POS_Y:
            return v.y;
        case Axis::NEG_Y:
            return -v.y;
        case Axis::POS_Z:
            return v.z;
        case Axis::NEG_Z:
            return -v.z;
    }
    return 0.0;
}

void CoordinateMapping::writeAxis(math::Vec3& v, Axis a, double val)
{
    switch (a)
    {
        case Axis::POS_X:
            v.x = val;
            break;
        case Axis::NEG_X:
            v.x = -val;
            break;
        case Axis::POS_Y:
            v.y = val;
            break;
        case Axis::NEG_Y:
            v.y = -val;
            break;
        case Axis::POS_Z:
            v.z = val;
            break;
        case Axis::NEG_Z:
            v.z = -val;
            break;
    }
}

math::Vec3 CoordinateMapping::toInternal(const math::Vec3& v) const
{
    return {readAxis(v, m_userEast), readAxis(v, m_userUp), readAxis(v, m_userNorth)};
}

math::Vec3 CoordinateMapping::toExternal(const math::Vec3& v) const
{
    // v is EUN: v.x=East, v.y=Up, v.z=North
    // scatter into user axes
    math::Vec3 out{0, 0, 0};
    writeAxis(out, m_userEast, v.x);
    writeAxis(out, m_userUp, v.y);
    writeAxis(out, m_userNorth, v.z);
    return out;
}

bool CoordinateMapping::isIdentity() const
{
    return m_userEast == Axis::POS_X && m_userUp == Axis::POS_Y && m_userNorth == Axis::POS_Z;
}

} // namespace geography
} // namespace BulletPhysics
