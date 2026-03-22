/*
 * DragModel.cpp
 */

#include "DragModel.h"
#include "PhysicsBody.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace forces {
namespace drag {

double DragCurve::getCd(double mach) const
{
    if (!m_points || m_count == 0)
    {
        return constants::DEFAULT_C_D;
    }

    // clamp to range
    if (mach <= m_points[0].mach)
    {
        return m_points[0].cd;
    }
    if (mach >= m_points[m_count - 1].mach)
    {
        return m_points[m_count - 1].cd;
    }

    // find two surrounding points for linear interpolation
    size_t i = 0;
    while (i < m_count - 1 && m_points[i + 1].mach < mach)
    {
        ++i;
    }

    double t = (mach - m_points[i].mach) / (m_points[i + 1].mach - m_points[i].mach);
    return math::lerp(m_points[i].cd, m_points[i + 1].cd, t);
}

StandardDragModel::StandardDragModel(DragCurveModel model) : m_model(model)
{
    switch (model)
    {
        #define X(name) \
            case DragCurveModel::name: \
                m_curve = {data::name, data::name##_SIZE}; \
                break;
        DRAG_CURVE_MODELS(X)
        #undef X
        default:
            break;
    }
}

double StandardDragModel::getCd(double mach) const
{
    return m_curve.getCd(mach);
}

} // namespace drag
} // namespace forces
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
