/*
 * DragModel.cpp
 */

#include "DragModel.h"

namespace BulletPhysics {
namespace dynamics {
namespace forces {
namespace drag {

double DragCurve::getCd(double mach) const
{
    if (!m_points || m_count == 0)
    {
        return constants::DEFAULT_CD;
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
        case DragCurveModel::G1:
            m_curve = {data::G1, data::G1_SIZE};
            break;
        case DragCurveModel::G2:
            m_curve = {data::G2, data::G2_SIZE};
            break;
        case DragCurveModel::G5:
            m_curve = {data::G5, data::G5_SIZE};
            break;
        case DragCurveModel::G6:
            m_curve = {data::G6, data::G6_SIZE};
            break;
        case DragCurveModel::G7:
            m_curve = {data::G7, data::G7_SIZE};
            break;
        case DragCurveModel::G8:
            m_curve = {data::G8, data::G8_SIZE};
            break;
        case DragCurveModel::GL:
            m_curve = {data::GL, data::GL_SIZE};
            break;
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
} // namespace dynamics
} // namespace BulletPhysics
