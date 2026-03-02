/*
 * DragModel.h
 */

#pragma once

#include <cmath>
#include <algorithm>

#include "Constants.h"
#include "DragData.h"
#include "math/Algorithms.h"

namespace BulletPhysics {
namespace dynamics {
namespace forces {
namespace drag {

enum class DragCurveModel {
    G1,
    G2,
    G5,
    G6,
    G7,
    G8,
    GL,
    CUSTOM
};

// represents a single drag curve
class DragCurve {
public:
    DragCurve() = default;
    DragCurve(const DragPoint* points, size_t count) : m_points(points), m_count(count) {}

    // get Cd for given mach number (linear interpolation)
    double getCd(double mach) const;

private:
    const DragPoint* m_points = nullptr;
    size_t m_count = 0;
};

// interface for drag model selection
class IDragModel {
public:
    virtual ~IDragModel() = default;
    virtual double getCd(double mach) const = 0;
};

// standard G1-G8, GL curves
class StandardDragModel : public IDragModel {
public:
    explicit StandardDragModel(DragCurveModel model);

    double getCd(double mach) const override;
    DragCurveModel getModel() const { return m_model; }

private:
    DragCurveModel m_model;
    DragCurve m_curve;
};

// custom constant Cd
class CustomDragModel : public IDragModel {
public:
    explicit CustomDragModel(double cd) : m_cd(cd) {}

    double getCd(double mach) const override { return m_cd; }
    double getCd() const { return m_cd; }

private:
    double m_cd;
};

} // namespace drag
} // namespace forces
} // namespace dynamics
} // namespace BulletPhysics
