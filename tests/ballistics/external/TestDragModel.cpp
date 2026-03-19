/*
 * TestDragModel.cpp
 */

#include <gtest/gtest.h>

#include "ballistics/external/forces/drag/DragModel.h"

namespace BulletPhysics {
namespace tests {

using namespace ballistics::external::forces::drag;

class TestDragModel : public ::testing::Test {};

// CustomDragModel

TEST_F(TestDragModel, CustomConstantCd)
{
    CustomDragModel model(0.47);

    ASSERT_DOUBLE_EQ(model.getCd(0.0), 0.47);
    ASSERT_DOUBLE_EQ(model.getCd(1.0), 0.47);
    ASSERT_DOUBLE_EQ(model.getCd(5.0), 0.47);
}

// StandardDragModel

TEST_F(TestDragModel, G1AtZeroMach)
{
    StandardDragModel model(DragCurveModel::G1);

    ASSERT_DOUBLE_EQ(model.getCd(0.0), 0.2629);
}

TEST_F(TestDragModel, G1BelowMinMachClamps)
{
    StandardDragModel model(DragCurveModel::G1);

    ASSERT_DOUBLE_EQ(model.getCd(-1.0), 0.2629);
}

TEST_F(TestDragModel, G1AtExactDataPoint)
{
    StandardDragModel model(DragCurveModel::G1);

    ASSERT_DOUBLE_EQ(model.getCd(1.0), 0.4805);
}

TEST_F(TestDragModel, G1InterpolatesBetweenPoints)
{
    StandardDragModel model(DragCurveModel::G1);
    double cd = model.getCd(0.025);

    ASSERT_GT(cd, 0.2558);
    ASSERT_LT(cd, 0.2629);
}

TEST_F(TestDragModel, G1AboveMaxMachClamps)
{
    StandardDragModel model(DragCurveModel::G1);
    double cdMax = model.getCd(100.0);
    double cdHigh = model.getCd(50.0);

    ASSERT_DOUBLE_EQ(cdMax, cdHigh);
}

TEST_F(TestDragModel, G1TransonicRise)
{
    StandardDragModel model(DragCurveModel::G1);
    double cdSubsonic = model.getCd(0.5);
    double cdTransonic = model.getCd(1.0);

    ASSERT_GT(cdTransonic, cdSubsonic);
}

} // namespace tests
} // namespace BulletPhysics
