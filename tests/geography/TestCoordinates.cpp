/*
 * TestCoordinates.cpp
 */

#include <gtest/gtest.h>

#include "geography/Coordinates.h"
#include "math/Angles.h"

#include <cmath>

namespace BulletPhysics {
namespace tests {

using namespace geography;

class TestCoordinates : public ::testing::Test {
protected:
    GeographicPosition equator{0.0, 0.0, 0.0};
    GeographicPosition northPole{math::deg2rad(90.0), 0.0, 0.0};
};

// helpers

void AssertGeodeticNear(const GeographicPosition& a, const GeographicPosition& b, double eps = 1e-6)
{
    ASSERT_NEAR(a.latitude, b.latitude, eps);
    ASSERT_NEAR(a.longitude, b.longitude, eps);
    ASSERT_NEAR(a.altitude, b.altitude, 0.01);
}

void AssertECEFNear(const ECEFPosition& a, const ECEFPosition& b, double eps = 0.01)
{
    ASSERT_NEAR(a.x, b.x, eps);
    ASSERT_NEAR(a.y, b.y, eps);
    ASSERT_NEAR(a.z, b.z, eps);
}

// geodetic to ECEF, known points

TEST_F(TestCoordinates, EquatorToECEF)
{
    ECEFPosition ecef = geodeticToECEF(equator);

    ASSERT_NEAR(ecef.x, constants::WGS84_EARTH_SEMI_MAJOR_AXIS, 0.01);
    ASSERT_NEAR(ecef.y, 0.0, 0.01);
    ASSERT_NEAR(ecef.z, 0.0, 0.01);
}

TEST_F(TestCoordinates, PoleToECEF)
{
    ECEFPosition ecef = geodeticToECEF(northPole);

    ASSERT_NEAR(ecef.x, 0.0, 0.01);
    ASSERT_NEAR(ecef.y, 0.0, 0.01);
    ASSERT_NEAR(ecef.z, constants::WGS84_EARTH_SEMI_MINOR_AXIS, 1.0);
}

// geodetic -> ECEF -> geodetic roundtrip

TEST_F(TestCoordinates, RoundtripEquator)
{
    ECEFPosition ecef = geodeticToECEF(equator);
    GeographicPosition result = ecefToGeodetic(ecef);

    AssertGeodeticNear(result, equator);
}

TEST_F(TestCoordinates, RoundtripPole)
{
    ECEFPosition ecef = geodeticToECEF(northPole);
    GeographicPosition result = ecefToGeodetic(ecef);

    AssertGeodeticNear(result, northPole);
}

TEST_F(TestCoordinates, RoundtripAltitude)
{
    GeographicPosition high{math::deg2rad(45.0), math::deg2rad(90.0), 10000.0};
    ECEFPosition ecef = geodeticToECEF(high);
    GeographicPosition result = ecefToGeodetic(ecef);

    AssertGeodeticNear(result, high);
}

// ECEF -> ENU -> ECEF roundtrip

TEST_F(TestCoordinates, ENURoundtripEquator)
{
    GeographicPosition offset{0.0, math::deg2rad(0.001), 0.0};
    ECEFPosition point = geodeticToECEF(offset);

    math::Vec3 enu = ecefToENU(point, equator);
    ECEFPosition back = enuToECEF(enu, equator);

    AssertECEFNear(back, point);
}

TEST_F(TestCoordinates, ENUOriginZero)
{
    ECEFPosition origin = geodeticToECEF(equator);
    math::Vec3 enu = ecefToENU(origin, equator);

    ASSERT_NEAR(enu.x, 0.0, 1e-6);
    ASSERT_NEAR(enu.y, 0.0, 1e-6);
    ASSERT_NEAR(enu.z, 0.0, 1e-6);
}

// gravitational acceleration

TEST_F(TestCoordinates, GravityEquator)
{
    double g = gravitationalAccelerationAtGeodetic(equator);

    ASSERT_NEAR(g, 9.78, 0.05);
}

TEST_F(TestCoordinates, GravityPole)
{
    double g = gravitationalAccelerationAtGeodetic(northPole);

    ASSERT_NEAR(g, 9.83, 0.05);
}

TEST_F(TestCoordinates, GravityPoleGreaterEquator)
{
    double gEquator = gravitationalAccelerationAtGeodetic(equator);
    double gPole = gravitationalAccelerationAtGeodetic(northPole);

    ASSERT_GT(gPole, gEquator);
}

TEST_F(TestCoordinates, GravityAltitudeDecreases)
{
    double gSurface = gravitationalAccelerationAtGeodetic(equator);

    GeographicPosition high{0.0, 0.0, 100000.0};
    double gHigh = gravitationalAccelerationAtGeodetic(high);

    ASSERT_GT(gSurface, gHigh);
}

} // namespace tests
} // namespace BulletPhysics
