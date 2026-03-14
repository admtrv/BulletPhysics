/*
 * Coordinates.h
 */

#pragma once

#include "Constants.h"
#include "math/Vec3.h"

namespace BulletPhysics {
namespace geography {

// Geodetic coordinates
struct GeographicPosition {
    double latitude;   // radians
    double longitude;  // radians
    double altitude;   // meters above

    GeographicPosition() : latitude(0.0), longitude(0.0), altitude(0.0) {}
    GeographicPosition(double lat, double lon, double alt) : latitude(lat), longitude(lon), altitude(alt) {}
};

// ECEF coordinates (Earth-Centered, Earth-Fixed)
struct ECEFPosition {
    double x, y, z;

    ECEFPosition() : x(0.0), y(0.0), z(0.0) {}
    ECEFPosition(double x, double y, double z) : x(x), y(y), z(z) {}

    double r() const;   // distance from Earth's center
};

// geographic coordinate conversions ECEF <-> Geodetic <-> ENU (standard in physics)

ECEFPosition geodeticToECEF(const GeographicPosition& geodetic);                            // convert geodetic (lat, lon, alt) to ECEF (x, y, z)
GeographicPosition ecefToGeodetic(const ECEFPosition& ecef);                                // convert ECEF (x, y, z) to geodetic (lat, lon, alt)
math::Vec3 ecefToENU(const ECEFPosition& point, const GeographicPosition& reference);       // convert ECEF (x, y, z) to local ENU (East-North-Up) relative to reference point
ECEFPosition enuToECEF(const math::Vec3& enu, const GeographicPosition& reference);         // convert local ENU (East-North-Up) to ECEF (x, y, z) relative to reference point

// gravity calculation depend on location

double gravitationalAcceleration(const ECEFPosition& position);                     // calculate gravitational acceleration at ECEF position
double gravitationalAccelerationAtGeodetic(const GeographicPosition& position);     // calculate gravitational acceleration at geodetic position

} // namespace geography
} // namespace BulletPhysics
