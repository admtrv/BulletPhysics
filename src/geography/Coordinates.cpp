/*
 * Coordinates.cpp
 */

#include "Coordinates.h"
#include <cmath>

namespace BulletPhysics {
namespace geography {

double ECEFPosition::r() const
{
    return std::sqrt(x * x + y * y + z * z);
}

ECEFPosition geodeticToECEF(const GeographicPosition& geodetic)
{
    double lat = geodetic.latitude;
    double lon = geodetic.longitude;
    double alt = geodetic.altitude;

    double sinLat = std::sin(lat);
    double cosLat = std::cos(lat);
    double sinLon = std::sin(lon);
    double cosLon = std::cos(lon);

    // radius of curvature in prime vertical
    double N = constants::EARTH_SEMI_MAJOR_AXIS / std::sqrt(1.0 - constants::EARTH_ECCENTRICITY_SQUARED * sinLat * sinLat);

    ECEFPosition ecef;
    ecef.x = (N + alt) * cosLat * cosLon;
    ecef.y = (N + alt) * cosLat * sinLon;
    ecef.z = (N * (1.0 - constants::EARTH_ECCENTRICITY_SQUARED) + alt) * sinLat;

    return ecef;
}

GeographicPosition ecefToGeodetic(const ECEFPosition& ecef)
{
    double x = ecef.x;
    double y = ecef.y;
    double z = ecef.z;

    double lon = std::atan2(y, x);

    double p = std::sqrt(x * x + y * y);
    double lat = std::atan2(z, p * (1.0 - constants::EARTH_ECCENTRICITY_SQUARED));

    // iterate to improve latitude accuracy
    for (int i = 0; i < 5; i++)
    {
        double sinLat = std::sin(lat);
        double N = constants::EARTH_SEMI_MAJOR_AXIS / std::sqrt(1.0 - constants::EARTH_ECCENTRICITY_SQUARED * sinLat * sinLat);
        lat = std::atan2(z + constants::EARTH_ECCENTRICITY_SQUARED * N * sinLat, p);
    }

    double sinLat = std::sin(lat);
    double cosLat = std::cos(lat);
    double N = constants::EARTH_SEMI_MAJOR_AXIS / std::sqrt(1.0 - constants::EARTH_ECCENTRICITY_SQUARED * sinLat * sinLat);
    double alt = p / cosLat - N;

    return GeographicPosition(lat, lon, alt);
}

math::Vec3 ecefToENU(const ECEFPosition& point, const GeographicPosition& reference)
{
    ECEFPosition refEcef = geodeticToECEF(reference);

    double dx = point.x - refEcef.x;
    double dy = point.y - refEcef.y;
    double dz = point.z - refEcef.z;

    double sinLat = std::sin(reference.latitude);
    double cosLat = std::cos(reference.latitude);
    double sinLon = std::sin(reference.longitude);
    double cosLon = std::cos(reference.longitude);

    // Rotation matrix from ECEF to ENU
    double east = -sinLon * dx + cosLon * dy;
    double north = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz;
    double up = cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz;

    return math::Vec3(east, up, north);
}

ECEFPosition enuToECEF(const math::Vec3& enu, const GeographicPosition& reference)
{
    double east = enu.x;
    double up = enu.y;
    double north = enu.z;

    double sinLat = std::sin(reference.latitude);
    double cosLat = std::cos(reference.latitude);
    double sinLon = std::sin(reference.longitude);
    double cosLon = std::cos(reference.longitude);

    // inverse rotation matrix from ENU to ECEF
    double dx = -sinLon * east - sinLat * cosLon * north + cosLat * cosLon * up;
    double dy = cosLon * east - sinLat * sinLon * north + cosLat * sinLon * up;
    double dz = cosLat * north + sinLat * up;

    ECEFPosition refEcef = geodeticToECEF(reference);
    return ECEFPosition(refEcef.x + dx, refEcef.y + dy, refEcef.z + dz);
}

double gravitationalAcceleration(const ECEFPosition& position)
{
    double r = position.r();
    return constants::EARTH_GRAVITATIONAL_CONSTANT / (r * r);
}

double gravitationalAccelerationAtGeodetic(const GeographicPosition& position)
{
    ECEFPosition ecef = geodeticToECEF(position);
    return gravitationalAcceleration(ecef);
}

} // namespace geography
} // namespace BulletPhysics
