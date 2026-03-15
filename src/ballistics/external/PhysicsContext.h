/*
 * PhysicsContext.h
 */

#pragma once

#include "math/Vec3.h"
#include "Constants.h"


namespace BulletPhysics {
namespace ballistics {
namespace external {

// shared context for physics simulation (environments provide, forces consume)
class PhysicsContext {
public:
    // atmosphere context
    double airDensity = constants::BASE_ATMOSPHERIC_DENSITY;        // kg/m^3
    double airTemperature = constants::BASE_TEMPERATURE;            // K
    double airPressure = constants::BASE_ATMOSPHERIC_PRESSURE;      // Pa
    double airHumidity = constants::DEFAULT_RELATIVE_HUMIDITY;      // % (0-100)
    math::Vec3 wind = constants::DEFAULT_WIND;                      // m/s

    // geographic context
    double latitude = constants::DEFAULT_LATITUDE;          // rad
    double longitude = constants::DEFAULT_LONGITUDE;        // rad
    double altitude = constants::DEFAULT_ALTITUDE;          // m (above sea level)

    math::Vec3 gravity = constants::GRAVITY;        // m/s^2

    void reset()
    {
        airDensity = constants::BASE_ATMOSPHERIC_DENSITY;
        airTemperature = constants::BASE_TEMPERATURE;
        airPressure = constants::BASE_ATMOSPHERIC_PRESSURE;
        airHumidity = constants::DEFAULT_RELATIVE_HUMIDITY;
        wind = constants::DEFAULT_WIND;

        latitude = constants::DEFAULT_LATITUDE;
        longitude = constants::DEFAULT_LONGITUDE;
        altitude = constants::DEFAULT_ALTITUDE;

        gravity = constants::GRAVITY;
    }
};

} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
