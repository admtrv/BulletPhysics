/*
 * PhysicsContext.h
 */

#pragma once

#include "Constants.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {

// shared context for physics simulation (environments provide, forces consume)
class PhysicsContext {
public:
    // reset all to initial state
    void reset();

    // atmosphere context
    double airDensity = constants::ISA_BASE_DENSITY;           // kg/m^3
    double airTemperature = constants::ISA_BASE_TEMPERATURE;   // K
    double airPressure = constants::ISA_BASE_PRESSURE;         // Pa
    double airHumidity = constants::DEFAULT_RELATIVE_HUMIDITY; // % (0-100)
    math::Vec3 wind = constants::DEFAULT_WIND;                 // m/s

    // geographic context
    double latitude = constants::DEFAULT_LATITUDE;       // rad
    double longitude = constants::DEFAULT_LONGITUDE;     // rad
    double altitude = constants::DEFAULT_ALTITUDE;       // m (above sea level)

    math::Vec3 gravity = constants::GRAVITY;    // m/s^2

};

} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
