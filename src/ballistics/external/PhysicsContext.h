/*
 * PhysicsContext.h
 */

#pragma once

#include "math/Vec3.h"

#include <optional>

namespace BulletPhysics {
namespace ballistics {
namespace external {

// shared context for physics simulation (environments provide, forces consume)
class PhysicsContext {
public:
    // atmosphere context
    std::optional<double> airDensity;        // kg/m^3
    std::optional<double> airTemperature;    // K
    std::optional<double> airPressure;       // Pa
    std::optional<double> airHumidity;       // % (relative humidity 0-100)
    std::optional<math::Vec3> wind;          // m/s

    // geographic context
    std::optional<double> latitude;         // rad
    std::optional<double> longitude;        // rad
    std::optional<double> altitude;         // m (above sea level)

    std::optional<double> gravity;          // m/s^2 (gravity acceleration magnitude)

    void reset()
    {
        airDensity.reset();
        airTemperature.reset();
        airPressure.reset();
        airHumidity.reset();
        wind.reset();

        latitude.reset();
        longitude.reset();
        altitude.reset();

        gravity.reset();
    }
};

} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
