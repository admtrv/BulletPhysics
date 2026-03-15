/*
* PhysicsContext.cpp
 */

#include "PhysicsContext.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {

void PhysicsContext::reset()
{
    airDensity = constants::ISA_BASE_DENSITY;
    airTemperature = constants::ISA_BASE_TEMPERATURE;
    airPressure = constants::ISA_BASE_PRESSURE;
    airHumidity = constants::DEFAULT_RELATIVE_HUMIDITY;
    wind = constants::DEFAULT_WIND;

    latitude = constants::DEFAULT_LATITUDE;
    longitude = constants::DEFAULT_LONGITUDE;
    altitude = constants::DEFAULT_ALTITUDE;

    gravity = constants::GRAVITY;
}

} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
