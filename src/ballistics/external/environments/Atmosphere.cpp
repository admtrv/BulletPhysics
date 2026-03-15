/*
 * Atmosphere.cpp
 */

#include "Atmosphere.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace environments {

// barometric formula exponent: g / (R * L)
const double BAROMETRIC_EXP = constants::GRAVITY.length() / (constants::GAS_CONSTANT_DRY_AIR * constants::ISA_TEMPERATURE_LAPSE_RATE);

Atmosphere::Atmosphere(double baseTemperature, double basePressure, double groundY) : m_baseTemperature(baseTemperature), m_basePressure(basePressure), m_groundY(groundY) {}

void Atmosphere::update(IPhysicsBody& body, PhysicsContext& context)
{
    // calculate altitude above ground level and crop it on troposphere
    double altitude = std::max(0.0, std::min(body.getPosition().y - m_groundY, constants::ISA_TROPOSPHERE_MAX));

    // linear temperature decrease: T = T0 - L * h
    double temperature = m_baseTemperature - constants::ISA_TEMPERATURE_LAPSE_RATE * altitude;

    // barometric formula: p = p0 * (T / T0)^(g / (R * L))
    double pressure = m_basePressure * std::pow(temperature / m_baseTemperature, BAROMETRIC_EXP);

    // ideal gas law: rho = p / (R * T)
    double density = pressure / (constants::GAS_CONSTANT_DRY_AIR * temperature);

    // store atmosphere conditions
    context.airTemperature = temperature;
    context.airPressure = pressure;
    context.airDensity = density;
}

} // namespace environments
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
