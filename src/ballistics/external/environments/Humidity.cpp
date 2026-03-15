/*
 * Humidity.cpp
 */

#include "Humidity.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace environments {

Humidity::Humidity(double relativeHumidity) : m_relativeHumidity(relativeHumidity) {
    m_relativeHumidity = std::max(0.0, std::min(100.0, relativeHumidity)); // crop on percentage
}

// Tetens approximation: p_sat = 0.61078 * exp((17.27 * (T - 273.15)) / (T - 35.85))
static double saturationVaporPressure(double tempK)
{
    double tempC = tempK - constants::CELSIUS_TO_KELVIN;
    double exponent = constants::TETENS_A * tempC / (tempK + constants::TETENS_B);
    return constants::TETENS_C * std::exp(exponent);
}

// correct air density for humidity
// rho_hum = p_dry / (R_dry * T) + p_vap / (R_vap * T)
// where p_dry = p - p_vap
static double correctDensityForHumidity(double tempK, double pressure, double humidityPercent)
{
    // saturation vapor pressure
    double pressureSaturation = saturationVaporPressure(tempK);

    // water vapor pressure: p_vap = phi * p_sat, where phi is relative humidity
    double pressureVapor = (humidityPercent / 100.0) * pressureSaturation;

    // partial pressure of dry air: p_dry = p - p_vap
    double pressureDry = pressure - pressureVapor;

    // rho = p_dry / (R_dry * T) + p_vap / (R_vap * T)
    return pressureDry / (constants::GAS_CONSTANT_DRY_AIR * tempK) + pressureVapor / (constants::GAS_CONSTANT_WATER_VAPOR * tempK);
}

void Humidity::update(IPhysicsBody& /*body*/, PhysicsContext& context)
{
    // store relative humidity
    context.airHumidity = m_relativeHumidity;

    // apply humidity correction
    context.airDensity = correctDensityForHumidity(context.airTemperature, context.airPressure, m_relativeHumidity);
}


} // namespace environments
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
