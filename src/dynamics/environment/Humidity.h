/*
 * Humidity.h
 */

#pragma once

#include "Environment.h"
#include "Constants.h"

#include <cmath>

namespace BulletPhysics {
namespace dynamics {
namespace environment {

// provides humidity correction to air density
class Humidity : public IEnvironment {
public:
    explicit Humidity(double relativeHumidity = 50.0)
        : m_relativeHumidity(relativeHumidity)
    {
        m_relativeHumidity = std::max(0.0, std::min(100.0, relativeHumidity));
    }

    void update(IPhysicsBody& /*body*/, PhysicsContext& context) override
    {
        // store relative humidity in context
        context.airHumidity = m_relativeHumidity;

        // density correction requires pressure and temperature from Atmosphere
        if (!context.airPressure.has_value() || !context.airTemperature.has_value() || !context.airDensity.has_value())
        {
            return;
        }

        double temperature = *context.airTemperature;
        double pressure = *context.airPressure;
        double density = *context.airDensity;

        // apply humidity correction
        context.airDensity = correctDensityForHumidity(density, temperature, pressure, m_relativeHumidity);
    }

    const std::string& getName() const override { return m_name; }

private:
    std::string m_name = "Humidity";

    double m_relativeHumidity; // % (0-100)

    // Tetens approximation: p_sat = 0.61078 * exp((17.27 * (T - 273.15)) / (T - 35.85))
    static double saturationVaporPressure(double tempK)
    {
        double tempC = tempK - constants::CELSIUS_TO_KELVIN;
        double exponent = constants::TETENS_A * tempC / (tempK + constants::TETENS_B);
        return constants::TETENS_C * std::exp(exponent);
    }

    // correct air density for humidity
    // rho_humid = rho_dry + rho_vapor
    // where rho_dry = p_dry / (R_dry * T) and rho_vap = p_vap / (R_vap * T)
    static double correctDensityForHumidity(double rhoDry, double tempK, double pressure, double humidityPercent)
    {
        // saturation vapor pressure
        double pressureSaturation = saturationVaporPressure(tempK);

        // water vapor pressure: p_vap = phi * p_sat, where phi is relative humidity
        double pressureVapor = (humidityPercent / 100.0) * pressureSaturation;

        double densityVapor = pressureVapor / (constants::GAS_CONSTANT_WATER_VAPOR * tempK);

        return rhoDry + densityVapor;
    }
};

} // namespace environment
} // namespace dynamics
} // namespace BulletPhysics
