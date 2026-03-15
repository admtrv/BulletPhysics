/*
 * Atmosphere.h
 */

#pragma once

#include "Environment.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace environments {

// provides atmospheric properties based on altitude (ISA model for Troposphere)
class Atmosphere : public IEnvironment {
public:
    explicit Atmosphere(
        double baseTemperature = constants::BASE_TEMPERATURE,
        double basePressure = constants::BASE_ATMOSPHERIC_PRESSURE,
        double groundY = 0.0)
            : m_baseTemperature(baseTemperature)
            , m_basePressure(basePressure)
            , m_groundY(groundY)
    {}

    void update(IPhysicsBody& body, PhysicsContext& context) override
    {
        double altitude = std::max(0.0, std::min(body.getPosition().y - m_groundY, constants::TROPOSPHERE_MAX));

        // linear temperature decrease: T = T0 - L * h
        double temperature = m_baseTemperature - constants::LAPSE_RATE * altitude;

        // barometric formula: p = p0 * (T / T0)^(g / (R * L))
        double pressure = m_basePressure * std::pow(temperature / m_baseTemperature, BAROMETRIC_EXP);

        // ideal gas law: rho = p / (R * T)
        double density = pressure / (constants::GAS_CONSTANT_DRY_AIR * temperature);

        context.airTemperature = temperature;
        context.airPressure = pressure;
        context.airDensity = density;
    }

    const std::string& getName() const override { return m_name; }

private:
    std::string m_name = "Atmosphere";

    // barometric formula exponent: g / (R * L)
    static inline const double BAROMETRIC_EXP = constants::GRAVITY.length() / (constants::GAS_CONSTANT_DRY_AIR * constants::LAPSE_RATE);

    double m_baseTemperature;      // K
    double m_basePressure;         // Pa
    double m_groundY;
};

} // namespace environments
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
