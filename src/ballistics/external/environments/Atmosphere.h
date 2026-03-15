/*
 * Atmosphere.h
 */

#pragma once

#include "Environment.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace environments {

// ISA model for Troposphere: provides atmospheric properties based on altitude
class Atmosphere : public IEnvironment {
public:
    Atmosphere(double baseTemperature = constants::ISA_BASE_TEMPERATURE, double basePressure = constants::ISA_BASE_PRESSURE, double groundY = 0.0);

    void update(IPhysicsBody& body, PhysicsContext& context) override;

    // getters
    const std::string& getName() const override { return m_name; }
    int getPriority() const override { return m_priority; }

private:
    std::string m_name = "Atmosphere";
    int m_priority = 0;

    double m_baseTemperature;      // K
    double m_basePressure;         // Pa
    double m_groundY;
};

} // namespace environments
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
