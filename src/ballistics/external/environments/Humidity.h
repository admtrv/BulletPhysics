/*
 * Humidity.h
 */

#pragma once

#include "Environment.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace environments {

// provides humidity correction to air density
class Humidity : public IEnvironment {
public:
    explicit Humidity(double relativeHumidity = constants::DEFAULT_RELATIVE_HUMIDITY);

    void update(IPhysicsBody& /*body*/, PhysicsContext& context) override;

    // getters
    const std::string& getName() const override { return m_name; }
    int getPriority() const override { return m_priority; }

private:
    std::string m_name = "Humidity";
    int m_priority = 10;

    double m_relativeHumidity; // % (0-100)
};

} // namespace environments
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
