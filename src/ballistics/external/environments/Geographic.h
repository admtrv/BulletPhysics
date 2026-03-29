/*
 * Geographic.h
 */

#pragma once

#include <algorithm>

#include "Environment.h"
#include "geography/Coordinates.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace environments {

// provides geographic context and gravity corrections based on actual position from Earth center
class Geographic : public IEnvironment {
public:
    explicit Geographic(double referenceLatitude = constants::DEFAULT_LATITUDE, double referenceLongitude = constants::DEFAULT_LONGITUDE, double groundY = 0.0);

    void update(IPhysicsBody& body, PhysicsContext& context) override;

    // getters
    const std::string& getName() const override { return m_name; }
    int getPriority() const override { return m_priority; }

private:
    std::string m_name = "Geographic";
    int m_priority = 0;

    geography::GeographicPosition m_reference;
    double m_groundY;
};

} // namespace environments
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
