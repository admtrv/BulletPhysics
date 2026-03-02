/*
 * Geographic.h
 */

#pragma once

#include "Environment.h"
#include "geography/Coordinates.h"

#include <cmath>

namespace BulletPhysics {
namespace dynamics {
namespace environment {

// provides geographic context and gravity corrections based on actual position from Earth center
class Geographic : public IEnvironment {
public:
    explicit Geographic(double referenceLatitude, double referenceLongitude, double groundY = 0.0)
        : m_reference(referenceLatitude, referenceLongitude, 0.0)
        , m_groundY(groundY)
    {}

    void update(IPhysicsBody& body, PhysicsContext& context) override
    {
        // calculate altitude above ground level
        double altitudeAbove = std::max(0.0, body.getPosition().y - m_groundY);

        // store geographic information
        context.latitude = m_reference.latitude;
        context.longitude = m_reference.longitude;
        context.altitude = altitudeAbove;

        // correct gravity
        geography::GeographicPosition currentPosition(m_reference.latitude, m_reference.longitude, altitudeAbove);

        context.gravity = geography::gravitationalAccelerationAtGeodetic(currentPosition);
    }

    const std::string& getName() const override { return m_name; }

    double getReferenceLatitude() const { return m_reference.latitude; }
    double getReferenceLongitude() const { return m_reference.longitude; }

private:
    std::string m_name = "Geographic";

    geography::GeographicPosition m_reference;
    double m_groundY;
};

} // namespace environment
} // namespace dynamics
} // namespace BulletPhysics
