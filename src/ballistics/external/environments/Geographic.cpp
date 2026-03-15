/*
 * Geographic.cpp
 */

#include "Geographic.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace environments {

Geographic::Geographic(double referenceLatitude, double referenceLongitude, double groundY) : m_reference(referenceLatitude, referenceLongitude, 0.0), m_groundY(groundY) {}

void Geographic::update(IPhysicsBody& body, PhysicsContext& context)
{
    // calculate altitude above ground level
    double altitude = std::max(0.0, body.getPosition().y - m_groundY);

    // store geographic information
    context.latitude = m_reference.latitude;
    context.longitude = m_reference.longitude;
    context.altitude = altitude;

    // position dependent gravity
    geography::GeographicPosition currentPosition(m_reference.latitude, m_reference.longitude, altitude);
    double g = geography::gravitationalAccelerationAtGeodetic(currentPosition);

    // store corrected gravity
    context.gravity = math::Vec3{0.0, -g, 0.0};
}

} // namespace environments
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
