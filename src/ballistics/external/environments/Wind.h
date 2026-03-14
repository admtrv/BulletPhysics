/*
 * Wind.h
 */

#pragma once

#include "Environment.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace environments {

// provides wind velocity
class Wind : public IEnvironment {
public:
    explicit Wind(const math::Vec3& windVelocity = math::Vec3{0.0, 0.0, 0.0}) : m_velocity(windVelocity) {}

    void update(IPhysicsBody& /*body*/, PhysicsContext& context) override
    {
        context.wind = m_velocity;
    }

    void setWind(const math::Vec3& windVel) { m_velocity = windVel; }
    const math::Vec3& getWind() const { return m_velocity; }

    const std::string& getName() const override { return m_name; }

private:
    std::string m_name = "Wind";

    math::Vec3 m_velocity;
};

} // namespace environments
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
