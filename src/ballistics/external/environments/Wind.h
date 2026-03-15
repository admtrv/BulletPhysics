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
    explicit Wind(const math::Vec3& windVelocity = constants::DEFAULT_WIND);

    void update(IPhysicsBody& /*body*/, PhysicsContext& context) override;

    // getters
    const std::string& getName() const override { return m_name; }
    int getPriority() const override { return m_priority; }

private:
    std::string m_name = "Wind";
    int m_priority = 0;

    math::Vec3 m_velocity;
};

} // namespace environments
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
