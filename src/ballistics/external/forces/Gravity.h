/*
 * Gravity.h
 */

#pragma once

#include "Force.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace forces {

class Gravity : public IForce {
public:
    void apply(IPhysicsBody& body, PhysicsContext& context) override;

    // getters
    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Gravitational";
    std::string m_symbol = "Fg";
};

} // namespace forces
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
