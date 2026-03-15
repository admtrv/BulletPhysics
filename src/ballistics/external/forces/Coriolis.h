/*
 * Coriolis.h
 */

#pragma once

#include "Force.h"
#include "geography/Coordinates.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace forces {

class Coriolis : public IForce {
public:
    void apply(IPhysicsBody& body, PhysicsContext& context) override;

    // getters
    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Coriolis";
    std::string m_symbol = "Fc";
};

} // namespace forces
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
