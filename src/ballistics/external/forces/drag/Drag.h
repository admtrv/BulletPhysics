/*
 * Drag.h
 */

#pragma once

#include "../Force.h"
#include "DragModel.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace forces {

// aerodynamic drag force
class Drag : public IForce {
public:
    void apply(IPhysicsBody& body, PhysicsContext& context) override;

    // getters
    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Aerodynamic Drag";
    std::string m_symbol = "Fd";
};

} // namespace forces
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
