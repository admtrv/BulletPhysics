/*
 * SpinDrift.h
 */

#pragma once

#include "Force.h"

namespace BulletPhysics {
namespace ballistics {
namespace external {
namespace forces {

class Lift : public IForce {
public:
    void apply(IPhysicsBody& body, PhysicsContext& context) override;

    // getters
    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Lift";
    std::string m_symbol = "Fl";
};

class Magnus : public IForce {
public:
    void apply(IPhysicsBody& body, PhysicsContext& context) override;

    // getters
    const std::string& getName() const override { return m_name; }
    const std::string& getSymbol() const override { return m_symbol; }

private:
    std::string m_name = "Magnus";
    std::string m_symbol = "Fm";
};

// spin drift influence
class SpinDrift {
public:
    template<typename PhysicsWorldType>
    static void addTo(PhysicsWorldType& world)
    {
        world.addForce(std::make_unique<Lift>());
        world.addForce(std::make_unique<Magnus>());
    }
};

} // namespace forces
} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
