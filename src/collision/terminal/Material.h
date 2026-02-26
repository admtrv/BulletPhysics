/*
 * Material.h
 */

#pragma once

#include <string>

namespace BulletPhysics {
namespace collision {
namespace terminal {

struct Material {
    std::string name;

    // penetration
    float density;                  // kg/m^3
    float penetrationResistance;    // Pa, resistance pressure against projectile cross-section

    // ricochet
    float yieldStrength;            // Pa, ultimate tensile strength (Wijk ricochet model)

    float restitutionN;             // 0..1, coefficient of restitution (normal impulse)
    float frictionT;                // 0..1, Coulomb friction coefficient (tangential impulse)
    float empiricalEnergyLoss;      // 0..1, fraction of kinetic energy lost on ricochet (experimental)
};

namespace materials {

// Steel (experimental loss: ~12.1%, 7.62 mm data)
inline Material Steel() {
    return {"Steel", 7850.0f, 500e6f, 1200e6f, 0.25f, 0.4f, 0.121f};
}

// Standard structural concrete (experimental loss: ~85.2%, 7.62mm AKM data)
inline Material Concrete() {
    return {"Concrete", 2300.0f, 40e6f, 30e6f, 0.15f, 0.5f, 0.852f};
}

// Softwood (experimental loss: 71-97%, avg ~85.2%, 7.65 mm Browning data)
inline Material Wood() {
    return {"Wood", 550.0f, 4e6f, 40e6f, 0.1f, 0.35f, 0.852f};
}

// Compacted soil / earth (similar to sand) (experimental loss: ~96.4%, 25mm projectile data)
inline Material Soil() {
    return {"Soil", 1600.0f, 1.5e6f, 0.5e6f, 0.05f, 0.5f, 0.964f};
}

} // namespace materials
} // namespace terminal
} // namespace collision
} // namespace BulletPhysics
