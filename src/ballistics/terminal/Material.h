/*
 * Material.h
 */

#pragma once

#include <string>

namespace BulletPhysics {
namespace ballistics {
namespace terminal {

struct Material {
    std::string name;

    // penetration
    double density;                  // kg/m^3
    double penetrationResistance;    // Pa, resistance pressure against projectile cross-section

    // ricochet
    double yieldStrength;            // Pa, ultimate tensile strength (Wijk ricochet model)

    double restitutionN;             // 0..1, coefficient of restitution (normal impulse)
    double frictionT;                // 0..1, Coulomb friction coefficient (tangential impulse)
    double empiricalEnergyLoss;      // 0..1, fraction of kinetic energy lost on ricochet (experimental)
};

namespace materials {

// Steel (experimental loss: ~12.1%, 7.62 mm data)
inline Material Steel() {
    return {"Steel", 7850.0, 500e6, 1200e6, 0.25, 0.4, 0.121};
}

// Standard structural concrete (experimental loss: ~85.2%, 7.62mm AKM data)
inline Material Concrete() {
    return {"Concrete", 2300.0, 40e6, 30e6, 0.15, 0.5, 0.852};
}

// Softwood (experimental loss: 71-97%, avg ~85.2%, 7.65 mm Browning data)
inline Material Wood() {
    return {"Wood", 550.0, 4e6, 40e6, 0.1, 0.35, 0.852};
}

// Compacted soil / earth (similar to sand) (experimental loss: ~96.4%, 25mm projectile data)
inline Material Soil() {
    return {"Soil", 1600.0, 1.5e6, 0.5e6, 0.05, 0.5, 0.964};
}

} // namespace materials
} // namespace terminal
} // namespace ballistics
} // namespace BulletPhysics
