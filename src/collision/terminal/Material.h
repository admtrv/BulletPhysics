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
    float density;          // kg/m^3
    float brinellHardness;  // BHN — resistance to deformation
    float yieldStrength;    // Pa — plastic deformation threshold
    float restitution;      // 0..1 — energy retention on ricochet
};

namespace materials {

// Heat-treated hardened steel
inline Material Steel() {
    return {"Steel", 7850.0f, 500.0f, 1200e6f, 0.25f};
}

// Standard structural concrete
inline Material Concrete() {
    return {"Concrete", 2300.0f, 40.0f, 30e6f, 0.15f};
}

// Softwood (pine)
inline Material Wood() {
    return {"Wood", 550.0f, 4.0f, 40e6f, 0.1f};
}

// Compacted soil / earth
inline Material Soil() {
    return {"Soil", 1600.0f, 1.5f, 0.5e6f, 0.05f};
}

} // namespace materials
} // namespace terminal
} // namespace collision
} // namespace BulletPhysics
