# Project Structure

```
src/
├── ballistics/         physics related
│   ├── external/           trajectory simulation
│   │   ├── forces/             all forces
│   │   ├── environments/       all environments
│   │   ├── PhysicsContext
│   │   └── PhysicsWorld
│   └── terminal/           impact resolution
├── math/               math related
│   ├── Vec3                vec3 via double
│   ├── Integrator          all integrators
│   └── ...
├── geography/          geography related
├── builtin/            concrete implementations
│   ├── bodies/             rigid bodies
│   └── collision/          collision detection
├── PhysicsBody         core abstract interfaces
└── Constants
```

# Usage

## Body

The library operates on the `IPhysicsBody` interface. Integrate it with your own rigid body system, or use the builtin `RigidBody` / `ProjectileRigidBody`.

For basic simulation:

```cpp
#include "PhysicsBody.h"
#include "builtin/bodies/RigidBody.h"

RigidBody body;
body.setMass(1.0);
body.setPosition({0.0, 1.5, 0.0});
body.setVelocity({20.0, 10.0, 0.0});
```

For projectile simulation with ballistic properties, fill `ProjectileSpecs` and use `ProjectileRigidBody`:

```cpp
#include "PhysicsBody.h"
#include "builtin/bodies/RigidBody.h"

ProjectileSpecs specs{};
specs.mass = 0.01;
specs.diameter = 0.00762;
specs.dragModel = DragCurveModel::G7;
...

ProjectileRigidBody body(specs);
body.setPosition({0.0, 1.5, 0.0});
body.setVelocity({20.0, 10.0, 0.0});
```

## Integrator

Choose a numerical integration method:

```cpp
#include "math/Integrator.h"

EulerIntegrator euler;          // fastest, least accurate
MidpointIntegrator midpoint;    // good balance
RK4Integrator rk4;              // slowest, most accurate
```

## PhysicsWorld

`PhysicsWorld` manages forces and environments. Environments update the shared `PhysicsContext`, then Forces read it.

```cpp
#include "ballistics/external/PhysicsWorld.h"

PhysicsWorld world;

// environments
world.addEnvironment(std::make_unique<Atmosphere>(280.0, 100000.0));                    // sea-level temp K, sea-level pressure Pa
world.addEnvironment(std::make_unique<Humidity>(60));                                   // relative humidity correction %
world.addEnvironment(std::make_unique<Wind>(Vec3{0.0, 0.0, 2.0}));                      // wind velocity m/s
world.addEnvironment(std::make_unique<Geographic>(deg2rad(48.15), deg2rad(17.11)));     // latitude/longitude in radians

// forces
world.addForce(std::make_unique<Gravity>());
world.addForce(std::make_unique<Drag>());
world.addForce(std::make_unique<Coriolis>());
world.addForce(std::make_unique<Lift>());
world.addForce(std::make_unique<Magnus>());
```

## Simulation Loop

```cpp
double dt = 0.001;
double t = 0.0;

while (true)
{
    integrator.step(body, &world, dt);
    t += dt;

    auto pos = body.getPosition();
    // use position...
}
```

## Detail Levels

Configure based on required realism level:

**Minimum (Gravity only):**
```cpp
PhysicsWorld world;
world.addForce(std::make_unique<Gravity>());
```

**+ Aerodynamic drag:**
```cpp
world.addEnvironment(std::make_unique<Atmosphere>());   // for standard isa atmosphere
world.addForce(std::make_unique<Drag>());

ProjectileSpecs specs{};
specs.mass = 0.01;
specs.diameter = 0.00762;               // for area calculation
specs.dragModel = DragCurveModel::G7;
```

**+ Sea-level condition correction:**
```cpp
world.addEnvironment(std::make_unique<Atmosphere>(280.0, 100000.0));
```

**+ Humidity correction:**
```cpp
world.addEnvironment(std::make_unique<Humidity>(60));
```

**+ Wind vector:**
```cpp
world.addEnvironment(std::make_unique<Wind>(Vec3{0.0, 0.0, 2.0}));
```

**+ Coriolis effect:**
```cpp
world.addEnvironment(std::make_unique<Geographic>(lat, lon));   // for geographic coordinates and position-dependent gravity
world.addForce(std::make_unique<Coriolis>());
```

**+ Spin drift (Lift + Magnus):**
```cpp
world.addForce(std::make_unique<Lift>());
world.addForce(std::make_unique<Magnus>());
// or all at once via SpinDrift::addTo(world);

specs.spinSpecs = SpinSpecs{};
specs.spinSpecs.riflingSpecs = RiflingSpecs{ Direction::RIGHT, 12.0 };      // for projectile rotation data
```

## Terminal Ballistics

Resolve projectile impact against a collider with material:

```cpp
#include "ballistics/terminal/Impact.h"
#include "ballistics/terminal/Material.h"

void CollisionSystem::onCollision()
{
    ...
    
    ImpactInfo info;
    info.normal = manifold.info.normal;
    info.material = collider.getMaterial();                                                 // Wood(), Steel(), ...
    info.thickness = collider.computeThickness(body.getPosition(), body.getVelocity());     // effective thickness

    auto result = Impact::resolve(projectileBody, info);

    switch (result.outcome)
    {
        case ImpactOutcome::Ricochet:
            // your logic...
        case ImpactOutcome::Penetration:
            // your logic...
        case ImpactOutcome::Embed:
            // your logic...
    }
    
    ...
```