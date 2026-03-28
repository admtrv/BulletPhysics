/*
 * Integrator.cpp
 */

#include "Integrator.h"
#include "geography/CoordinateMapping.h"

namespace BulletPhysics {
namespace math {

// convert body state between user space and internal EUN
static void bodyToInternal(IPhysicsBody& body, const geography::CoordinateMapping& m)
{
    if (!m.isIdentity())
    {
        body.setPosition(m.toInternal(body.getPosition()));
        body.setVelocity(m.toInternal(body.getVelocity()));
    }
}

static void bodyToExternal(IPhysicsBody& body, const geography::CoordinateMapping& m)
{
    if (!m.isIdentity())
    {
        body.setPosition(m.toExternal(body.getPosition()));
        body.setVelocity(m.toExternal(body.getVelocity()));
    }
}

static const geography::CoordinateMapping& getMapping(ballistics::external::PhysicsWorld* world)
{
    static const geography::CoordinateMapping identity = geography::mappings::EUN();
    return world ? world->getCoordinateMapping() : identity;
}

// calculate acceleration at given state
Vec3 calcAccel(IPhysicsBody& body, ballistics::external::PhysicsWorld* world, double dt, const Vec3& pos, const Vec3& vel)
{
    body.setPosition(pos);
    body.setVelocity(vel);
    body.clearForces();

    if (world)
    {
        world->applyForces(body);
    }

    Vec3 a = {0, 0, 0};
    if (body.getMass() > 0.0)
    {
        a = body.getAccumulatedForces() / body.getMass();
    }

    return a;
}

void EulerIntegrator::step(IPhysicsBody& body, ballistics::external::PhysicsWorld* world, double dt)
{
    const auto& mapping = getMapping(world);
    bodyToInternal(body, mapping);

    // clear previous forces
    body.clearForces();

    // apply all registered forces
    if (world)
    {
        world->applyForces(body);
    }

    // calculate acceleration
    Vec3 a = {0, 0, 0};
    if (body.getMass() > 0.0)
    {
        a = body.getAccumulatedForces() / body.getMass();
    }

    // integrate
    Vec3 v = body.getVelocity() + a * dt;
    Vec3 r = body.getPosition() + body.getVelocity() * dt;

    body.setPosition(r);
    body.setVelocity(v);
    body.clearForces();

    bodyToExternal(body, mapping);
}

void MidpointIntegrator::step(IPhysicsBody& body, ballistics::external::PhysicsWorld* world, double dt)
{
    const auto& mapping = getMapping(world);
    bodyToInternal(body, mapping);

    // clear previous forces
    body.clearForces();

    const Vec3 r0 = body.getPosition();
    const Vec3 v0 = body.getVelocity();

    // RK2 (Midpoint) steps
    Vec3 a0 = calcAccel(body, world, dt, r0, v0);

    const Vec3 k1_v = a0 * dt;
    const Vec3 k1_r = v0 * dt;

    // evaluate at midpoint
    Vec3 a_mid = calcAccel(body, world, dt, r0 + k1_r * 0.5, v0 + k1_v * 0.5);
    const Vec3 k2_v = a_mid * dt;
    const Vec3 k2_r = (v0 + k1_v * 0.5) * dt;

    // combine steps
    Vec3 v = v0 + k2_v;
    Vec3 r = r0 + k2_r;

    body.setPosition(r);
    body.setVelocity(v);
    body.clearForces();

    bodyToExternal(body, mapping);
}

void RK4Integrator::step(IPhysicsBody& body, ballistics::external::PhysicsWorld* world, double dt)
{
    const auto& mapping = getMapping(world);
    bodyToInternal(body, mapping);

    // clear previous forces
    body.clearForces();

    const Vec3 r0 = body.getPosition();
    const Vec3 v0 = body.getVelocity();

    // RK4 steps
    Vec3 a0 = calcAccel(body, world, dt, r0, v0);

    const Vec3 k1_v = a0 * dt;
    const Vec3 k1_r = v0 * dt;

    Vec3 a1 = calcAccel(body, world, dt, r0 + k1_r * 0.5, v0 + k1_v * 0.5);
    const Vec3 k2_v = a1 * dt;
    const Vec3 k2_r = (v0 + k1_v * 0.5) * dt;

    Vec3 a2 = calcAccel(body, world, dt, r0 + k2_r * 0.5, v0 + k2_v * 0.5);
    const Vec3 k3_v = a2 * dt;
    const Vec3 k3_r = (v0 + k2_v * 0.5) * dt;

    Vec3 a3 = calcAccel(body, world, dt, r0 + k3_r, v0 + k3_v);
    const Vec3 k4_v = a3 * dt;
    const Vec3 k4_r = (v0 + k3_v) * dt;

    // combine steps
    Vec3 v = v0 + (k1_v + k2_v * 2.0 + k3_v * 2.0 + k4_v) * (1.0 / 6.0);
    Vec3 r = r0 + (k1_r + k2_r * 2.0 + k3_r * 2.0 + k4_r) * (1.0 / 6.0);

    body.setPosition(r);
    body.setVelocity(v);
    body.clearForces();

    bodyToExternal(body, mapping);
}

} // namespace math
} // namespace BulletPhysics
