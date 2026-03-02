/*
 * Integrator.cpp
 */

#include "Integrator.h"

namespace BulletPhysics {
namespace math {

void EulerIntegrator::step(dynamics::IPhysicsBody& body, dynamics::PhysicsWorld* world, double dt)
{
    // clear previous forces
    body.clearForces();

    // apply all registered forces
    if (world)
    {
        world->applyForces(body, dt);
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
}

void MidpointIntegrator::step(dynamics::IPhysicsBody& body, dynamics::PhysicsWorld* world, double dt)
{
    // clear previous forces
    body.clearForces();

    const Vec3 r0 = body.getPosition();
    const Vec3 v0 = body.getVelocity();

    // helper lambda to calculate acceleration at given state
    auto calcAccel = [&](const Vec3& pos, const Vec3& vel) -> Vec3
    {
        // temporarily set state
        auto tempBody = body.clone();
        tempBody->setPosition(pos);
        tempBody->setVelocity(vel);
        tempBody->clearForces();

        if (world)
        {
            world->applyForces(*tempBody, dt);
        }

        Vec3 a = {0, 0, 0};
        if (tempBody->getMass() > 0.0)
        {
            a = tempBody->getAccumulatedForces() / tempBody->getMass();
        }

        return a;
    };

    // RK2 (Midpoint) steps
    Vec3 a0 = calcAccel(r0, v0);

    const Vec3 k1_v = a0 * dt;
    const Vec3 k1_r = v0 * dt;

    // evaluate at midpoint
    Vec3 a_mid = calcAccel(r0 + k1_r * 0.5, v0 + k1_v * 0.5);
    const Vec3 k2_v = a_mid * dt;
    const Vec3 k2_r = (v0 + k1_v * 0.5) * dt;

    // combine steps
    Vec3 v = v0 + k2_v;
    Vec3 r = r0 + k2_r;

    body.setPosition(r);
    body.setVelocity(v);
    body.clearForces();
}

void RK4Integrator::step(dynamics::IPhysicsBody& body, dynamics::PhysicsWorld* world, double dt)
{
    // clear previous forces
    body.clearForces();

    const Vec3 r0 = body.getPosition();
    const Vec3 v0 = body.getVelocity();

    // helper lambda to calculate acceleration at given state
    auto calcAccel = [&](const Vec3& pos, const Vec3& vel) -> Vec3
    {
        // temporarily set state
        auto tempBody = body.clone();
        tempBody->setPosition(pos);
        tempBody->setVelocity(vel);
        tempBody->clearForces();

        if (world)
        {
            world->applyForces(*tempBody, dt);
        }

        Vec3 a = {0, 0, 0};
        if (tempBody->getMass() > 0.0)
        {
            a = tempBody->getAccumulatedForces() / tempBody->getMass();
        }

        return a;
    };

    // RK4 steps
    Vec3 a0 = calcAccel(r0, v0);

    const Vec3 k1_v = a0 * dt;
    const Vec3 k1_r = v0 * dt;

    Vec3 a1 = calcAccel(r0 + k1_r * 0.5, v0 + k1_v * 0.5);
    const Vec3 k2_v = a1 * dt;
    const Vec3 k2_r = (v0 + k1_v * 0.5) * dt;

    Vec3 a2 = calcAccel(r0 + k2_r * 0.5, v0 + k2_v * 0.5);
    const Vec3 k3_v = a2 * dt;
    const Vec3 k3_r = (v0 + k2_v * 0.5) * dt;

    Vec3 a3 = calcAccel(r0 + k3_r, v0 + k3_v);
    const Vec3 k4_v = a3 * dt;
    const Vec3 k4_r = (v0 + k3_v) * dt;

    // combine steps
    Vec3 v = v0 + (k1_v + k2_v * 2.0 + k3_v * 2.0 + k4_v) * (1.0 / 6.0);
    Vec3 r = r0 + (k1_r + k2_r * 2.0 + k3_r * 2.0 + k4_r) * (1.0 / 6.0);

    body.setPosition(r);
    body.setVelocity(v);
    body.clearForces();
}

} // namespace math
} // namespace BulletPhysics
