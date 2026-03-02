/*
 * PhysicsWorld.cpp
 */

#include "PhysicsWorld.h"

namespace BulletPhysics {
namespace dynamics {

void PhysicsWorld::addForce(std::unique_ptr<forces::IForce> force)
{
    if (force)
    {
        m_forces.push_back(std::move(force));
    }
}

void PhysicsWorld::addEnvironment(std::unique_ptr<environment::IEnvironment> environment)
{
    if (environment)
    {
        m_environments.push_back(std::move(environment));
    }
}

void PhysicsWorld::clear()
{
    m_forces.clear();
    m_environments.clear();
}

void PhysicsWorld::applyForces(IPhysicsBody& body, double dt)
{
    m_context.reset();

    // phase 1: environment providers update context
    for (auto& env : m_environments)
    {
        if (env)
        {
            env->update(body, m_context);
        }
    }

    // phase 2: forces apply using context
    for (auto& force : m_forces)
    {
        if (force && force->isActive())
        {
            force->apply(body, m_context, dt);
        }
    }
}

forces::IForce* PhysicsWorld::getForce(const std::string& name)
{
    for (auto& force : m_forces)
    {
        if (force && force->getName() == name)
        {
            return force.get();
        }
    }
    return nullptr;
}

environment::IEnvironment* PhysicsWorld::getEnvironment(const std::string& name)
{
    for (auto& env : m_environments)
    {
        if (env && env->getName() == name)
        {
            return env.get();
        }
    }
    return nullptr;
}

} // namespace dynamics
} // namespace BulletPhysics
