/*
 * PhysicsWorld.cpp
 */

#include "PhysicsWorld.h"

#include <algorithm>

namespace BulletPhysics {
namespace ballistics {
namespace external {

void PhysicsWorld::addForce(std::unique_ptr<forces::IForce> force)
{
    if (force)
    {
        m_forces.push_back(std::move(force));
    }
}

void PhysicsWorld::addEnvironment(std::unique_ptr<environments::IEnvironment> environment)
{
    if (environment)
    {
        int priority = environment->getPriority();
        auto it = std::upper_bound(m_environments.begin(), m_environments.end(), priority,
            [](int p, const std::unique_ptr<environments::IEnvironment>& e) {
                return p < e->getPriority();
            });
        m_environments.insert(it, std::move(environment));
    }
}

void PhysicsWorld::clear()
{
    m_forces.clear();
    m_environments.clear();
}

void PhysicsWorld::applyForces(IPhysicsBody& body)
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
            force->apply(body, m_context);
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

environments::IEnvironment* PhysicsWorld::getEnvironment(const std::string& name)
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


} // namespace external
} // namespace ballistics
} // namespace BulletPhysics
