/*
 * IslandManager.cpp
 */

#include "IslandManager.h"

#include "dynamics/body/RigidBody.h"

#include <algorithm>

namespace BulletPhysics {
namespace dynamics {

void IslandManager::build(const std::vector<RigidBody*>& bodies, const std::vector<collision::Manifold>& contacts)
{
    m_bodies.clear();
    m_index.clear();

    // static and kinematic bodies join no island, they would tie the whole scene
    // into one group through the ground
    for (RigidBody* body : bodies)
    {
        if (!body->isDynamic())
        {
            continue;
        }

        m_index[body] = static_cast<int>(m_bodies.size());
        m_bodies.push_back(body);
    }

    m_parent.resize(m_bodies.size());
    for (size_t i = 0; i < m_parent.size(); i++)
    {
        m_parent[i] = static_cast<int>(i);
    }

    for (const auto& contact : contacts)
    {
        const auto a = m_index.find(contact.colliderA->getBody());
        const auto b = m_index.find(contact.colliderB->getBody());

        if (a == m_index.end() || b == m_index.end())
        {
            continue;
        }

        m_parent[rootOf(a->second)] = rootOf(b->second);
    }

    // collect the sets, the root of each one names its island
    std::unordered_map<int, size_t> islandOf;
    m_islands.clear();

    for (size_t i = 0; i < m_bodies.size(); i++)
    {
        const int root = rootOf(static_cast<int>(i));

        auto found = islandOf.find(root);
        if (found == islandOf.end())
        {
            found = islandOf.emplace(root, m_islands.size()).first;
            m_islands.emplace_back();
        }

        m_islands[found->second].bodies.insert(m_bodies[i]);
    }

    // an island carries on the stillness of the bodies it took in, and one whose
    // company changed starts over, the box it rested on may have just left
    std::unordered_map<const RigidBody*, Members> company;

    for (auto& island : m_islands)
    {
        double stillTime = SLEEP_DELAY;

        for (const RigidBody* body : island.bodies)
        {
            const auto found = m_stillTime.find(body);
            stillTime = std::min(stillTime, (found != m_stillTime.end()) ? found->second : 0.0);

            company[body] = island.bodies;
        }

        const auto previous = m_company.find(*island.bodies.begin());
        island.stillTime = (previous != m_company.end() && previous->second == island.bodies) ? stillTime : 0.0;
    }

    m_company = std::move(company);
}

void IslandManager::updateSleep(double dt)
{
    for (auto& island : m_islands)
    {
        const bool moving = std::any_of(island.bodies.begin(), island.bodies.end(), [](const RigidBody* body) {
            return body->getVelocity().length() > SLEEP_LINEAR_SPEED
                || body->getAngularVelocity().length() > SLEEP_ANGULAR_SPEED;
        });

        island.stillTime = moving ? 0.0 : island.stillTime + dt;

        const bool asleep = island.stillTime >= SLEEP_DELAY;

        for (RigidBody* body : island.bodies)
        {
            m_stillTime[body] = island.stillTime;

            if (asleep)
            {
                body->sleep();
            }
            else
            {
                body->wake();
            }
        }
    }
}

void IslandManager::wake(RigidBody* body)
{
    for (auto& island : m_islands)
    {
        if (island.bodies.count(body) == 0)
        {
            continue;
        }

        island.stillTime = 0.0;

        for (RigidBody* member : island.bodies)
        {
            m_stillTime[member] = 0.0;
            member->wake();
        }

        return;
    }
}

int IslandManager::rootOf(int index)
{
    while (m_parent[index] != index)
    {
        // path halving keeps the trees flat without a second pass
        m_parent[index] = m_parent[m_parent[index]];
        index = m_parent[index];
    }

    return index;
}

} // namespace dynamics
} // namespace BulletPhysics
