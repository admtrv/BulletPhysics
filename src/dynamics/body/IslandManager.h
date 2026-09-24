/*
 * IslandManager.h
 */

#pragma once

#include "collision/Collision.h"
#include "math/Vec3.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace BulletPhysics {
namespace dynamics {

class RigidBody;

// groups bodies joined by contacts
class IslandManager {
public:
    void build(const std::vector<RigidBody*>& bodies, const std::vector<collision::Manifold>& contacts);

    // parks islands that stood still long enough, wakes rest
    void updateSleep(double dt);

    void wake(RigidBody* body);

private:
    using Members = std::unordered_set<RigidBody*>;

    // static neighbour never joins island, watched by pose instead
    struct Support {
        const RigidBody* body = nullptr;
        math::Vec3 position{};

        bool operator==(const Support& other) const { return body == other.body && position == other.position; }
    };

    using Supports = std::vector<Support>;

    struct Island {
        Members bodies;
        double stillTime = 0.0;
    };

    int rootOf(int index);
    Supports supportsOf(const Members& bodies, const std::vector<collision::Manifold>& contacts) const;

    std::vector<Island> m_islands;

    // union-find over body list, contacts merge sets
    std::vector<int> m_parent;
    std::vector<RigidBody*> m_bodies;
    std::unordered_map<const RigidBody*, int> m_index;

    // carried across steps by bodies, island itself lives one step
    std::unordered_map<const RigidBody*, double> m_stillTime;

    // company of last step, changed one means support lost or gained
    std::unordered_map<const RigidBody*, Members> m_company;
    std::unordered_map<const RigidBody*, Supports> m_supports;
};

} // namespace dynamics
} // namespace BulletPhysics
