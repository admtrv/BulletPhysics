/*
 * ContactListener.h
 */

#pragma once

#include "collision/Collision.h"

namespace BulletPhysics {
namespace dynamics {

// notified as contacts come and go
class IContactListener {
public:
    virtual ~IContactListener() = default;

    virtual void onContactBegin(const collision::Manifold& manifold) {}
    virtual void onContactStay(const collision::Manifold& manifold) {}
    virtual void onContactEnd(collision::collider::Collider* a, collision::collider::Collider* b) {}
};

} // namespace dynamics
} // namespace BulletPhysics
