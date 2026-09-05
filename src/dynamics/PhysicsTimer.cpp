/*
 * PhysicsTimer.cpp
 */

#include "PhysicsTimer.h"

namespace BulletPhysics {
namespace dynamics {

void PhysicsTimer::setTimeStep(double timeStep)
{
    m_timeStep = (timeStep > 0.0 ? timeStep : DEFAULT_TIME_STEP);
}

void PhysicsTimer::setMaxSubSteps(int maxSubSteps)
{
    m_maxSubSteps = (maxSubSteps > 0 ? maxSubSteps : 1);
}

int PhysicsTimer::consume(double frameTime)
{
    if (frameTime > 0.0)
    {
        m_accumulator += frameTime;
    }

    int steps = static_cast<int>(m_accumulator / m_timeStep);

    if (steps > m_maxSubSteps)
    {
        // drop backlog, better slow motion for a moment than a stall
        steps = m_maxSubSteps;
        m_accumulator = 0.0;

        return steps;
    }

    m_accumulator -= steps * m_timeStep;

    return steps;
}

} // namespace dynamics
} // namespace BulletPhysics
