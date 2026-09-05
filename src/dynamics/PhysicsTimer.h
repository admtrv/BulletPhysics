/*
 * PhysicsTimer.h
 */

#pragma once

namespace BulletPhysics {
namespace dynamics {

inline constexpr double DEFAULT_TIME_STEP = 1.0 / 60.0;
inline constexpr int DEFAULT_MAX_SUB_STEPS = 8;

// splits frame time into equal steps
class PhysicsTimer {
public:
    PhysicsTimer() = default;

    double getTimeStep() const { return m_timeStep; }
    void setTimeStep(double timeStep);

    // cap on steps per frame, long frame would ask for even more
    int getMaxSubSteps() const { return m_maxSubSteps; }
    void setMaxSubSteps(int maxSubSteps);

    int consume(double frameTime);

    // 0..1 towards next step, for interpolating render
    double getAlpha() const { return m_accumulator / m_timeStep; }

    void reset() { m_accumulator = 0.0; }

private:
    double m_timeStep = DEFAULT_TIME_STEP;
    int m_maxSubSteps = DEFAULT_MAX_SUB_STEPS;

    double m_accumulator = 0.0;
};

} // namespace dynamics
} // namespace BulletPhysics
