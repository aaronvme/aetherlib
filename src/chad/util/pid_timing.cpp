// The clock-reading half of PID, kept apart so the arithmetic in pid.cpp
// builds and tests off the robot.
#include <cmath>

#include "chad/util/pid.hpp"
#include "pros/rtos.hpp"

using namespace chad;

float PID::update(float error) {
    if (!std::isfinite(error)) return 0.0f;

    uint32_t currentTime = pros::millis();
    float dt = (currentTime - mPreviousTime) / 1000.0f;

    if (dt > 0.1f) dt = 0.1f;
    mPreviousTime = currentTime;

    if (mFixedDt > 0.0f) dt = mFixedDt;

    return step(error, dt);
}

void PID::reset() {
    clearState();
    mPreviousTime = pros::millis();
}
