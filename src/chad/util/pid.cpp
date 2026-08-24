// Ported from FBLib - https://github.com/jiazegao/FBLib
#include "chad/util/pid.hpp"

#include "pros/rtos.hpp"

namespace chad {

PID::PID(float kP, float kI, float kD, float windupRange, bool flipReset)
    : mCore(PIDGains{kP, kI, kD}, windupRange, flipReset),
      mPreviousTime(pros::millis()) {}

PID::PID(const PIDGains& gains, float windupRange, bool flipReset)
    : mCore(gains, windupRange, flipReset), mPreviousTime(pros::millis()) {}

float PID::update(float error) {
    const uint32_t now = pros::millis();
    float dt = static_cast<float>(now - mPreviousTime) / 1000.0f;
    mPreviousTime = now;

    // A stalled task or a first call after a long gap would otherwise charge
    // the integrator for the whole idle period.
    if (dt > 0.1f) dt = 0.1f;
    if (mFixedDt > 0.0f) dt = mFixedDt;

    return mCore.update(error, dt);
}

void PID::reset() {
    mCore.reset();
    mPreviousTime = pros::millis();
}

}  // namespace chad
