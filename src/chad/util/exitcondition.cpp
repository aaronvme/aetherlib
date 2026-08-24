// Ported from LemLib - https://github.com/LemLib/LemLib (MIT)
#include "chad/util/exitcondition.hpp"

#include "pros/rtos.hpp"

namespace chad {

namespace {
SettleSpec specFor(float range, int time) {
    SettleSpec spec;
    spec.errorTolerance = range;
    spec.holdMs = time > 0 ? static_cast<uint32_t>(time) : 0u;
    spec.graceMs = 0;
    return spec;
}
}  // namespace

ExitCondition::ExitCondition(float range, int time) : mMonitor(specFor(range, time)) {}

bool ExitCondition::getExit() const { return mMonitor.done(); }

bool ExitCondition::update(float input) {
    const uint32_t now = pros::millis();
    if (!mStarted) {
        mMonitor.reset(now);
        mStarted = true;
    }
    return mMonitor.update(now, input, 0.0f) != SettleState::Running;
}

void ExitCondition::reset() {
    mMonitor.reset(pros::millis());
    mStarted = true;
}

}  // namespace chad
