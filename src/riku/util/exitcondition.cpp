// Ported from LemLib - https://github.com/LemLib/LemLib (MIT)
#include "riku/util/exitcondition.hpp"

#include <cmath>

#include "pros/rtos.hpp"

namespace riku {

ExitCondition::ExitCondition(float range, int time) : mRange(range), mTime(time) {}

bool ExitCondition::getExit() const { return mDone; }

bool ExitCondition::update(float input) {
    const int now = pros::millis();
    if (std::fabs(input) > mRange) {
        mStartTime = -1;
    } else if (mStartTime == -1) {
        mStartTime = now;
    } else if (now >= mStartTime + mTime) {
        mDone = true;
    }
    return mDone;
}

void ExitCondition::reset() {
    mStartTime = -1;
    mDone = false;
}

}  // namespace riku
