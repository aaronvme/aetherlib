// Ported from LemLib - https://github.com/LemLib/LemLib (MIT)
#pragma once

namespace riku {

/// Debounced settle check: error must stay within `range` for `time` ms.
/// Prevents exiting a motion while still carrying velocity through the target.
class ExitCondition {
public:
    ExitCondition(float range, int time);

    bool getExit() const;

    /// Feed the current error. Returns true once the condition is met.
    bool update(float input);

    void reset();

private:
    float mRange;
    int mTime;
    int mStartTime{-1};
    bool mDone{false};
};

}  // namespace riku
