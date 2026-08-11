// Ported from LemLib - https://github.com/LemLib/LemLib (MIT)
#pragma once

namespace riku {

/// Error must stay within `range` for `time` ms. Stops a motion exiting while
/// still carrying velocity through the target.
class ExitCondition {
public:
    ExitCondition(float range, int time);

    bool getExit() const;
    bool update(float input);

    void reset();

private:
    float mRange;
    int mTime;
    int mStartTime{-1};
    bool mDone{false};
};

}  // namespace riku
