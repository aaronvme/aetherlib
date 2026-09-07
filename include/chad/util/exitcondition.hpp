#pragma once

namespace chad {
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
}
