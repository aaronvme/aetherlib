// Ported from LemLib - https://github.com/LemLib/LemLib (MIT)
#pragma once

#include "chad/util/settle.hpp"

namespace chad {

/// Error inside `range` for `time` ms, timed against the PROS clock.
/// A thin wrapper over SettleMonitor, which is where the logic and the tests
/// live; use SettleMonitor directly when you need speed, stall or timeout tests.
class ExitCondition {
public:
    ExitCondition(float range, int time);

    bool getExit() const;
    bool update(float input);

    void reset();

private:
    SettleMonitor mMonitor;
    bool mStarted{false};
};

}  // namespace chad
