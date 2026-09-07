#pragma once

#include <atomic>
#include <vector>

#include "chad/movement_control/feedforward.hpp"
#include "chad/movement_control/lqr.hpp"
#include "chad/util/slew.hpp"
#include "chad/util/util.hpp"

namespace chad {
class Chassis;
class Drivetrain;

struct LtvLqrParams {
    LqrWeights weights;
    float maxSpeed{127.0f};
    float minSpeed{12.0f};
    float maxAccel{50.0f};
    float maxLateralAccel{60.0f};
    float targetTolerance{1.0f};
    float headingTolerance{4.0f};

    float stallSpeed{1.5f};
    bool  useVelocityProfile{true};
    float updatePeriodMs{10.0f};

    float slewRisePerSec{0.0f};
    float slewFallPerSec{0.0f};

    bool useVelocityController{false};
    FeedforwardGains ffGains{.kV = 6.2f, .ksStraight = 0.3f, .kP = 12.0f,
                             .integralClamp = 2.0f};
};

class LtvLqrFollower {
public:
    LtvLqrFollower(Chassis& chassis, Drivetrain& drivetrain);

    void followPath(const std::vector<Pose>& path, const LtvLqrParams& params,
                    float timeoutMs = 0.0f);
    void cancel() { mCancel = true; }
    bool isRunning() const { return mRunning; }

    float lastPeakError() const { return mPeakError; }
    float lastFinalError() const { return mFinalError; }

private:
    Chassis& mChassis;
    Drivetrain& mDrivetrain;
    LqrGainSchedule mSchedule;
    FeedforwardController mFf;
    SlewLimiter mSlewLeft, mSlewRight;
    LqrWeights mBuiltWith{};
    float mBuiltDt{0.0f};

    std::atomic<bool> mRunning{false};
    std::atomic<bool> mCancel{false};
    float mPeakError{0.0f};
    float mFinalError{0.0f};
};
}
