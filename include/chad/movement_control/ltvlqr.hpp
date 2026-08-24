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
    // Tracking authority, not grip -- see RAMSETEParams::maxLateralAccel.
    float maxLateralAccel{60.0f};
    float targetTolerance{1.0f};
    float headingTolerance{4.0f};
    // Below this the robot has stopped and cannot improve its heading further.
    float stallSpeed{1.5f};
    bool  useVelocityProfile{true};
    float updatePeriodMs{10.0f};

    // Volts per second on each side. 0 disables. Measured worse than leaving it
    // off; see SlewLimiter.
    float slewRisePerSec{0.0f};
    float slewFallPerSec{0.0f};

    // Close the loop on wheel velocity rather than treat voltage as speed.
    // Gains are in METRES, matching FeedforwardController, and were fitted
    // against the simulator -- refit on hardware. kP under 4 times out.
    bool useVelocityController{false};
    FeedforwardGains ffGains{.kV = 6.2f, .ksStraight = 0.3f, .kP = 12.0f,
                             .integralClamp = 2.0f};
};

/// DEPRECATED -- kept for completeness, not recommended for competition use.
/// moveRAMSETE with RAMSETEParams::useVelocityController measured equal or
/// better on time and cross-track at every setting tried, and this duplicates
/// the profile builder, reference selector and settle logic that already live in
/// Chassis. Its one remaining edge is 2-3 degrees of final heading. Prefer
/// moveRAMSETE; follow it with turnToHeading when the final heading matters.
///
/// The idea it demonstrates is still the interesting part: RAMSETE's b and zeta
/// come from a Lyapunov argument that never prices the input, so they need
/// retuning per path, whereas LqrWeights are a cost in real units.
///
/// Blocking only -- cancel() is for another task, and one instance cannot run
/// two paths at once.
class LtvLqrFollower {
public:
    LtvLqrFollower(Chassis& chassis, Drivetrain& drivetrain);

    void followPath(const std::vector<Pose>& path, const LtvLqrParams& params,
                    float timeoutMs = 0.0f);
    void cancel() { mCancel = true; }
    bool isRunning() const { return mRunning; }

    /// Last run, inches.
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

}  // namespace chad
