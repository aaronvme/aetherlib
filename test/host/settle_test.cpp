#include "chad/movement_control/settle_spec.hpp"
#include "chad/util/settle.hpp"
#include "chad/util/step_response.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include "chad_test.hpp"

using namespace chad;

namespace {
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

SettleSpec basicSpec() {
    SettleSpec s;
    s.errorTolerance = 1.0f;
    s.holdMs = 0;
    s.timeoutMs = 0;
    s.stallSpeed = 0.0f;
    s.stallHoldMs = 0;
    s.graceMs = 0;
    return s;
}
}  // namespace

CHAD_TEST(settle, exitsImmediatelyWithoutHold) {
    SettleMonitor m{basicSpec()};
    m.reset(1000);
    CHECK(m.update(1000, 5.0f, 10.0f) == SettleState::Running);
    CHECK(m.update(1010, 0.5f, 10.0f) == SettleState::Settled);
    CHECK(m.done());
    CHECK(m.reachedTarget());
}

CHAD_TEST(settle, holdMustBeContinuous) {
    SettleSpec s = basicSpec();
    s.holdMs = 100;
    SettleMonitor m{s};
    m.reset(0);
    CHECK(m.update(0, 0.5f, 0.0f) == SettleState::Running);
    CHECK(m.update(50, 0.5f, 0.0f) == SettleState::Running);
    CHECK(m.update(60, 5.0f, 0.0f) == SettleState::Running);
    CHECK(m.update(120, 0.5f, 0.0f) == SettleState::Running);
    CHECK(m.update(219, 0.5f, 0.0f) == SettleState::Running);
    CHECK(m.update(220, 0.5f, 0.0f) == SettleState::Settled);
}

CHAD_TEST(settle, speedToleranceGatesArrival) {
    SettleSpec s = basicSpec();
    s.speedTolerance = 2.0f;
    SettleMonitor m{s};
    m.reset(0);
    CHECK(m.update(10, 0.2f, 20.0f) == SettleState::Running);
    CHECK(m.update(20, 0.2f, 1.0f) == SettleState::Settled);
}

CHAD_TEST(settle, timeoutFires) {
    SettleSpec s = basicSpec();
    s.timeoutMs = 500;
    SettleMonitor m{s};
    m.reset(100);
    CHECK(m.update(500, 40.0f, 20.0f) == SettleState::Running);
    CHECK(m.update(600, 40.0f, 20.0f) == SettleState::TimedOut);
    CHECK(m.done());
    CHECK(!m.reachedTarget());
}

CHAD_TEST(settle, toleranceBeatsTimeoutOnTheSameTick) {
    SettleSpec s = basicSpec();
    s.timeoutMs = 100;
    SettleMonitor m{s};
    m.reset(0);
    CHECK(m.update(100, 0.1f, 0.0f) == SettleState::Settled);
}

CHAD_TEST(settle, stallEndsAHopelessMotion) {
    SettleSpec s = basicSpec();
    s.timeoutMs = 100000;
    s.stallSpeed = 1.0f;
    s.stallHoldMs = 200;
    s.graceMs = 0;
    SettleMonitor m{s};
    m.reset(0);
    CHECK(m.update(50, 30.0f, 0.2f) == SettleState::Running);
    CHECK(m.update(249, 30.0f, 0.2f) == SettleState::Running);
    CHECK(m.update(250, 30.0f, 0.2f) == SettleState::Stalled);
    CHECK(!m.reachedTarget());
}

CHAD_TEST(settle, graceSuppressesTheStandingStart) {
    SettleSpec s = basicSpec();
    s.stallSpeed = 1.0f;
    s.stallHoldMs = 50;
    s.graceMs = 300;
    SettleMonitor m{s};
    m.reset(0);
    for (uint32_t t = 0; t <= 290; t += 10) {
        CHECK(m.update(t, 30.0f, 0.0f) == SettleState::Running);
    }
    m.update(300, 30.0f, 0.0f);
    CHECK(m.update(360, 30.0f, 0.0f) == SettleState::Stalled);
}

CHAD_TEST(settle, movingAgainClearsTheStallTimer) {
    SettleSpec s = basicSpec();
    s.stallSpeed = 1.0f;
    s.stallHoldMs = 100;
    s.graceMs = 0;
    SettleMonitor m{s};
    m.reset(0);
    m.update(10, 30.0f, 0.1f);
    m.update(80, 30.0f, 5.0f);
    CHECK(m.update(150, 30.0f, 0.1f) == SettleState::Running);
    CHECK(m.update(250, 30.0f, 0.1f) == SettleState::Stalled);
}

CHAD_TEST(settle, earlyExitTakesPrecedence) {
    SettleSpec s = basicSpec();
    s.errorTolerance = 0.5f;
    s.earlyExitRange = 6.0f;
    s.holdMs = 500;
    SettleMonitor m{s};
    m.reset(0);
    CHECK(m.update(10, 10.0f, 40.0f) == SettleState::Running);
    CHECK(m.update(20, 5.0f, 40.0f) == SettleState::EarlyExit);
    CHECK(m.reachedTarget());
}

CHAD_TEST(settle, deadSensorNeverCountsAsArrival) {
    SettleSpec s = basicSpec();
    s.timeoutMs = 1000;
    SettleMonitor m{s};
    m.reset(0);
    CHECK(m.update(10, kNan, 0.0f) == SettleState::Running);
    CHECK(m.update(20, kNan, 0.0f) == SettleState::Running);
    SettleSpec gated = basicSpec();
    gated.speedTolerance = 1.0f;
    SettleMonitor m2{gated};
    m2.reset(0);
    CHECK(m2.update(10, 0.1f, kNan) == SettleState::Running);
}

CHAD_TEST(settle, stateIsStickyOnceDone) {
    SettleMonitor m{basicSpec()};
    m.reset(0);
    CHECK(m.update(10, 0.1f, 0.0f) == SettleState::Settled);
    CHECK(m.update(20, 500.0f, 90.0f) == SettleState::Settled);
}

CHAD_TEST(settle, survivesMillisWraparound) {
    SettleSpec s = basicSpec();
    s.holdMs = 100;
    s.timeoutMs = 1000;
    SettleMonitor m{s};
    const uint32_t start = 0xFFFFFF00u;
    m.reset(start);
    CHECK(m.update(start + 50, 0.5f, 0.0f) == SettleState::Running);
    CHECK(m.update(start + 150, 0.5f, 0.0f) == SettleState::Settled);
    CHECK_EQ(m.elapsedMs(start + 150), uint32_t{150});
}

CHAD_TEST(settle, resetReturnsToRunning) {
    SettleMonitor m{basicSpec()};
    m.reset(0);
    m.update(10, 0.0f, 0.0f);
    CHECK(m.done());
    m.reset(100);
    CHECK(!m.done());
    CHECK(m.state() == SettleState::Running);
}

// ============================================================================
// SettleInput: the stall signal is separate from the progress signal
// ============================================================================

CHAD_TEST(settle, stallReadsDriveSpeedNotProgress) {
    SettleSpec s = basicSpec();
    s.timeoutMs = 100000;
    s.stallSpeed = 0.5f;
    s.stallHoldMs = 200;
    s.graceMs = 0;
    SettleMonitor m{s};
    m.reset(0);
    // Rotating in place: no progress toward the point, wheels turning hard.
    for (uint32_t t = 0; t <= 600; t += 10) {
        CHECK(m.update(t, SettleInput{30.0f, 0.0f, 20.0f}) == SettleState::Running);
    }
    // Pinned: wheels stopped, error unchanged.
    m.update(610, SettleInput{30.0f, 0.0f, 0.05f});
    CHECK(m.update(810, SettleInput{30.0f, 0.0f, 0.05f}) == SettleState::Stalled);
    CHECK(!m.reachedTarget());
}

CHAD_TEST(settle, graceDoesNotBankTheRestTimer) {
    // The drive is at rest for the whole grace window. If the rest clock were
    // started inside the grace and only its FIRING gated, the first sample past
    // graceMs would fire at once -- 300 ms banked against a 200 ms hold.
    SettleSpec s = basicSpec();
    s.timeoutMs = 100000;
    s.stallSpeed = 0.5f;
    s.stallHoldMs = 200;
    s.graceMs = 300;
    SettleMonitor m{s};
    m.reset(0);
    for (uint32_t t = 0; t <= 300; t += 10) {
        CHECK(m.update(t, SettleInput{24.0f, 0.0f, 0.0f}) == SettleState::Running);
    }
    CHECK(m.update(499, SettleInput{24.0f, 0.0f, 0.0f}) == SettleState::Running);
    CHECK(m.update(500, SettleInput{24.0f, 0.0f, 0.0f}) == SettleState::Stalled);

    // The same grace gates the arrival-at-rest path, or a motion that starts
    // inside its own target is declared arrived before it has moved at all.
    SettleMonitor m2{s};
    m2.reset(0);
    for (uint32_t t = 0; t < 300; t += 10) {
        CHECK(m2.update(t, SettleInput{24.0f, 0.0f, 0.0f, true}) == SettleState::Running);
    }
    CHECK(m2.update(300, SettleInput{24.0f, 0.0f, 0.0f, true}) == SettleState::Settled);
}

CHAD_TEST(settle, stoppingInsideItsOwnTargetIsArrivalNotAStall) {
    // A curved path ends inside its position tolerance but off its final
    // tangent, with the profile already at rest. The folded error is outside
    // tolerance, so atTarget is the only thing separating this from a genuine
    // stall -- and calling it a stall makes lastMotionArrived() false for a path
    // that arrived, stallHoldMs late.
    SettleSpec s = basicSpec();
    s.timeoutMs = 5000;
    s.stallSpeed = 1.5f;
    s.stallHoldMs = 500;
    s.graceMs = 300;

    SettleMonitor arrived{s};
    arrived.reset(0);
    CHECK(arrived.update(400, SettleInput{9.0f, 0.0f, 40.0f, true}) == SettleState::Running);
    CHECK(arrived.update(410, SettleInput{9.0f, 0.0f, 0.05f, true}) == SettleState::Settled);
    CHECK(arrived.reachedTarget());

    // The same sample with the primary target NOT met is a stall, and it still
    // has to hold for stallHoldMs before it says so.
    SettleMonitor stalled{s};
    stalled.reset(0);
    CHECK(stalled.update(410, SettleInput{9.0f, 0.0f, 0.05f, false}) == SettleState::Running);
    CHECK(stalled.update(909, SettleInput{9.0f, 0.0f, 0.05f, false}) == SettleState::Running);
    CHECK(stalled.update(910, SettleInput{9.0f, 0.0f, 0.05f, false}) == SettleState::Stalled);
    CHECK(!stalled.reachedTarget());
}

CHAD_TEST(settle, fullWindowIgnoresEveryTestButTheClock) {
    // Step-response capture. Anything that ends the motion early truncates the
    // trace, and the settle time and crossing count are then measured over a
    // window the operator did not ask for.
    SettleSpec s = basicSpec();
    s.timeoutMs = 1500;
    s.earlyExitRange = 6.0f;
    s.holdMs = 0;
    s.stallSpeed = 1.0f;
    s.stallHoldMs = 100;
    s.graceMs = 0;
    s.runFullWindow = true;
    SettleMonitor m{s};
    m.reset(0);
    for (uint32_t t = 0; t < 1500; t += 10) {
        CHECK(m.update(t, SettleInput{0.0f, 0.0f, 0.0f, true}) == SettleState::Running);
    }
    CHECK(m.update(1500, SettleInput{0.0f, 0.0f, 0.0f, true}) == SettleState::TimedOut);
}

CHAD_TEST(settle, holdLongerThanStallStillSettles) {
    // Inside tolerance the robot is stopped on purpose. Counting that as a
    // stall would mean a hold above stallHoldMs could never arrive.
    SettleSpec s = basicSpec();
    s.holdMs = 400;
    s.stallSpeed = 1.0f;
    s.stallHoldMs = 100;
    s.graceMs = 0;
    SettleMonitor m{s};
    m.reset(0);
    for (uint32_t t = 0; t < 400; t += 10) {
        CHECK(m.update(t, SettleInput{0.2f, 0.0f, 0.0f}) == SettleState::Running);
    }
    CHECK(m.update(400, SettleInput{0.2f, 0.0f, 0.0f}) == SettleState::Settled);
}

CHAD_TEST(settle, aRunningReferenceSuspendsBothRestTests) {
    // A reference that is legitimately stopped mid-motion is neither arrived nor
    // stalled, however long the wheels sit still -- even sitting inside
    // tolerance, because the clock still owns the motion.
    SettleSpec s = basicSpec();
    s.stallSpeed = 1.0f;
    s.stallHoldMs = 100;
    s.graceMs = 0;
    SettleMonitor m{s};
    m.reset(0);
    for (uint32_t t = 0; t <= 1000; t += 10) {
        CHECK(m.update(t, SettleInput{0.2f, 0.0f, 0.0f, true, false}) ==
              SettleState::Running);
    }
    // The reference runs out with the drive stopped and short of the target.
    m.update(1010, SettleInput{30.0f, 0.0f, 0.0f, false, true});
    CHECK(m.update(1110, SettleInput{30.0f, 0.0f, 0.0f, false, true}) ==
          SettleState::Stalled);
}

CHAD_TEST(settle, deadDriveSensorIsNotAStall) {
    SettleSpec s = basicSpec();
    s.stallSpeed = 1.0f;
    s.stallHoldMs = 100;
    s.graceMs = 0;
    SettleMonitor m{s};
    m.reset(0);
    for (uint32_t t = 0; t <= 500; t += 10) {
        CHECK(m.update(t, SettleInput{30.0f, 0.0f, kNan}) == SettleState::Running);
    }
}

CHAD_TEST(settle, inToleranceLeadsSettledByTheHold) {
    SettleSpec s = basicSpec();
    s.holdMs = 100;
    SettleMonitor m{s};
    m.reset(0);
    CHECK(m.update(0, 10.0f, 0.0f) == SettleState::Running);
    CHECK(!m.inTolerance());
    CHECK(m.update(10, 0.5f, 0.0f) == SettleState::Running);
    CHECK(m.inTolerance());
    CHECK(m.update(110, 0.5f, 0.0f) == SettleState::Settled);
}

CHAD_TEST(settle, outcomesAreDistinguishable) {
    // A routine that cannot tell these apart cannot decide anything about them.
    SettleSpec s = basicSpec();
    s.timeoutMs = 200;
    s.stallSpeed = 1.0f;
    s.stallHoldMs = 50;
    s.graceMs = 0;

    SettleMonitor arrived{s};
    arrived.reset(0);
    CHECK(arrived.update(10, SettleInput{0.4f, 0.0f, 0.0f}) == SettleState::Settled);
    CHECK(arrived.reachedTarget());

    SettleSpec chaining = s;
    chaining.earlyExitRange = 6.0f;
    SettleMonitor chained{chaining};
    chained.reset(0);
    CHECK(chained.update(10, SettleInput{4.0f, 40.0f, 40.0f}) == SettleState::EarlyExit);
    CHECK(chained.reachedTarget());

    SettleMonitor timedOut{s};
    timedOut.reset(0);
    CHECK(timedOut.update(100, SettleInput{40.0f, 10.0f, 30.0f}) == SettleState::Running);
    CHECK(timedOut.update(200, SettleInput{40.0f, 10.0f, 30.0f}) == SettleState::TimedOut);
    CHECK(!timedOut.reachedTarget());

    SettleMonitor stalled{s};
    stalled.reset(0);
    CHECK(stalled.update(10, SettleInput{40.0f, 0.0f, 0.0f}) == SettleState::Running);
    CHECK(stalled.update(60, SettleInput{40.0f, 0.0f, 0.0f}) == SettleState::Stalled);
    CHECK(!stalled.reachedTarget());
}

// ============================================================================
// RateEstimator
// ============================================================================

CHAD_TEST(settle, rateConvergesOnASteadySlope) {
    RateEstimator r{0.2f};
    float value = 24.0f;
    for (int i = 0; i < 400; ++i) {
        value -= 0.12f;                  // 12 in/s at a 10 ms tick
        r.update(value, 0.01f);
    }
    CHECK_NEAR(r.rate(), -12.0f, 0.01f);
}

CHAD_TEST(settle, rateIgnoresTheFirstSample) {
    // Differencing against a zeroed previous value would report 24/0.01 here.
    RateEstimator r;
    CHECK_EQ(r.update(24.0f, 0.01f), 0.0f);
}

CHAD_TEST(settle, rateAveragesOutZeroMeanJitter) {
    RateEstimator r{0.1f};
    const float jitter[8] = {0.4f, -0.3f, 0.2f, -0.5f, 0.35f, -0.15f, 0.05f, -0.2f};
    for (int i = 0; i < 800; ++i) {
        r.update(10.0f + jitter[i % 8], 0.01f);
    }
    CHECK(std::fabs(r.rate()) < 5.0f);   // raw differences here are +-90 in/s
}

CHAD_TEST(settle, aOneTickJumpIsADiscontinuityNotASpeed) {
    // A settle error that steps 2 in over one 10 ms tick is a relocalisation, or
    // a setpoint that moved. Differentiated it reads -20 in/s off a robot that
    // is standing still, and then blocks the speed test it feeds.
    RateEstimator r;
    for (int i = 0; i < 200; ++i) r.update(2.0f, 0.01f);
    CHECK_NEAR(r.update(0.0f, 0.01f), 0.0f, 1e-6);
    // A change that persists into a second sample IS a speed.
    r.update(-2.0f, 0.01f);
    CHECK(r.rate() < -10.0f);

    // And the gate is optional, so the old behaviour is still reachable.
    RateEstimator raw;
    raw.setJumpFactor(0.0f);
    for (int i = 0; i < 200; ++i) raw.update(2.0f, 0.01f);
    CHECK_NEAR(raw.update(0.0f, 0.01f), -20.0f, 1e-3);
}

CHAD_TEST(settle, rateRejectsBadInput) {
    RateEstimator r;
    r.update(0.0f, 0.01f);
    r.update(1.0f, 0.01f);
    const float before = r.rate();
    CHECK(before > 0.0f);
    CHECK_EQ(r.update(kNan, 0.01f), before);
    CHECK_EQ(r.update(2.0f, 0.0f), before);
    r.reset();
    CHECK_EQ(r.rate(), 0.0f);
}

// ============================================================================
// settleSpecFor / outcomeFor / the floor / the drain: the motion loop's exit
// decision, hoisted out of chassis.cpp so it can be tested at all.
// ============================================================================

CHAD_TEST(settleSpec, aTurnToleranceIsConvertedToRadians) {
    // The monitor is fed radians. Left in VEX degrees, a 2 degree tolerance
    // becomes 2 radians -- 114 degrees -- and every turn "arrives" at once.
    MotionSettleParams req;
    req.type = MotionType::Heading;
    req.timeout = 2000.0f;
    req.turnHeadingParams.targetTolerance = 2.0f;
    SettleConfig cfg;
    cfg.settleSpeedDeg = 30.0f;
    cfg.settleSpeedIn = 2.0f;          // must not leak into a turn
    const SettleSpec s = settleSpecFor(req, cfg);
    CHECK_NEAR(s.errorTolerance, degToRad(2.0f), 1e-6);
    CHECK_NEAR(s.speedTolerance, degToRad(30.0f), 1e-6);
    CHECK_EQ(s.timeoutMs, uint32_t{2000});
}

CHAD_TEST(settleSpec, aLateralToleranceStaysInInches) {
    MotionSettleParams req;
    req.type = MotionType::Point;
    req.movePointParams.targetTolerance = 1.5f;
    req.movePointParams.earlyExitRange = 6.0f;
    SettleConfig cfg;
    cfg.settleSpeedIn = 2.0f;
    cfg.settleSpeedDeg = 30.0f;        // must not leak into a lateral motion
    const SettleSpec s = settleSpecFor(req, cfg);
    CHECK_NEAR(s.errorTolerance, 1.5f, 1e-6);
    CHECK_NEAR(s.earlyExitRange, 6.0f, 1e-6);
    CHECK_NEAR(s.speedTolerance, 2.0f, 1e-6);
    CHECK_EQ(s.timeoutMs, uint32_t{0});   // no timeout requested
}

CHAD_TEST(settleSpec, aFollowerOverridesTheRobotWideHoldAndStall) {
    MotionSettleParams req;
    req.type = MotionType::RAMSETE;
    req.followerParams = {3.0f, 1.5f, 0u, true};
    SettleConfig cfg;
    cfg.settleTimeMs = 110;            // the hold that turned a 3.2 s path into 9 s
    cfg.stallSpeedIn = 0.5f;
    const SettleSpec s = settleSpecFor(req, cfg);
    CHECK_NEAR(s.errorTolerance, 3.0f, 1e-6);
    CHECK_EQ(s.holdMs, uint32_t{0});
    CHECK_NEAR(s.stallSpeed, 1.5f, 1e-6);

    // A follower stallSpeed of 0 means "use the robot-wide value", not "off".
    MotionSettleParams unset = req;
    unset.followerParams.stallSpeed = 0.0f;
    CHECK_NEAR(settleSpecFor(unset, cfg).stallSpeed, 0.5f, 1e-6);
}

CHAD_TEST(settleSpec, aTrajectoryThatDoesNotHoldIsNotGatedOnSpeed) {
    // holdAtEnd == false is documented as exiting the instant the clock does.
    MotionSettleParams req;
    req.type = MotionType::Trajectory;
    req.followerParams = {1.0f, 1.5f, 0u, false};
    SettleConfig cfg;
    cfg.settleSpeedIn = 2.0f;
    CHECK_EQ(settleSpecFor(req, cfg).speedTolerance, 0.0f);
    req.followerParams.holdAtEnd = true;
    CHECK_NEAR(settleSpecFor(req, cfg).speedTolerance, 2.0f, 1e-6);
}

CHAD_TEST(settleSpec, fullWindowWithoutATimeoutWouldNeverEnd) {
    MotionSettleParams req;
    req.type = MotionType::Distance;
    req.moveDistParams.targetTolerance = 1.0f;
    req.moveDistParams.holdForTimeout = true;
    CHECK(!settleSpecFor(req, {}).runFullWindow);
    req.timeout = 1500.0f;
    CHECK(settleSpecFor(req, {}).runFullWindow);
    // And it is opt-in.
    req.moveDistParams.holdForTimeout = false;
    CHECK(!settleSpecFor(req, {}).runFullWindow);
}

CHAD_TEST(settleSpec, eachSettleStateMapsToItsOwnOutcome) {
    CHECK(outcomeFor(SettleState::Settled) == MotionOutcome::Arrived);
    CHECK(outcomeFor(SettleState::EarlyExit) == MotionOutcome::ChainedOut);
    CHECK(outcomeFor(SettleState::Stalled) == MotionOutcome::Stalled);
    CHECK(outcomeFor(SettleState::TimedOut) == MotionOutcome::TimedOut);
    // Running is not an ending. A default: arm folded it, and anything added
    // later, silently into TimedOut.
    CHECK(outcomeFor(SettleState::Running) == MotionOutcome::None);
}

CHAD_TEST(settleSpec, theOutputFloorNeverBeatsTheCeiling) {
    // minSpeed overrides the PID and the slew; it does not override maxSpeed.
    CHECK_NEAR(applyOutputFloor(3.0f, 40.0f, 127.0f), 40.0f, 1e-6);
    CHECK_NEAR(applyOutputFloor(-3.0f, 40.0f, 127.0f), -40.0f, 1e-6);
    CHECK_NEAR(applyOutputFloor(3.0f, 40.0f, 20.0f), 20.0f, 1e-6);
    CHECK_NEAR(applyOutputFloor(-3.0f, 40.0f, 20.0f), -20.0f, 1e-6);
    // A loop asking for nothing stays at nothing, and a floor of 0 is off.
    CHECK_NEAR(applyOutputFloor(0.0f, 40.0f, 127.0f), 0.0f, 1e-6);
    CHECK_NEAR(applyOutputFloor(3.0f, 0.0f, 127.0f), 3.0f, 1e-6);
    // Above the floor nothing changes.
    CHECK_NEAR(applyOutputFloor(90.0f, 40.0f, 127.0f), 90.0f, 1e-6);
}

CHAD_TEST(settleSpec, onlyAChainedOutMotionHoldsVoltageAndOnlyBriefly) {
    // The safety invariant: a drained queue must not leave voltage on. Chaining
    // is the one exception, it is bounded, and nothing else gets it.
    CHECK(drainAction(MotionOutcome::ChainedOut, 0) == DrainAction::Wait);
    CHECK(drainAction(MotionOutcome::ChainedOut, kChainGraceMs - 1) == DrainAction::Wait);
    CHECK(drainAction(MotionOutcome::ChainedOut, kChainGraceMs) == DrainAction::Stop);
    CHECK(drainAction(MotionOutcome::ChainedOut, 100000) == DrainAction::Stop);
    for (MotionOutcome o : {MotionOutcome::None, MotionOutcome::Arrived,
                            MotionOutcome::TimedOut, MotionOutcome::Stalled,
                            MotionOutcome::Cancelled, MotionOutcome::Invalid}) {
        CHECK(drainAction(o, 0) == DrainAction::Stop);
    }
}

CHAD_TEST(settleSpec, motionTypeOrderIsTheOneTelemetryRecorded) {
    // ChassisSample::type logs the raw int, so inserting a case rewrites every
    // CSV already on disk.
    CHECK_EQ(static_cast<int>(MotionType::None), 0);
    CHECK_EQ(static_cast<int>(MotionType::Distance), 1);
    CHECK_EQ(static_cast<int>(MotionType::Point), 2);
    CHECK_EQ(static_cast<int>(MotionType::Heading), 3);
    CHECK_EQ(static_cast<int>(MotionType::TurnPoint), 4);
    CHECK_EQ(static_cast<int>(MotionType::Arc), 5);
    CHECK_EQ(static_cast<int>(MotionType::Boomerang), 6);
    CHECK_EQ(static_cast<int>(MotionType::RAMSETE), 7);
    CHECK_EQ(static_cast<int>(MotionType::Trajectory), 8);
    CHECK_EQ(static_cast<int>(MotionType::Swing), 9);
}

// ============================================================================
// The rest test in the motion's own units, the floor's retirement, and the two
// invariants that decide whether anything is still commanding the drivetrain.
// This robot: 11.918 in track, 2.6738 in wheels, 450 rpm (config.cpp).
// ============================================================================

CHAD_TEST(settleSpec, aTurnsRestSignalIsItsTurnRateNotItsWheelSpeed) {
    const float track = 11.918f;
    const float w = degToRad(4.0f);            // rad/s, a turn still closing
    const float wheel = w * track * 0.5f;      // 0.416 in/s per side, opposed
    // The whole problem in one line: on the wheel surface a turn converging at
    // 4 deg/s is under the 0.5 in/s threshold that means "stopped".
    CHECK(wheel < 0.5f);
    CHECK_NEAR(restSpeed(MotionType::Heading, -wheel, wheel, track), w, 1e-5);
    CHECK_NEAR(restSpeed(MotionType::TurnPoint, wheel, -wheel, track), w, 1e-5);
    CHECK_NEAR(restSpeed(MotionType::Swing, 0.0f, 2.0f * wheel, track), w, 1e-5);
    // A motion that translates keeps the wheel-surface reading, which is the
    // point of measuring the wheels: a robot rotating in place makes no headway
    // toward a point yet is plainly not stalled.
    CHECK_NEAR(restSpeed(MotionType::Point, -wheel, wheel, track), wheel, 1e-6);
    CHECK_NEAR(restSpeed(MotionType::Distance, 3.0f, -1.0f, track), 3.0f, 1e-6);
    CHECK_NEAR(restSpeed(MotionType::RAMSETE, -1.0f, 0.5f, track), 1.0f, 1e-6);
    CHECK_EQ(restSpeed(MotionType::Heading, -wheel, wheel, 0.0f), 0.0f);
}

CHAD_TEST(settleSpec, aTurnStillClosingAtFourDegreesPerSecondIsNotAStall) {
    MotionSettleParams req;
    req.type = MotionType::Heading;
    req.timeout = 2000.0f;
    req.turnHeadingParams.targetTolerance = 1.5f;   // config.cpp
    SettleConfig cfg;                               // shipped defaults
    cfg.settleTimeMs = 110;
    const SettleSpec spec = settleSpecFor(req, cfg);
    CHECK_NEAR(spec.stallSpeed, degToRad(2.0f), 1e-6);

    const float track = 11.918f;
    const float wheel = degToRad(4.0f) * track * 0.5f;
    const float rest = restSpeed(req.type, -wheel, wheel, track);

    // 6 deg out and closing at 4 deg/s: 2.4 deg left when the window ends, so
    // nothing here is an arrival and only the rest test can end it.
    SettleMonitor m{spec};
    m.reset(0);
    for (uint32_t t = 0; t <= 900; t += 10) {
        const float err = degToRad(6.0f - 0.004f * static_cast<float>(t));
        CHECK(m.update(t, SettleInput{err, -degToRad(4.0f), rest, false, true}) ==
              SettleState::Running);
    }

    // A turn that has actually stopped still ends, 500 ms after it stops.
    const float stopped = restSpeed(req.type, -0.05f, 0.05f, track);
    CHECK(stopped < spec.stallSpeed);
    SettleMonitor stuck{spec};
    stuck.reset(0);
    SettleState st = SettleState::Running;
    uint32_t exitAt = 0;
    for (uint32_t t = 0; t <= 2000; t += 10) {
        st = stuck.update(t, SettleInput{degToRad(4.0f), 0.0f, stopped, false, true});
        if (st != SettleState::Running) { exitAt = t; break; }
    }
    CHECK(st == SettleState::Stalled);
    CHECK_EQ(exitAt, uint32_t{800});
}

CHAD_TEST(settleSpec, turningOffStallDetectionTurnsItOffOnTurnsToo) {
    MotionSettleParams req;
    req.type = MotionType::Swing;
    req.swingParams.targetTolerance = 2.0f;
    SettleConfig cfg;
    cfg.stallSpeedIn = 0.0f;                  // the robot-wide off switch
    CHECK_EQ(settleSpecFor(req, cfg).stallSpeed, 0.0f);
    cfg.stallSpeedIn = 0.5f;
    cfg.stallSpeedDeg = 0.0f;                 // off for turns alone
    CHECK_EQ(settleSpecFor(req, cfg).stallSpeed, 0.0f);
    req.type = MotionType::Distance;          // and inches are untouched
    CHECK_NEAR(settleSpecFor(req, cfg).stallSpeed, 0.5f, 1e-6);
}

CHAD_TEST(settleSpec, oneJitterSampleDoesNotRetireTheOutputFloor) {
    // The floor is what stops a tuned loop creeping out its timeout, and it goes
    // off for good on arrival so it cannot kick the robot back out of tolerance.
    // Retiring it on a single sample left the robot parked outside tolerance
    // with the one mechanism that would have finished it disabled.
    SettleSpec spec = basicSpec();
    spec.holdMs = 110;
    spec.timeoutMs = 3000;
    SettleMonitor m{spec};
    m.reset(0);

    bool retired = false;
    auto step = [&](uint32_t t, float err) {
        m.update(t, SettleInput{err, 0.0f, 0.0f, false, true});
        retired = retired || m.inToleranceMs(t) >= kFloorRetireMs;
    };

    // 1.5 in of error against a 1.0 in tolerance, with single samples of pose
    // jitter dipping inside it.
    for (uint32_t t = 0; t < 400; t += 10) step(t, (t % 70 == 0) ? 0.9f : 1.5f);
    CHECK(!retired);

    // Three consecutive samples inside is an arrival, and retires it.
    step(400, 0.9f);
    CHECK(!retired);
    step(410, 0.9f);
    CHECK(!retired);
    step(420, 0.9f);
    CHECK(retired);
    // Well inside the 110 ms hold, so the floor is off for the settle window it
    // was interfering with.
    CHECK(!m.done());
}

CHAD_TEST(settleSpec, onlyChainingLeavesTheDrivetrainCommanded) {
    CHECK(!brakeOnEnd(MotionOutcome::ChainedOut));
    for (MotionOutcome o : {MotionOutcome::None, MotionOutcome::Arrived,
                            MotionOutcome::TimedOut, MotionOutcome::Stalled,
                            MotionOutcome::Cancelled, MotionOutcome::Invalid}) {
        CHECK(brakeOnEnd(o));
    }
}

CHAD_TEST(settleSpec, theCompletionCounterNeverGoesBackwards) {
    // cancelMotion() publishes the last id it enqueued; the motion task then
    // publishes the id of the motion it was running, which is lower. Storing
    // that leaves isSettled() false until the next motion is queued -- and
    // isSettled() gates tank/arcade/curvature, so driver control goes dead and
    // waitUntilSettled() never returns.
    CHECK_EQ(publishCompleted(7u, 5u), 7u);
    CHECK_EQ(publishCompleted(5u, 7u), 7u);
    CHECK_EQ(publishCompleted(7u, 7u), 7u);

    // Signed compare, so the counter may wrap past 2^32 mid-match.
    const uint32_t nearTop = 0xFFFFFFFEu;
    CHECK(sequenceSettled(1u, nearTop));      // 1 is three ids past it
    CHECK(!sequenceSettled(nearTop, 1u));
    CHECK_EQ(publishCompleted(nearTop, 1u), 1u);
    CHECK(sequenceSettled(4u, 4u));
    CHECK(!sequenceSettled(3u, 4u));
}

// ============================================================================
// step_response.hpp -- the objective the autotuner minimises.
// ============================================================================

CHAD_TEST(settleSpec, theSettleBandIsWhatTheDrivetrainCanHold) {
    // 8 in step converging to 0.4 in of steady-state error. A 2%-of-step band is
    // 0.16 in, which this base cannot hold, so settleMs was the trace length for
    // every candidate and the 8.0-per-second settle weight ranked nothing.
    std::vector<float> t, e;
    for (uint32_t ms = 0; ms <= 1500; ms += 10) {
        t.push_back(static_cast<float>(ms));
        e.push_back(std::max(0.4f, 8.0f - 0.019f * static_cast<float>(ms)));
    }
    const StepWeights w;
    const StepMetrics tight = analyseStep(t, e, 8.0f, w);
    CHECK(tight.valid);
    CHECK(!tight.settledInBand);
    CHECK_NEAR(tight.settleMs, 1500.0f, 1.0);

    const StepMetrics real = analyseStep(t, e, 8.0f, w, 0.75f);
    CHECK(real.settledInBand);
    CHECK_NEAR(real.settleMs, 390.0f, 1.0);
    CHECK(real.cost < tight.cost - 8.0f);     // 1.11 s of settle, weighted 8.0

    // The band is a floor on the 2%, never a replacement for it: a 100 in step
    // holding 1 in has settled, and reading the band as the whole answer would
    // score it as never settling.
    std::vector<float> bigE;
    for (uint32_t ms = 0; ms <= 1500; ms += 10) {
        bigE.push_back(std::max(1.0f, 100.0f - 0.25f * static_cast<float>(ms)));
    }
    const StepMetrics big = analyseStep(t, bigE, 100.0f, w, 0.75f);
    CHECK(big.settledInBand);
    CHECK_NEAR(big.settleMs, 400.0f, 1.0);
    CHECK_NEAR(big.settleMs, analyseStep(t, bigE, 100.0f, w).settleMs, 1.0);
}
