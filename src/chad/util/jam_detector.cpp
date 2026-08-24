// Ported from FBLib - https://github.com/jiazegao/FBLib
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "pros/rtos.hpp"

#include "chad/util/jam_detector.hpp"
#include "chad/util/util.hpp"

using namespace chad;

namespace {

/// Thermal cutoff hysteresis (deg C). The motors must cool this far below
/// Config::maxTempC before commands are restored, otherwise a motor sitting
/// exactly at the limit would chatter between cut and restored every tick.
constexpr float kTempHysteresisC = 2.0f;

/// A latched command below this magnitude counts as "not running", so the
/// motor is excluded from jam detection.
constexpr float kCommandEpsilonRpm = 1.0f;

JamTimingConfig timingOf(const JamDetectorConfig& c) {
    return {c.jamTriggerMs, c.reverseDurationMs, c.settleDurationMs, c.graceMs};
}

}  // namespace

JamDetector::JamDetector(const Config& config) : mConfig(config) {
    if (!mConfig.suppress) mConfig.suppress = []() { return false; };
    mJsm.setConfig(timingOf(mConfig));
    mJsm.reset(pros::millis());
}

JamDetector::JamDetector(const std::vector<pros::Motor*>& motors, const Config& config)
    : JamDetector(config) {
    for (pros::Motor* motor : motors) addMotor(motor);
}

void JamDetector::addMotor(pros::Motor* motor, float reverseScale, bool monitored) {
    if (motor == nullptr) return;

    MotorEntry entry;
    entry.motor = motor;
    entry.reverseScale = reverseScale;
    entry.monitored = monitored;
    entry.command = 0.0f;
    mMotors.push_back(entry);
}

void JamDetector::clearMotors() {
    mMotors.clear();
    reset();
}

std::size_t JamDetector::motorCount() const {
    return mMotors.size();
}

void JamDetector::setVelocities(const std::vector<float>& velocities) {
    const bool wasActive = anyMonitoredActive();
    const std::size_t count = (velocities.size() < mMotors.size()) ? velocities.size() : mMotors.size();
    for (std::size_t i = 0; i < count; ++i) mMotors[i].command = velocities[i];
    if (!wasActive && anyMonitoredActive()) mJsm.noteMotionStart(pros::millis());

    // Latch only while unjamming — the state machine re-applies the new
    // commands when the reversal finishes.
    if (!isUnjamming() && !mOverTemp.load()) applyCommands();
}

void JamDetector::setVelocity(std::size_t index, float velocity) {
    if (index >= mMotors.size()) return;
    const bool wasActive = anyMonitoredActive();
    mMotors[index].command = velocity;
    if (!wasActive && anyMonitoredActive()) mJsm.noteMotionStart(pros::millis());

    if (!isUnjamming() && !mOverTemp.load() && mMotors[index].motor != nullptr) {
        mMotors[index].motor->move_velocity(static_cast<std::int32_t>(velocity));
    }
}

void JamDetector::setAllVelocities(float velocity) {
    const bool wasActive = anyMonitoredActive();
    for (MotorEntry& entry : mMotors) entry.command = velocity;
    if (!wasActive && anyMonitoredActive()) mJsm.noteMotionStart(pros::millis());
    if (!isUnjamming() && !mOverTemp.load()) applyCommands();
}

void JamDetector::stop() {
    for (MotorEntry& entry : mMotors) entry.command = 0.0f;

    if (!isUnjamming()) {
        for (MotorEntry& entry : mMotors) {
            if (entry.motor != nullptr) entry.motor->brake();
        }
    }

    // A stopped mechanism cannot be jammed.
    mJsm.clearJamTimer();
}

float JamDetector::commandedVelocity(std::size_t index) const {
    if (index >= mMotors.size()) return 0.0f;
    return mMotors[index].command;
}

void JamDetector::applyCommands() {
    for (MotorEntry& entry : mMotors) {
        if (entry.motor == nullptr) continue;
        entry.motor->move_velocity(static_cast<std::int32_t>(entry.command));
    }
}

void JamDetector::applyReverseCommands() {
    for (MotorEntry& entry : mMotors) {
        if (entry.motor == nullptr) continue;
        entry.motor->move_velocity(static_cast<std::int32_t>(entry.command * entry.reverseScale));
    }
}

bool JamDetector::detectJam() const {
    for (const MotorEntry& entry : mMotors) {
        if (entry.motor == nullptr || !entry.monitored) continue;
        if (std::fabs(entry.command) < kCommandEpsilonRpm) continue;

        // get_efficiency() returns PROS_ERR_F (INFINITY) for a disconnected
        // motor — a non-finite reading must never look like a jam.
        const float eff = static_cast<float>(entry.motor->get_efficiency());
        if (!std::isfinite(eff)) continue;

        // Empirical band — see Config::jamEfficiencyLowPct/HighPct. Above the
        // band the motor is doing useful work; below it the motor is not
        // really loaded (or is barely being driven), which is not a jam.
        if (eff >= mConfig.jamEfficiencyLowPct && eff <= mConfig.jamEfficiencyHighPct) return true;
    }
    return false;
}

bool JamDetector::anyMonitoredActive() const {
    for (const MotorEntry& entry : mMotors) {
        if (entry.monitored && std::fabs(entry.command) >= kCommandEpsilonRpm) return true;
    }
    return false;
}

bool JamDetector::serviceThermalCutoff() {
    float maxTemp = 0.0f;
    bool anyValid = false;

    for (const MotorEntry& entry : mMotors) {
        if (entry.motor == nullptr) continue;
        const float temp = static_cast<float>(entry.motor->get_temperature());
        if (!std::isfinite(temp)) continue;  // PROS_ERR_F from a disconnected motor
        if (!anyValid || temp > maxTemp) maxTemp = temp;
        anyValid = true;
    }

    if (!anyValid) return mOverTemp.load();

    if (!mOverTemp.load()) {
        if (maxTemp > mConfig.maxTempC) {
            mOverTemp.store(true);

            // Cut power and abort any unjam in progress.
            for (MotorEntry& entry : mMotors) {
                if (entry.motor != nullptr) entry.motor->move_velocity(0);
            }
            mJsm.reset(pros::millis());
            mState.store(State::Normal);
            return true;
        }
        return false;
    }

    // Already cut: hold at zero until the stack cools past the hysteresis band.
    if (maxTemp > mConfig.maxTempC - kTempHysteresisC) {
        for (MotorEntry& entry : mMotors) {
            if (entry.motor != nullptr) entry.motor->move_velocity(0);
        }
        return true;
    }

    // Cooled down — restore the latched commands. The donor left the motors
    // dead until the caller happened to call set_velocities() again.
    mOverTemp.store(false);
    applyCommands();
    return false;
}

// The donor ran the unjam inline (move_velocity(-cmd); delay(250);
// move_velocity(cmd); delay(150)), so a function documented as a 10 ms tick
// actually blocked its caller ~400 ms. update() instead drives an explicit
// state machine off pros::millis() and never blocks.

void JamDetector::setTelemetry(std::function<void(const JamSample&)> callback) {
    mTelemetry = std::move(callback);
}

void JamDetector::update() {
    const uint32_t now = pros::millis();

    // Emitted before the thermal/suppression early-returns, so every tick is
    // reported. A disconnected motor reads inf, which is worth seeing.
    if (mTelemetry) {
        JamSample s;
        for (std::size_t i = 0; i < 3 && i < mMotors.size(); ++i) {
            if (mMotors[i].motor == nullptr) continue;
            s.efficiency[i] = static_cast<float>(mMotors[i].motor->get_efficiency());
            s.temperature[i] = static_cast<float>(mMotors[i].motor->get_temperature());
            s.command[i] = mMotors[i].command;
        }
        s.state = static_cast<int>(mState.load());
        s.jamCount = mJamCount.load();
        mTelemetry(s);
    }

    // Thermal protection runs in every state and outranks everything else.
    if (serviceThermalCutoff()) return;

    // Suppression / disable: skip detection and clear the jam timer. If a
    // reversal is in flight, abort it and restore the commanded velocities —
    // otherwise the mechanism would be stranded running backwards.
    const bool suppressed = !mEnabled.load() || (mConfig.suppress && mConfig.suppress());
    if (suppressed) {
        if (mJsm.state() != State::Normal) {
            mJsm.reset(now);
            mState.store(State::Normal);
            applyCommands();
        }
        mJsm.clearJamTimer();
        return;
    }

    const State before = mJsm.state();
    // detectJam() only matters in Normal; skip the extra get_efficiency() calls
    // in Reversing/Settling, matching the pre-extraction tick cost.
    const bool jammed = (before == State::Normal) && detectJam();
    const State after = mJsm.update(now, jammed);
    mState.store(after);

    if (before != State::Reversing && after == State::Reversing) {
        applyReverseCommands();
        mJamCount.fetch_add(1);
    } else if (before == State::Reversing && after == State::Settling) {
        // Detection stays paused through Settling so the mechanism can spin
        // back up without immediately re-triggering on its own restart current.
        applyCommands();
    }
}

void JamDetector::enable(bool enabled) {
    mEnabled.store(enabled);
    if (!enabled) reset();
}

bool JamDetector::isEnabled() const {
    return mEnabled.load();
}

void JamDetector::setSuppressCondition(std::function<bool()> suppress) {
    mConfig.suppress = suppress ? suppress : std::function<bool()>([]() { return false; });
}

bool JamDetector::isUnjamming() const {
    return mState.load() != State::Normal;
}

bool JamDetector::isOverTemp() const {
    return mOverTemp.load();
}

JamDetector::State JamDetector::state() const {
    return mState.load();
}

uint32_t JamDetector::jamCount() const {
    return mJamCount.load();
}

void JamDetector::resetStats() {
    mJamCount.store(0);
}

void JamDetector::reset() {
    const bool wasUnjamming = isUnjamming();

    mJsm.reset(pros::millis());
    mState.store(State::Normal);

    // Restore the latched commands so an aborted reversal does not leave the
    // mechanism running backwards.
    if (wasUnjamming && !mOverTemp.load()) applyCommands();
}

const JamDetector::Config& JamDetector::config() const {
    return mConfig;
}

void JamDetector::setConfig(const Config& config) {
    mConfig = config;
    if (!mConfig.suppress) mConfig.suppress = []() { return false; };
    mJsm.setConfig(timingOf(mConfig));
}
