#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "pros/rtos.hpp"

#include "chad/util/jam_detector.hpp"
#include "chad/util/util.hpp"

using namespace chad;

namespace {
constexpr float kTempHysteresisC = 2.0f;

constexpr float kCommandEpsilonRpm = 1.0f;
}

JamDetector::JamDetector(const Config& config)
    : mConfig(config), mStateStartMs(pros::millis()) {
    if (!mConfig.suppress) mConfig.suppress = []() { return false; };
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
    const std::size_t count = (velocities.size() < mMotors.size()) ? velocities.size() : mMotors.size();
    for (std::size_t i = 0; i < count; ++i) mMotors[i].command = velocities[i];

    if (!isUnjamming() && !mOverTemp.load()) applyCommands();
}

void JamDetector::setVelocity(std::size_t index, float velocity) {
    if (index >= mMotors.size()) return;
    mMotors[index].command = velocity;

    if (!isUnjamming() && !mOverTemp.load() && mMotors[index].motor != nullptr) {
        mMotors[index].motor->move_velocity(static_cast<std::int32_t>(velocity));
    }
}

void JamDetector::setAllVelocities(float velocity) {
    for (MotorEntry& entry : mMotors) entry.command = velocity;
    if (!isUnjamming() && !mOverTemp.load()) applyCommands();
}

void JamDetector::stop() {
    for (MotorEntry& entry : mMotors) entry.command = 0.0f;

    if (!isUnjamming()) {
        for (MotorEntry& entry : mMotors) {
            if (entry.motor != nullptr) entry.motor->brake();
        }
    }

    mJamStartMs.store(0);
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

        const float eff = static_cast<float>(entry.motor->get_efficiency());
        if (!std::isfinite(eff)) continue;

        if (eff >= mConfig.jamEfficiencyLowPct && eff <= mConfig.jamEfficiencyHighPct) return true;
    }
    return false;
}

bool JamDetector::serviceThermalCutoff() {
    float maxTemp = 0.0f;
    bool anyValid = false;

    for (const MotorEntry& entry : mMotors) {
        if (entry.motor == nullptr) continue;
        const float temp = static_cast<float>(entry.motor->get_temperature());
        if (!std::isfinite(temp)) continue;
        if (!anyValid || temp > maxTemp) maxTemp = temp;
        anyValid = true;
    }

    if (!anyValid) return mOverTemp.load();

    if (!mOverTemp.load()) {
        if (maxTemp > mConfig.maxTempC) {
            mOverTemp.store(true);

            for (MotorEntry& entry : mMotors) {
                if (entry.motor != nullptr) entry.motor->move_velocity(0);
            }
            mState.store(State::Normal);
            mStateStartMs.store(pros::millis());
            mJamStartMs.store(0);
            return true;
        }
        return false;
    }

    if (maxTemp > mConfig.maxTempC - kTempHysteresisC) {
        for (MotorEntry& entry : mMotors) {
            if (entry.motor != nullptr) entry.motor->move_velocity(0);
        }
        return true;
    }

    mOverTemp.store(false);
    applyCommands();
    return false;
}

void JamDetector::setTelemetry(std::function<void(const JamSample&)> callback) {
    mTelemetry = std::move(callback);
}

void JamDetector::update() {
    const uint32_t now = pros::millis();

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

    if (serviceThermalCutoff()) return;

    const bool suppressed = !mEnabled.load() || (mConfig.suppress && mConfig.suppress());
    if (suppressed) {
        if (mState.load() != State::Normal) {
            mState.store(State::Normal);
            mStateStartMs.store(now);
            applyCommands();
        }
        mJamStartMs.store(0);
        return;
    }

    switch (mState.load()) {
        case State::Normal: {
            if (detectJam()) {
                if (mJamStartMs.load() == 0) {
                    mJamStartMs.store(now == 0 ? 1 : now);
                } else if (now - mJamStartMs.load() >= mConfig.jamTriggerMs) {
                    applyReverseCommands();
                    mState.store(State::Reversing);
                    mStateStartMs.store(now);
                    mJamStartMs.store(0);
                    mJamCount.fetch_add(1);
                }
            } else {
                mJamStartMs.store(0);
            }
            break;
        }

        case State::Reversing: {
            if (now - mStateStartMs.load() >= mConfig.reverseDurationMs) {
                applyCommands();
                mState.store(State::Settling);
                mStateStartMs.store(now);
            }
            break;
        }

        case State::Settling: {
            if (now - mStateStartMs.load() >= mConfig.settleDurationMs) {
                mState.store(State::Normal);
                mStateStartMs.store(now);
                mJamStartMs.store(0);
            }
            break;
        }
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

    mState.store(State::Normal);
    mStateStartMs.store(pros::millis());
    mJamStartMs.store(0);

    if (wasUnjamming && !mOverTemp.load()) applyCommands();
}

const JamDetector::Config& JamDetector::config() const {
    return mConfig;
}

void JamDetector::setConfig(const Config& config) {
    mConfig = config;
    if (!mConfig.suppress) mConfig.suppress = []() { return false; };
}
