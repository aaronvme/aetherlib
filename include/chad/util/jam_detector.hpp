#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "pros/motors.hpp"

#include "chad/util/util.hpp"

namespace chad {
struct JamDetectorConfig {
    uint32_t jamTriggerMs{150};
    uint32_t reverseDurationMs{250};
    uint32_t settleDurationMs{150};

    float maxTempC{55.0f};

    float jamEfficiencyLowPct{7.0f};
    float jamEfficiencyHighPct{20.0f};

    std::function<bool()> suppress{[]() { return false; }};
};

struct JamSample {
    float efficiency[3]{};
    float temperature[3]{};
    float command[3]{};
    int state{0};
    uint32_t jamCount{0};
};

class JamDetector {
public:
    using Config = JamDetectorConfig;

    enum class State { Normal, Reversing, Settling };

    explicit JamDetector(const Config& config = {});
    JamDetector(const std::vector<pros::Motor*>& motors, const Config& config = {});

    void addMotor(pros::Motor* motor, float reverseScale = -1.0f, bool monitored = true);
    void clearMotors();
    std::size_t motorCount() const;

    void setVelocities(const std::vector<float>& velocities);
    void setVelocity(std::size_t index, float velocity);
    void setAllVelocities(float velocity);
    void stop();
    float commandedVelocity(std::size_t index) const;

    void update();

    void enable(bool enabled);
    bool isEnabled() const;

    void setSuppressCondition(std::function<bool()> suppress);

    bool isUnjamming() const;
    bool isOverTemp() const;
    State state() const;
    uint32_t jamCount() const;
    void resetStats();

    void reset();

    const Config& config() const;
    void setConfig(const Config& config);

    void setTelemetry(std::function<void(const JamSample&)> callback);

private:
    struct MotorEntry {
        pros::Motor* motor{nullptr};
        float reverseScale{-1.0f};
        bool monitored{true};
        float command{0.0f};
    };

    void applyCommands();
    void applyReverseCommands();
    bool detectJam() const;

    bool serviceThermalCutoff();

    std::vector<MotorEntry> mMotors;
    Config mConfig;
    std::function<void(const JamSample&)> mTelemetry;

    std::atomic<State> mState{State::Normal};
    std::atomic<uint32_t> mStateStartMs{0};
    std::atomic<uint32_t> mJamStartMs{0};
    std::atomic<bool> mEnabled{true};
    std::atomic<bool> mOverTemp{false};
    std::atomic<uint32_t> mJamCount{0};
};
}
