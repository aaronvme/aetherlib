// Ported from FBLib - https://github.com/jiazegao/FBLib
// Config moved to namespace scope (adapted for riku): a nested struct's
// default member initializers can't be used in a default argument of the
// enclosing class, so the original did not compile.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "pros/motors.hpp"

#include "riku/Util/Util.hpp"

namespace riku {

// Generic N-motor jam detection with automatic reversal. Game-agnostic: motors
// are added by pointer, and game-specific "not a jam right now" logic is
// injected via Config::suppress. update() never blocks.

struct JamDetectorConfig {
    uint32_t jamTriggerMs{150};       // continuous stall before unjamming
    uint32_t reverseDurationMs{250};  // how long to run in reverse
    uint32_t settleDurationMs{150};   // detection pause after a reversal

    /// Above this, motors are held at zero and detection is skipped.
    float maxTempC{55.0f};

    /// Efficiency band (percent) identifying a jam. A free motor and a healthy
    /// loaded motor both read high; one driven into a stuck object sits in a
    /// narrow low band. Below `low` means barely commanded, so not a jam.
    ///
    /// EMPIRICAL — from one robot's intake. Log get_efficiency() while
    /// deliberately jamming the mechanism before trusting these.
    float jamEfficiencyLowPct{7.0f};
    float jamEfficiencyHighPct{20.0f};

    /// Return true to skip detection and reset the jam timer. Empty = never.
    std::function<bool()> suppress{[]() { return false; }};
};

class JamDetector {
public:
    using Config = JamDetectorConfig;

    /// Normal — watching; Reversing — clearing a jam; Settling — detection paused
    enum class State { Normal, Reversing, Settling };

    explicit JamDetector(const Config& config = {});
    JamDetector(const std::vector<pros::Motor*>& motors, const Config& config = {});

    // Motor management. Do this before update() starts running.

    /// @param reverseScale  command multiplier while unjamming: -1 reverses,
    ///                      0 holds still, +1 keeps driving forward.
    /// @param monitored     whether this motor's efficiency feeds detection.
    void addMotor(pros::Motor* motor, float reverseScale = -1.0f, bool monitored = true);
    void clearMotors();
    std::size_t motorCount() const;

    // Commanding. Velocities are LATCHED and re-applied after a reversal.

    void setVelocities(const std::vector<float>& velocities);
    void setVelocity(std::size_t index, float velocity);
    void setAllVelocities(float velocity);
    void stop();
    float commandedVelocity(std::size_t index) const;

    /// Advance the state machine. Non-blocking; call every 10-20 ms.
    void update();

    // Control & status. Safe to call from a task other than the one updating.

    void enable(bool enabled);
    bool isEnabled() const;

    /// Empty std::function restores the default "never suppress".
    void setSuppressCondition(std::function<bool()> suppress);

    bool isUnjamming() const;
    bool isOverTemp() const;
    State state() const;
    uint32_t jamCount() const;
    void resetStats();

    /// Abort any reversal and clear the jam timer. Latched commands preserved.
    void reset();

    const Config& config() const;
    void setConfig(const Config& config);

private:
    struct MotorEntry {
        pros::Motor* motor{nullptr};
        float reverseScale{-1.0f};
        bool monitored{true};
        float command{0.0f};     // latched velocity (rpm)
    };

    void applyCommands();
    void applyReverseCommands();
    bool detectJam() const;

    /// Returns true if the thermal cutoff is active this tick.
    bool serviceThermalCutoff();

    std::vector<MotorEntry> mMotors;
    Config mConfig;

    std::atomic<State> mState{State::Normal};
    std::atomic<uint32_t> mStateStartMs{0};
    std::atomic<uint32_t> mJamStartMs{0};   // 0 = not stalling
    std::atomic<bool> mEnabled{true};
    std::atomic<bool> mOverTemp{false};
    std::atomic<uint32_t> mJamCount{0};
};

}  // namespace riku
