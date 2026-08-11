// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include <functional>

#include "riku/drivetrain.hpp"

namespace riku {

// ============================================================================
// FeedforwardGains — characterized drivetrain model
// ============================================================================
//
// A tank drivetrain's steady-state voltage is modelled as
//
//     V = kV*v + kA*a + kS*sign(v)
//
// with separate kA/kS terms for the linear and angular axes, because a chassis
// accelerating straight and a chassis accelerating into a turn fight different
// inertias and different amounts of scrub friction.
//
// Gains are obtained by fitting recorded (velocity, voltage) data — see
// FeedforwardController::setTelemetry() below, which dumps CSV to the SD card
// suitable for an offline least-squares fit.
//
// UNITS: kV, kA and kS are only meaningful in the units they were fitted in.
// Whatever length unit you fit in, `trackWidth` and every velocity passed to
// update() must use that same unit. The reference characterization this was
// ported from was fitted in metres, so velocities are m/s and rad/s and
// trackWidth is in metres.
//
// Output is in VOLTS, matching those same reference gains (kV ~= 5 volts per
// m/s). This is worth stating loudly because the reference implementation got
// it wrong: its controller desaturated against a hardcoded 12000.0, as though
// the gains produced millivolts. With volt-scale gains that test never fired,
// so the follower downstream re-clamped each side independently to +/-12 and
// multiplied by 1000 — which turned every nonzero output into full +/-12 V and
// silently discarded the controller's actual command. setMaxVoltage() here
// defaults to 12.0 for that reason; use DrivetrainVoltages::leftMillivolts()
// when you need to feed pros::Motor::move_voltage().
// ============================================================================

struct FeedforwardGains {
    float kV{0.0f};           // volts-per-velocity
    float kaStraight{0.0f};   // volts-per-linear-acceleration
    float kaTurn{0.0f};       // volts-per-angular-acceleration
    float ksStraight{0.0f};   // static friction to overcome when driving
    float ksTurn{0.0f};       // static friction to overcome when turning
    float kP{0.0f};           // proportional gain on velocity error
    float kI{0.0f};           // integral gain on velocity error

    /// Only accumulate the integral while |error| is below this. Keeps the
    /// integrator from winding up during large transients (e.g. the start of a
    /// motion) and lets it do its job — killing steady-state droop — near the
    /// setpoint. <= 0 disables the gate and always integrates.
    float integralErrorGate{0.0f};

    /// Hard clamp on the accumulated integral, in the same units as the
    /// integral term itself. <= 0 disables clamping.
    ///
    /// The reference implementation had no clamp at all and ran with a gate of
    /// 5000.0 — which, for velocities of a few m/s, never actually gated
    /// anything. That combination lets the integrator grow without bound if the
    /// drivetrain is ever unable to reach its target (pushing against a wall,
    /// against another robot, a stalled motor). Clamping is the fix.
    float integralClamp{0.0f};
};

// DrivetrainVoltages lives in Drivetrain.hpp.

// ============================================================================
// FeedforwardController
// ============================================================================
//
// Converts a desired chassis twist (linear velocity, angular velocity) into
// left/right motor voltages using the characterized model above, plus a
// per-side PI correction on measured velocity error, then desaturates so a
// saturated side does not silently distort the commanded curvature.
//
// This is an alternative to the PID-to-voltage motion path in Chassis, not a
// replacement. It is what a trajectory follower wants: the trajectory already
// says how fast to go, so the controller should predict the voltage rather
// than wait for error to build up.
//
// Not thread-safe — one instance per control loop.
// ============================================================================

// One update() tick, for offline kV/kA/kS fitting.
struct FeedforwardSample {
    float vTarget{0}, wTarget{0};
    float vLeftTarget{0}, vRightTarget{0};
    float vLeftMeasured{0}, vRightMeasured{0};
    float aLinear{0}, aAngular{0};
    float voltLeft{0}, voltRight{0};
    float dt{0};
};

class FeedforwardController {
public:
    /// @param gains            characterized model + PI gains
    /// @param trackWidth       wheel-to-wheel distance, in the SAME length
    ///                         unit the gains were fitted in (metres, if you
    ///                         fitted in metres)
    FeedforwardController(const FeedforwardGains& gains, float trackWidth);

    /// Compute motor voltages for one control tick.
    ///
    /// @param targetLinearVel   desired forward velocity
    /// @param targetAngularVel  desired yaw rate (CCW positive)
    /// @param measuredLeftVel   measured left-side velocity, same units
    /// @param measuredRightVel  measured right-side velocity, same units
    /// @param dt                elapsed time since the previous call, seconds
    /// @return left/right voltages in volts, already desaturated
    ///
    /// The reference implementation hardcoded dt = 0.01 in three places, so it
    /// silently produced wrong acceleration and integral terms whenever the
    /// caller's loop drifted off a perfect 10 ms tick. Pass the real elapsed
    /// time — pros task scheduling is not exact.
    DrivetrainVoltages update(float targetLinearVel, float targetAngularVel,
                              float measuredLeftVel, float measuredRightVel,
                              float dt);

    /// Clear integrators and acceleration history. Call before starting a new
    /// motion, otherwise the first tick differentiates against a stale target.
    void reset();

    void setGains(const FeedforwardGains& gains);
    const FeedforwardGains& getGains() const { return mGains; }

    void setTrackWidth(float trackWidth) { mTrackWidth = trackWidth; }
    float getTrackWidth() const { return mTrackWidth; }

    /// Saturation limit in volts. Default 12.0 (full V5 motor scale).
    /// Lower it to cap authority without rescaling the gains.
    void setMaxVoltage(float volts) { mMaxVoltage = volts; }
    float getMaxVoltage() const { return mMaxVoltage; }

    /// Empty by default. util/tuning.hpp has a hook that logs this to the SD
    /// card for offline kV/kA/kS fitting.
    void setTelemetry(std::function<void(const FeedforwardSample&)> callback);

private:
    std::function<void(const FeedforwardSample&)> mTelemetry;
    FeedforwardGains mGains;
    float mTrackWidth;
    float mMaxVoltage{12.0f};

    // Every one of these was left uninitialized by the reference
    // implementation — its constructor initialized only the gains, so the
    // first update() differentiated against and integrated from whatever
    // happened to be on the stack.
    float mPrevLinearVel{0.0f};
    float mPrevAngularVel{0.0f};
    float mPrevLeftError{0.0f};
    float mPrevRightError{0.0f};
    float mLeftIntegral{0.0f};
    float mRightIntegral{0.0f};

    /// Suppresses the acceleration term on the first tick after construction
    /// or reset(). Without this, differencing against a zeroed previous target
    /// reports an acceleration of target/dt — a spike of hundreds of m/s² that
    /// saturates both sides on tick one.
    bool mFirstUpdate{true};
};

}  // namespace riku
