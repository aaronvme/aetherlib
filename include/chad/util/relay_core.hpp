#pragma once

namespace chad::tuning {

struct RelayFit {
    float ku{0.0f};          // ultimate gain, output units per error unit
    float tu{0.0f};          // ultimate period, seconds
    float amplitude{0.0f};   // measured half-amplitude of the limit cycle
    float kP{0.0f};
    float kI{0.0f};
    float kD{0.0f};
    int cycles{0};
    bool valid{false};
};

enum class RelayOutcome {
    Cycling,      // still running
    Fitted,
    RanAway,      // left the abort band instead of cycling
    NoCycle,      // never switched enough to measure a period
    DwellFloor,   // the period measured minHalfPeriodMs, not the robot
    Chatter,      // swing too small against the dead band to trust
};

/// Relay (Astrom-Hagglund) identification, with the plant left to the caller.
///
/// The dead band is not a nuisance to be minimised: a lag feeding an integrator
/// never reaches -180 degrees of phase, so the plant has no ultimate frequency
/// and the dead band is what picks the excited one. So `ku` and `tu` are a point
/// at -180 + asin(eps/a1) degrees, NOT the critical point Tyreus-Luyben is
/// defined at, and `minSwingRatio` is a phase budget rather than a noise margin.
///
/// `amplitude` is the measured peak, while the describing function wants the
/// first-harmonic amplitude a1, which is smaller -- 1.28x on an integrator-lag
/// plant, near the pi^2/8 of a triangle wave. So ku and kP come out about 22%
/// low. That is left alone as headroom, since PidAutoTuner refines from here,
/// but it is why the budget below is stated on the peak.
struct RelayIdentifyConfig {
    float hysteresis{0.0f};
    float amplitude{0.0f};
    float abort{0.0f};
    int cycles{4};
    float minHalfPeriodMs{60.0f};
    /// Peak swing in dead bands. The phase error is asin(1.28/minSwingRatio),
    /// so 3.0 is a 25 degree budget where the 1.3 it replaces was 80, at which
    /// the tuning rule describes nothing. Refusing here means the dead band is
    /// too wide for the relay output -- raise the output, not the tolerance.
    /// Not tighter than 3.0: a 20 degree budget also refuses a 1 degree dead
    /// band at the shipped 40 unit output, and a refusal that names its fix
    /// still beats silently tuning off a point 80 degrees away.
    float minSwingRatio{3.0f};
    float dwellSlack{1.2f};
    float kpFactor{0.45f};
    float tdDivisor{6.3f};
    float tiFactor{2.2f};
    bool includeIntegral{false};
};

class RelayIdentifier {
public:
    static constexpr int kMaxHalfPeriods = 32;

    explicit RelayIdentifier(const RelayIdentifyConfig& config);

    /// One tick. `elapsedMs` is time since the run began. Returns the relay
    /// output to command; zero once it stops.
    float update(float elapsedMs, float error);

    /// Ends the run on the caller's timeout, fitting whatever was collected.
    void finish();

    RelayOutcome outcome() const { return mOutcome; }
    const RelayFit& fit() const { return mFit; }
    int switches() const { return mSwitches; }

private:
    void computeFit();

    RelayIdentifyConfig mConfig;
    RelayFit mFit;
    RelayOutcome mOutcome{RelayOutcome::Cycling};
    float mHalfPeriods[kMaxHalfPeriods]{};
    float mPeaks[kMaxHalfPeriods]{};
    int mCount{0};
    float mSign{1.0f};
    float mPeak{0.0f};
    float mLastSwitchMs{-1.0f};
    int mSwitches{0};
};

/// Tyreus-Luyben off an identified point. Ziegler-Nichols (0.6, Tu/8, Tu/2)
/// rings on a drivetrain.
RelayFit relayGains(float ku, float tu, float amplitude, int cycles,
                    const RelayIdentifyConfig& config);

}  // namespace chad::tuning
