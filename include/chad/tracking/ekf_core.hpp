#pragma once

#include <array>
#include <cstdint>

#include "chad/tracking/pose_estimator.hpp"
#include "chad/util/util.hpp"

namespace chad {

/// A field feature that returns a range but is not a wall, so a beam that
/// ranged it carries no information about where the wall is.
struct RangeObstacle {
    float x{0.0f};
    float y{0.0f};
    float radius{0.0f};
};

// ============================================================================
// EkfCore — the SE(2) filter numerics, with no hardware and no clock
// ============================================================================
//
// State is [x, y, theta] with a full 3x3 covariance. Odometry drives the
// prediction; three measurement families correct it:
//
//   * distance sensor range against a known field wall (1 dof each)
//   * IMU heading                                      (1 dof)
//   * MCL position                                     (2 dof)
//
// The EKF is a unimodal filter: it cannot represent "the robot is in one of two
// places", so it cannot recover from a kidnapping on its own. MCL can, and is
// wired in as a measurement precisely to supply that.
//
// Everything correctness-critical lives here so it can be exercised on the
// host. `EkfTracking` is this class driven by PROS devices and a PROS clock.
// ============================================================================

class EkfCore {
public:
    static constexpr int SENSOR_COUNT = MAX_DISTANCE_SENSORS;
    static constexpr int MAX_OBSTACLES = 24;
    /// 64 samples at a 10 ms cycle is 640 ms of rewind, ten times the worst
    /// smart-port latency.
    static constexpr int HISTORY = 64;

    struct Config {
        // Tags: [robot] phase 3 or 4 measures it and prints it -- paste it in.
        //       [yours] a fact about your hardware or CPU budget. Set it once.
        //       [model] a modelling choice or a sensor/statistics standard, not a
        //               property of your robot. Nothing to measure.
        //       [sim]   the VALUE came off simulator sweeps. Structure is sound,
        //               the number deserves checking on hardware.
        //       [gap]   nothing here measures it; see the bias comments.
        // --- process noise -------------------------------------------------
        // Odometry error grows with distance travelled, not with wall time, so
        // these are per-inch and per-radian rather than per-second. The floors
        // stop P collapsing to zero while parked, which would lock the filter
        // onto its own prediction and reject every real measurement.
        float vertNoiseProp{0.045f};      // sigma per inch of |dVert| [robot]
        float horizNoiseProp{0.045f};     // sigma per inch of |dHoriz| [model]
        float slipNoiseProp{0.035f};      // lateral sigma per inch driven [robot]
        float slipNoiseFloor{0.010f};     // lateral sigma per update [model]

        // Slip and a wrong wheel diameter are SYSTEMATIC: the error they leave
        // grows with distance driven, not with its square root. Charged per tick
        // as a white term, (prop*dVert)^2 sums to prop^2*sum(dVert^2), which for
        // many small ticks is far less than the real (prop*L)^2 -- so P collapses,
        // the filter turns overconfident, and the Mahalanobis gate starts
        // rejecting the correction it needs. These add a term that accumulates
        // linearly in L instead, as a bias decorrelating over biasDecorrIn.
        //
        // Sized against what phase 3 measures. measureOdomDrift reports the
        // out-and-back CLOSURE error per inch driven (0.0040/in on this robot),
        // which cancels a symmetric scale error by construction. 0.010/in is
        // 2.5x that residual, which is the margin a tape-measured wheel diameter
        // leaves. Larger swamps the measurement: at 0.06/in the range gate opens
        // to +/-7 inches after 40 inches of dead reckoning and another robot's
        // shin passes it.
        float vertBiasProp{0.010f};       // systematic along-track scale error [gap]
        float slipBiasProp{0.010f};       // systematic cross-track error [gap]
        float horizBiasProp{0.010f};      // ditto, with a lateral wheel fitted [gap]
        float biasDecorrIn{40.0f};        // inches over which that bias renews [gap]
        float thetaNoiseProp{0.020f};     // sigma per radian of |dTheta| [model]
        float thetaNoisePerInch{0.0015f}; // heading sigma per inch driven [model]
        float posNoiseFloor{0.015f};      // inches per update [model]
        float thetaNoiseFloor{0.0004f};   // radians per update [model]
        // Charged while the heading is UNMEASURED. With no IMU reading the
        // prediction assumes dTheta == 0, so heading is unobservable and its
        // error grows at whatever rate the robot is really turning -- the floor
        // above would declare a third of a degree through a 90 degree outage.
        // 1.0 rad^2/s reaches 81 degrees of sigma over a two second outage.
        float thetaUnmeasuredVarPerSec{1.0f};   // [model]

        // --- range-to-wall measurements ------------------------------------
        bool  useRangeUpdates{true};   // [yours]
        float rangeSigmaBase{0.60f};      // inches, fixed part [robot]
        float rangeSigmaProp{0.030f};     // fraction of the reading [robot]
        float rangeGateChi2{9.0f};        // 1 dof; 9 == 3 sigma [model]
        // Past this incidence a real ToF return is unreliable and the range
        // Jacobian blows up (1/cos), so the beam is worth more as noise than
        // as evidence.
        float maxIncidenceRad{1.15f};     // 66 degrees off the wall normal [model]
        float maxRangeIn{100.0f};   // [yours]
        float sentinelMm{9000.0f};        // at or above this, no return [yours]
        float minRangeMm{20.0f};   // [yours]
        int   minConfidence{10};   // [yours]
        // A ToF reading describes where the robot was when the beam left, not
        // where it is when the port delivers it. Fusing it against the present
        // pose is a BIAS of speed*latency -- 3.0 inches at 60 in/s and 50 ms --
        // and more beams do not average it away, so zero is the one value
        // guaranteed to be wrong.
        //
        // 30 ms is an estimate of (smart-port refresh + half the sensor's own
        // time-of-flight integration window), not a measurement: ~10 ms for the
        // 100 Hz VEXos device cycle plus ~20 ms of ToF integration whose reported
        // value is centred mid-window. Measure it before racing -- see
        // rangeLatencyMs in the notes at the top of src/gamespecific/config.cpp.
        float rangeLatencyMs{30.0f};   // [gap]

        // --- obstacle deadzones --------------------------------------------
        // Same cone-plus-range test MCL uses, but widened by the filter's own
        // position sigma: when the EKF is unsure where it is, it is also unsure
        // which beams a post could be spoiling, and should drop more of them.
        // Every use of the widened tolerance is monotone in that direction.
        //
        // A post in the cone is NOT enough on its own: the reading also has to
        // match the post surface. One that matches the wall instead says the
        // cone was the conservative over-estimate it is, and is kept. Dropping
        // those unconditionally blinded an 18 inch strip of wall per post.
        float obstacleConeFrac{0.22f};    // tan(12.5 deg), the V5 half-cone [model]
        float obstacleRangeTol{6.0f};     // inches, range match only [model]
        /// Extra tolerance in position sigmas. Widens the range match and the
        /// cone; the cone gets this part only, since obstacleRangeTol is slop on
        /// a range and the cone measures a perpendicular offset.
        float obstacleSigmaPad{2.0f};   // [model]
        /// Cap on the widened tolerance. Without it a lost filter deadzones
        /// every beam it owns and can never see its way back.
        float obstacleTolMaxIn{18.0f};    // [model]

        // --- IMU heading ---------------------------------------------------
        bool  useImuHeading{true};   // [yours]
        float imuSigmaRad{0.006f};   // [model]
        // Zero disables the gate. On a drivetrain whose only absolute heading
        // source is the IMU, gating it out leaves heading unobserved -- which is
        // why this is off by default despite being good statistics.
        float imuGateChi2{0.0f};   // [model]
        // Zero on purpose, and not for want of a measurement: the state's own
        // heading is propagated from THIS IMU's differences (OdomCore turns them
        // into OdomDelta::dTheta), so prediction and measurement carry the same
        // port delay and are already aligned. Only the differential latency
        // between the two would belong here, and it is one odometry tick at
        // most, in the direction that makes the measurement the fresher of the
        // pair. Fit an encoder-differential heading source and this stops being
        // true.
        float imuLatencyMs{0.0f};   // [model]

        // --- MCL position --------------------------------------------------
        bool  useMclUpdates{true};   // [yours]
        // The particle cloud's own spread understates its error once it has
        // converged, so scale it up and never let it claim better than the
        // floor. Trusting the raw spread makes the EKF chase MCL's jitter.
        float mclSigmaScale{1.6f};   // [sim]
        float mclSigmaFloor{1.2f};        // inches [sim]
        float mclGateChi2{11.8f};         // 2 dof; 11.8 == 3 sigma [model]
        // A kidnapped robot fails the gate forever, which is the one case where
        // gating is wrong. Count consecutive failures, then adjudicate the
        // disagreement against the distance readings rather than against the
        // cloud's self-report -- see rangesPreferMcl().
        int   mclResetCycles{10};   // [model]
        /// How much better MCL's position must explain the same beams than the
        /// filter's own state before a persistent disagreement is acted on, as a
        /// ratio of chi-square. 12 is residuals 3.5x smaller in sigmas. It is a
        /// ratio and not a chi-square threshold because both hypotheses are
        /// scored on the same readings with the same sigma, so what matters is
        /// which fits better and by how much, not the absolute fit.
        /// Swept against both rates in ekfReloc. Asymmetric on purpose: a
        /// missed relocation is retried on the next tick, a false one gates out
        /// the beams that would have argued against it. Inert below about 10 mm
        /// of range noise -- 1 and 40 score identically there.
        float mclRelocEvidenceRatio{12.0f};   // [sim]

        // --- output ---------------------------------------------------------
        bool  autoSync{true};             // write the fused pose into odometry [model]
        // Fraction of the correction applied per update. A hard write steps the
        // pose every cycle and every controller downstream differentiates it;
        // the filter's own state stays exact either way. Relocation bypasses it.
        float syncProp{0.12f};   // [model]
        /// INERT in the shipped configuration. OdomCore::Config::headingFromImu
        /// defaults true, which re-slaves the odometry pose heading to the raw
        /// IMU on the next 10 ms tick, so the blend this fraction computes is
        /// overwritten before anything reads it -- including after a relocation.
        /// Nothing in the library calls OdomTracking::setCoreConfig, and
        /// Chassis::getPose() returns the odometry pose, not the filter's. To
        /// make this live, set headingFromImu false via that call; the EKF's own
        /// state() carries the fused heading either way.
        float thetaSyncProp{0.20f};   // [model]
        float updatePeriodMs{10.0f};   // [model]
    };

    /// Per-beam outcome from the most recent cycle.
    enum class BeamState : std::uint8_t {
        Absent,     // no sensor in this slot
        NoReturn,   // out of range, below confidence, or no wall along the ray
        Deadzone,   // an obstacle owns this return
        Grazing,    // incidence too high to invert
        Rejected,   // failed the Mahalanobis gate
        Used,
    };

    /// V5 pros::Distance::get_confidence() reports 0..CONFIDENCE_MAX. A faulted
    /// port returns PROS_ERR (2147483647) instead, and this value scales the
    /// range variance, so anything outside the range is read as no confidence
    /// and the beam is dropped -- the same treatment sentinelMm gives a faulted
    /// get().
    static constexpr int CONFIDENCE_MAX = 63;
    /// Confidence at which the range sigma is exactly the bench-measured one.
    /// Above it the scale factor stays at 1: no confidence report entitles the
    /// filter to claim better than the hardware was measured to deliver.
    static constexpr int CONFIDENCE_NOMINAL = 32;

    /// One distance sensor read, exactly as the port reports it.
    struct Beam {
        bool  present{false};
        float mm{0.0f};
        int   confidence{0};   // 0..CONFIDENCE_MAX; validated, not trusted
    };

    /// Everything one cycle needs. No hardware, no clock.
    struct Inputs {
        OdomDelta delta{};
        bool hasHorizontalTracking{false};

        /// False when no IMU is fitted, or when the odometry layer served a held
        /// value because the read faulted. A hold is not a measurement.
        bool  headingValid{false};
        float headingRad{0.0f};

        std::array<Beam, SENSOR_COUNT> beams{};

        bool  mclValid{false};
        Pose  mclPose{};
        float mclCovXX{0.0f}, mclCovXY{0.0f}, mclCovYY{0.0f};
        /// Diagnostic only. Effective sample size measures weight DISPARITY, so
        /// it is high both when the cloud is lost (every particle equally wrong,
        /// and log-sum-exp with max subtraction flattens the weights) and when
        /// the cloud has no measurements at all. It is not a health metric in
        /// either direction and nothing here gates on it.
        float mclNeffFrac{0.0f};
    };

    EkfCore() = default;
    explicit EkfCore(const Config& config) : mConfig(config) {}

    const Config& config() const { return mConfig; }
    void setConfig(const Config& config) { mConfig = config; }

    void setMounts(const std::array<Pose, SENSOR_COUNT>& mounts) { mMounts = mounts; }
    const std::array<Pose, SENSOR_COUNT>& mounts() const { return mMounts; }

    /// Copies, so the caller's storage may go away. Beyond MAX_OBSTACLES the
    /// remainder is dropped rather than silently wrapping.
    void setObstacles(const RangeObstacle* obstacles, int count);

    /// State to `pose`, covariance to a diagonal, all history and diagnostics
    /// cleared. Use after a deliberate reposition, not to nudge the filter.
    void reset(const Pose& pose, float posSigma, float thetaSigma, std::uint32_t nowMs);

    /// One full cycle: predict, then every enabled measurement family.
    Pose step(std::uint32_t nowMs, const Inputs& in);

    Pose state() const { return {mState[0], mState[1], mState[2]}; }
    /// Row-major 3x3, ordered [x, y, theta].
    const float* covariance() const { return mP; }

    struct Uncertainty {
        float sigmaX{0.0f}, sigmaY{0.0f}, sigmaTheta{0.0f};
    };
    Uncertainty uncertainty() const;

    // ------------------------------------------------------------------------
    // Steps, individually callable so each can be pinned down on its own
    // ------------------------------------------------------------------------

    /// `headingValid` false means the odometry layer served a held heading, so
    /// delta.dTheta is zero whatever the robot did: heading goes unmeasured and
    /// is charged thetaUnmeasuredVarPerSec instead of the floor.
    void predict(const OdomDelta& delta, bool hasHorizontalTracking,
                 std::uint32_t nowMs, bool headingValid = true);
    void updateHeading(float headingRad, std::uint32_t nowMs);
    void updateRanges(const std::array<Beam, SENSOR_COUNT>& beams, std::uint32_t nowMs);
    void updateMcl(const Inputs& in);

    /// Scalar (1-dof) Kalman correction with a Joseph-form covariance update.
    /// H is the 1x3 row, `innovation` the residual (already angle-wrapped if it
    /// is an angle), R the measurement variance. gateChi2 <= 0 disables the
    /// Mahalanobis test. False means nothing was folded in.
    bool fuseScalar(const float H[3], float innovation, float R, float gateChi2);

    // ------------------------------------------------------------------------
    // Models, as pure functions of a pose so they can be differentiated
    // ------------------------------------------------------------------------

    /// Exact SE(2) mean propagation of a body-frame odometry tick.
    static Pose propagate(const Pose& pose, const OdomDelta& delta);

    /// d(propagate)/d(pose), row-major.
    static void processJacobian(const Pose& pose, const OdomDelta& delta, float F[9]);

    /// d(pose.transformBy(t))/d(pose), row-major. Used to carry a Jacobian
    /// evaluated at a rewound pose back onto the current state.
    static void transformJacobian(const Pose& pose, const Transform2d& t, float J[9]);

    struct RangePrediction {
        BeamState state{BeamState::NoReturn};
        float range{0.0f};
        float cosIncidence{0.0f};
        float H[3]{0.0f, 0.0f, 0.0f};
        float sx{0.0f}, sy{0.0f};        // beam origin, world frame
        float rayCos{1.0f}, raySin{0.0f};
        float nx{0.0f}, ny{0.0f};        // unit normal of the surface hit
        bool valid() const { return state == BeamState::Used; }
    };

    /// Predicted range to the nearest field wall for a sensor mounted at
    /// `mount` on a robot at `pose`, with the measurement Jacobian at `pose`.
    RangePrediction predictRange(const Pose& pose, const Pose& mount) const;

    /// True when an obstacle, not a wall, is what this beam ranged.
    bool obstacleOwnsReading(float sx, float sy, float rayCos, float raySin,
                             float readingIn, float wallRange) const;

    /// Chi-square of `beams` under two position hypotheses -- the filter's own
    /// state and `other` -- scored on exactly the beams both can predict, so the
    /// two numbers are comparable. `other`'s heading is ignored: it is a
    /// position hypothesis, and reusing the state's heading is what keeps this
    /// from becoming a three-degree-of-freedom question. Returns the number of
    /// beams scored; 0 means there was no evidence either way.
    int compareRangeFit(const std::array<Beam, SENSOR_COUNT>& beams,
                        const Pose& other, float* chi2Self, float* chi2Other) const;

    /// Does the range evidence prefer MCL's position to the filter's own, by
    /// mclRelocEvidenceRatio? This is what makes a relocation an inference from
    /// measurements instead of a report from the thing being doubted.
    bool rangesPreferMcl(const Inputs& in) const;

    // ------------------------------------------------------------------------
    // Diagnostics. A filter that silently rejects every measurement looks
    // identical from the outside to a healthy one, so all of it is counted.
    // ------------------------------------------------------------------------

    const std::array<BeamState, SENSOR_COUNT>& beamStates() const { return mBeamStates; }
    const std::array<float, SENSOR_COUNT>& innovations() const { return mInnovations; }
    int beamsUsed() const { return mBeamsUsed; }

    std::uint32_t rangeRejects() const { return mRangeRejects; }
    std::uint32_t headingRejects() const { return mHeadingRejects; }
    std::uint32_t mclRejects() const { return mMclRejects; }
    /// Beams discarded before the gate saw them, cumulative. Uninformative is
    /// not the same failure as mis-gated, but a filter blinded by a field full
    /// of posts is exactly as deaf as one whose gate is too tight, so all three
    /// are counted separately rather than not at all.
    std::uint32_t rangeNoReturns() const { return mRangeNoReturns; }
    std::uint32_t rangeDeadzoned() const { return mRangeDeadzoned; }
    std::uint32_t rangeGrazed() const { return mRangeGrazed; }
    /// Consecutive cycles in which beams returned and none survived the gate.
    int rangeRejectStreak() const { return mRangeRejectStreak; }
    /// Consecutive cycles in which a sensor was present and no beam was folded
    /// in, for ANY reason. This is the one that catches a permanent deadzone.
    int rangeBlindStreak() const { return mRangeBlindStreak; }
    int mclRejectStreak() const { return mMclRejectStreak; }
    int relocateCount() const { return mRelocateCount; }
    /// Times the covariance had to be clamped back to positive-semidefinite.
    /// Joseph form makes this unreachable in exact arithmetic; a nonzero count
    /// means float32 is losing the fight and P is worth logging.
    std::uint32_t covarianceRepairs() const { return mCovRepairs; }

    /// Set by a relocation, cleared by reading. The caller should land the whole
    /// correction at once rather than lerping toward it.
    bool consumeRelocation();

private:
    struct WallHit {
        bool  valid{false};
        float range{0.0f};
        float nx{0.0f}, ny{0.0f};
        float cosIncidence{0.0f};
    };
    WallHit castWalls(float sx, float sy, float rayCos, float raySin) const;

    /// Validated reading in inches, or false when the port did not deliver a
    /// range. Shared with the relocation adjudicator so one definition of "this
    /// beam is a measurement" serves both.
    bool beamReadingIn(const Beam& beam, float* readingIn) const;
    /// Range sigma in inches for that reading at that incidence.
    float rangeSigma(const Beam& beam, float readingIn, float cosIncidence) const;

    /// Force P symmetric with a positive diagonal and correlations in range.
    /// True when it had to clamp, which in exact arithmetic never happens.
    static bool symmetrize(float* P);

    /// Transform from the current mean back to where the robot was `latencyMs`
    /// ago, per dead reckoning. Identity when there is no history to rewind
    /// through or the latency is zero.
    Transform2d rewind(float latencyMs, std::uint32_t nowMs) const;

    Config mConfig{};
    std::array<Pose, SENSOR_COUNT> mMounts{};
    std::array<RangeObstacle, MAX_OBSTACLES> mObstacles{};
    int mObstacleCount{0};

    float mState[3]{0.0f, 0.0f, 0.0f};
    float mP[9]{};

    /// Dead reckoning alone, never corrected: only differences of it are used,
    /// as the rigid transform between two instants.
    Pose mOdomTrail{};
    PoseBuffer<HISTORY> mTrail{};

    std::array<BeamState, SENSOR_COUNT> mBeamStates{};
    std::array<float, SENSOR_COUNT> mInnovations{};
    int mBeamsUsed{0};
    std::uint32_t mRangeRejects{0};
    std::uint32_t mRangeNoReturns{0};
    std::uint32_t mRangeDeadzoned{0};
    std::uint32_t mRangeGrazed{0};
    std::uint32_t mHeadingRejects{0};
    std::uint32_t mMclRejects{0};
    std::uint32_t mCovRepairs{0};
    int mRangeRejectStreak{0};
    int mRangeBlindStreak{0};
    int mMclRejectStreak{0};
    int mRelocateCount{0};
    bool mRelocated{false};
};

}  // namespace chad
