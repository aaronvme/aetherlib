#pragma once

#include <array>
#include <atomic>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"

#include "chad/tracking/mcl_tracking.hpp"
#include "chad/tracking/odom_tracking.hpp"
#include "chad/util/util.hpp"

namespace chad {

// ============================================================================
// EkfTracking — extended Kalman filter over SE(2)
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
// wired in as a measurement precisely to supply that. Run them together --
// MCL finds the robot, the EKF decides how much to believe it and delivers a
// smooth pose at odometry rate.
//
// usage:
//   EkfTracking ekf(odom, distanceSensors, startPose, config);
//   ekf.attachMcl(&mcl);
//   ekf.startTracking();
// ============================================================================

class EkfTracking {
public:
    static constexpr int SENSOR_COUNT = MAX_DISTANCE_SENSORS;

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
        float vertBiasProp{0.06f};        // systematic along-track scale error [gap]
        float slipBiasProp{0.05f};        // systematic cross-track error [gap]
        float biasDecorrIn{40.0f};        // inches over which that bias renews [gap]
        float thetaNoiseProp{0.020f};     // sigma per radian of |dTheta| [model]
        float thetaNoisePerInch{0.0015f}; // heading sigma per inch driven [model]
        float posNoiseFloor{0.015f};      // inches per update [model]
        float thetaNoiseFloor{0.0004f};   // radians per update [model]

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

        // --- obstacle deadzones --------------------------------------------
        // Same cone-plus-range test MCL uses, but widened by the filter's own
        // position sigma: when the EKF is unsure where it is, it is also unsure
        // which beams a post could be spoiling, and should drop more of them.
        float obstacleConeFrac{0.22f};    // tan(12.5 deg), the V5 half-cone [model]
        float obstacleRangeTol{6.0f};     // inches [model]
        float obstacleSigmaPad{2.0f};     // extra tolerance, in position sigmas [model]

        // --- IMU heading ---------------------------------------------------
        bool  useImuHeading{true};   // [yours]
        float imuSigmaRad{0.006f};   // [model]

        // --- MCL position --------------------------------------------------
        bool  useMclUpdates{true};   // [yours]
        // The particle cloud's own spread understates its error once it has
        // converged, so scale it up and never let it claim better than the
        // floor. Trusting the raw spread makes the EKF chase MCL's jitter.
        float mclSigmaScale{1.6f};   // [sim]
        float mclSigmaFloor{1.2f};        // inches [sim]
        float mclGateChi2{11.8f};         // 2 dof; 11.8 == 3 sigma [model]
        // A kidnapped robot fails the gate forever, which is the one case where
        // gating is wrong. Count consecutive failures and hand over to MCL once
        // it is both persistent and coming from a healthy cloud.
        int   mclResetCycles{10};   // [model]
        float mclMinNeffFrac{0.20f};   // [model]

        // --- output ---------------------------------------------------------
        bool  autoSync{true};             // write the fused pose into odometry [model]
        // Fraction of the correction applied per update. A hard write steps the
        // pose every cycle and every controller downstream differentiates it;
        // the filter's own state stays exact either way. Relocation bypasses it.
        float syncProp{0.12f};   // [model]
        float thetaSyncProp{0.20f};   // [model]
        float updatePeriodMs{10.0f};   // [model]
    };

    EkfTracking(OdomTracking& odom,
                std::array<pros::Distance*, SENSOR_COUNT> distanceSensors,
                const Pose& startPose,
                const Config& config);
    ~EkfTracking();

    // ========================================================================
    // Lifecycle
    // ========================================================================

    void startTracking();
    void stopTracking();
    bool isTracking() const;

    /// One full cycle: predict, then every enabled measurement family.
    Pose update();

    // ========================================================================
    // State
    // ========================================================================

    Pose getEstimate() const;

    /// Position standard deviations (inches) and heading sigma (radians),
    /// straight off the diagonal of P.
    struct Uncertainty {
        float sigmaX{0.0f}, sigmaY{0.0f}, sigmaTheta{0.0f};
    };
    Uncertainty uncertainty() const;

    /// Reset the state to `pose` and the covariance to the initial-belief
    /// values. Use after a deliberate reposition, not to nudge the filter.
    void setPose(const Pose& pose);

    /// Reset the state to `pose` with an explicit position sigma, for handing
    /// over from a filter that knows how wrong it might be.
    void setPose(const Pose& pose, float posSigma, float thetaSigma);

    // ========================================================================
    // Wiring
    // ========================================================================

    /// Supply the particle filter used as the global-position measurement.
    /// Pass nullptr to run the EKF on ranges and the IMU alone.
    void attachMcl(MclTracking* mcl) { mMcl = mcl; }

    void setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts);
    void setDistanceSensors(const std::array<pros::Distance*, SENSOR_COUNT>& sensors);

    /// Obstacles whose returns must not be read as wall ranges.
    void setObstacles(const std::vector<MclTracking::CircleObstacle>* circles) {
        mCircleObstacles = circles;
    }

    // ========================================================================
    // Diagnostics
    // ========================================================================

    /// Per-beam outcome from the most recent cycle.
    enum class BeamState : std::uint8_t {
        Absent,     // no sensor in this slot
        NoReturn,   // out of range or below confidence
        Deadzone,   // an obstacle owns this return
        Grazing,    // incidence too high to invert
        Rejected,   // failed the Mahalanobis gate
        Used,
    };
    const std::array<BeamState, SENSOR_COUNT>& beamStates() const { return mBeamStates; }
    const std::array<float, SENSOR_COUNT>& innovations() const { return mInnovations; }

    /// Beams folded into the state on the last cycle.
    int beamsUsed() const { return mBeamsUsed; }

    /// Consecutive cycles MCL has disagreed with the filter beyond its gate.
    int mclRejectStreak() const { return mMclRejectStreak; }

    /// Times the filter has been forced onto MCL's estimate since construction.
    int relocateCount() const { return mRelocateCount; }

    const Config& config() const { return mConfig; }
    void setConfig(const Config& config) { mConfig = config; }

private:
    // ========================================================================
    // Core steps
    // ========================================================================

    void predict(const OdomDelta& delta);
    void updateRanges();
    void updateImuHeading();
    void updateMcl();

    /// Scalar (1-dof) correction. H is the 1x3 row, innovation the residual,
    /// R the measurement variance. Returns false when the gate rejects it.
    bool applyScalarUpdate(const float H[3], float innovation, float R,
                           float gateChi2);

    /// Ray/wall intersection with the surface normal, needed for the Jacobian.
    struct WallHit {
        bool  valid{false};
        float range{0.0f};
        float nx{0.0f}, ny{0.0f};   // unit normal of the surface hit
        float cosIncidence{0.0f};   // |n . rayDir|
    };
    WallHit castWalls(float sx, float sy, float rayCos, float raySin) const;

    /// True when an obstacle, not a wall, is what this beam ranged — either the
    /// return matches the obstacle surface, or the obstacle occludes the wall at
    /// `wallRange`.
    bool obstacleOwnsReading(float sx, float sy, float rayCos, float raySin,
                             float readingIn, float wallRange) const;

    static void taskLoop(void* param);
    void run();

    // ========================================================================
    // Members
    // ========================================================================

    Config mConfig;
    OdomTracking& mOdom;
    MclTracking* mMcl{nullptr};
    std::array<pros::Distance*, SENSOR_COUNT> mDistanceSensors;
    const std::vector<MclTracking::CircleObstacle>* mCircleObstacles{nullptr};

    std::array<Pose, SENSOR_COUNT> mSensorMounts{};

    // State and covariance. Row-major 3x3, ordered [x, y, theta].
    float mState[3]{0.0f, 0.0f, 0.0f};
    float mP[9]{};

    std::array<BeamState, SENSOR_COUNT> mBeamStates{};
    std::array<float, SENSOR_COUNT> mInnovations{};
    int mBeamsUsed{0};
    int mMclRejectStreak{0};
    int mRelocateCount{0};
    bool mSnapNextSync{false};   // relocation: land the whole correction at once

    // Guards mState/mP: the background task writes them while getEstimate()
    // and uncertainty() are called from motion code on another task.
    mutable pros::Mutex mMutex;

    pros::Task* mTask{nullptr};
    std::atomic<bool> mRunning{false};
};

}  // namespace chad
