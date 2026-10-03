#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <utility>
#include <vector>

#include "pros/distance.hpp"
#include "pros/rtos.hpp"

#include "chad/tracking/odom_tracking.hpp"
#include "chad/util/util.hpp"

namespace chad {
enum class CoordType { X, Y, INVALID };

class RclTracking {
public:

    /// Why a reading did or did not reach the axis estimate. Ordered so the
    /// diagnostics can group them: everything past WallHit is a rejection, and
    /// the reason is the only way to tell a sensor that is broken from one that
    /// is being asked to look at a robot.
    enum class ReadingClass {
        Invalid,        // no device, a faulted one, or outside what it can report
        WallHit,        // used

        Stale,          // the device has not produced a new sample yet
        LowConfidence,
        OffAxis,        // beam is not within angleTolerance of a field axis
        Turning,        // the whole cycle is suppressed by the turn-rate gate
        PostOwned,      // a mapped obstacle explains the return
        FarSide,        // the return comes from across the auton line
        Blocked,        // reads short of the wall by more than maxWallInnovation
        TooFar,         // reads long of it by the same
        Impossible,     // no return at all, with a wall well inside reach
    };

    struct RclSensor {
        pros::Distance* sensor{nullptr};
        Pose mountOffset;
        bool enabled{true};

        float worldX{0.0f}, worldY{0.0f};
        float rayCos{0.0f}, raySin{0.0f};
        float worldHeading{0.0f};
        float readingInch{0.0f};
        int confidence{0};
        bool valid{false};

        // --- sampling bookkeeping ---
        int   rawMm{-1};            // last raw value seen from the device
        float heldMs{0.0f};         // how long it has read the same thing
        bool  fresh{false};         // this cycle saw a new sample
        ReadingClass cls{ReadingClass::Invalid};
        float innovationIn{0.0f};   // reading minus predicted wall range
        /// Confirmed by its opposite number spanning the field, which needs no
        /// reference to the current estimate.
        bool  pairVerified{false};
        /// Dropped by the axis median, having passed everything else.
        bool  outlier{false};
        /// The device answered PROS_ERR: unplugged, or the port holds something
        /// else. Never fresh, never evidence. Distinct from a working sensor's
        /// 9999, which is a measurement.
        bool  faulted{false};
        /// Mount offset rotated into the field at sample time: world - robot.
        float mountWorldX{0.0f}, mountWorldY{0.0f};
    };

    /// lifetimeMs 0 = permanent. Expiry is a pros::millis() deadline set
    /// when the obstacle is added.
    struct LineObstacle {
        float x1, y1, x2, y2;
        float lifetimeMs = 0.0f;
        std::uint32_t expiresAtMs = 0;
    };

    struct CircleObstacle {
        float x, y, radius;
        float lifetimeMs = 0.0f;
        std::uint32_t expiresAtMs = 0;
    };

    struct Config {
        /// How many slots hold a device. Reported for diagnostics; every loop
        /// walks all MAX_DISTANCE_SENSORS slots and skips the empty ones, so a
        /// config with a gap in it is not silently half-read.
        int sensorCount = MAX_DISTANCE_SENSORS;
        float maxSyncDist = 5.0f;

        bool hardSync = false;
        float syncProp = 0.15f;

        float maxSyncRate = 8.0f;

        float accumulationAlpha = 0.3f;
        float updatePeriodMs = 10.0f;
        float angleTolerance = 10.0f;

        float maxWallInnovation = 8.0f;

        float recoverySyncProp = 0.9f;

        int displaceCycles = 3;

        float noReturnWallLimit = 35.0f;

        int confidenceThreshold = 30;

        float obstacleConeFrac = 0.22f;
        float obstacleRangeTol = 6.0f;
        bool autoSync = true;
        bool useAccumulation = false;

        // --- sampling validity -------------------------------------------
        /// A V5 distance sensor refreshes about every 30 ms and holds the
        /// previous value in between, while this loop runs at 10. Folding the
        /// held value in again lets one physical sample vote three times, which
        /// triples the effective sync rate and makes the noise look three times
        /// more consistent than it is. A sample counts as new when the raw
        /// millimetre value changes; one that has genuinely not moved for this
        /// long is allowed through again so a stationary robot still corrects.
        float staleHoldMs = 45.0f;

        /// Past this the cone smears across its own sweep and the sample
        /// describes a heading the robot has already left. Suppresses the whole
        /// cycle rather than individual beams, because the smear is common mode.
        float maxTurnRateRadS = 1.5f;

        /// The reading describes where the robot was one device period ago. At
        /// 60 in/s that is 1.8 in, which is larger than the fix is worth. The
        /// solve runs against the pose at sample time and the result is carried
        /// forward. Zero disables it.
        float sensorLatencyMs = 30.0f;

        /// Votes on one axis further than this from their median are dropped.
        float axisAgreeIn = 2.0f;

        /// Two anti-parallel beams on the same axis must span the field between
        /// them. Inside this tolerance both are confirmed outright, with no
        /// reference to the current estimate -- the only check here that a
        /// wrong pose cannot talk its way through.
        float pairSpanTolIn = 1.5f;

        /// Clean cycles needed to clear a displacement, against displaceCycles
        /// to declare one. Asymmetric on purpose: entering recovery on noise
        /// costs a few soft cycles, leaving it early costs the pose.
        int recoverCycles = 8;

        /// While displaced, an axis needs at least this many agreeing votes
        /// before it may move the pose. One is usually the honest setting: the
        /// robot carries two sensors per axis and the far one is normally out
        /// of range, so demanding two means an axis that can be recovered often
        /// is not. The step is bounded to maxSyncDist per reading instead,
        /// which is what keeps a single wrong one survivable.
        int recoveryMinVotes = 1;

        // --- far-side gate ------------------------------------------------
        /// Ignore returns from beyond a half-plane -- the auton line. During
        /// the 15 s period the far half of the field holds two robots that are
        /// moving, so anything ranged across it is measuring them rather than a
        /// wall, and it is far enough away to be the weakest reading anyway.
        bool farSideGate = false;
        /// Unit normal pointing AWAY from our side, and its offset: a point p
        /// is on the far side when nx*px + ny*py > d.
        float farSideNx = 0.0f;
        float farSideNy = 0.0f;
        float farSideD = 0.0f;
    };

    RclTracking(OdomTracking& odom, const Config& config);
    ~RclTracking();

    void setSensor(int index, pros::Distance* sensor, const Pose& mountOffset);

    void configureSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors,
                          const std::array<Pose, MAX_DISTANCE_SENSORS>& mounts);

    void enableSensor(int index);
    void disableSensor(int index);
    /// durationMs <= 0 disables until enableSensor(). A deadline, so it runs
    /// out whether or not the task is running.
    void disableSensorFor(int index, float durationMs);

    void addLineObstacle(float x1, float y1, float x2, float y2, float lifetimeMs = 0.0f);
    void addCircleObstacle(float x, float y, float radius, float lifetimeMs = 0.0f);
    void clearObstacles();

    /// Arm the far-side gate from a half-plane. `nx, ny` need not be
    /// normalised. Points satisfying n.p > d are ignored.
    void setFarSideGate(float nx, float ny, float d, float lifetimeMs = 0.0f);

    /// Arm it from a line through two points, keeping the half the robot is
    /// standing in. Alliance-agnostic. The side is taken from the pose now.
    /// If no pose was declared in the kFarSideOrientWindowMs before arming,
    /// the first hardReset() -- i.e. setPose -- inside that window after
    /// arming picks the side again, so a routine that arms before declaring
    /// its start pose still keeps its own half. Later resets never flip it.
    void setFarSideGateFromLine(float x1, float y1, float x2, float y2,
                                float lifetimeMs = 0.0f);
    static constexpr std::uint32_t kFarSideOrientWindowMs = 500;

    /// Disarm, including any pending side choice and deadline. Takes the lock
    /// with a timeout; if it is held (a task deleted inside it), the clear is
    /// flagged and the tracking task applies it on its next cycle.
    void clearFarSideGate();
    /// A lifetime is a wall-clock deadline, so it runs out whether or not the
    /// tracking task is running.
    bool farSideGateActive() const;
    float farSideGateRemainingMs() const;

    void update();

    void updateSensorPoses(const Pose& robotPose);

    ReadingClass classifyReading(int index, float& wallDist) const;

    bool isValidReading(int index) const;

    bool isDisplaced() const { return mDisplaced; }

    std::pair<CoordType, float> getBotCoordFromSensor(int index) const;

    struct AxisEstimate {
        float x{0.0f}, y{0.0f};
        int xCount{0}, yCount{0};
        /// Spread of the surviving votes on each axis. Zero with one vote.
        float xSpread{0.0f}, ySpread{0.0f};
        /// At least one opposing pair spanned the field on this axis.
        bool xPaired{false}, yPaired{false};
    };
    AxisEstimate estimateAxes() const;

    std::pair<Pose, int> computePose() const;

    void startTracking();

    void stopTracking();

    bool isTracking() const;

    void syncUpdate();

    bool updateBotPoseFromBestSensor();

    /// Adopt `pose` as the whole of what this filter believes, discarding the
    /// accumulated residual, the displacement state and the per-sensor sample
    /// history. A beam reset that left these standing would be walked back by
    /// the next sync, or -- worse -- land while mDisplaced is still set and be
    /// followed by an unbounded recovery snap from readings taken against the
    /// pose it just replaced.
    void hardReset(const Pose& pose);

    const Config& config() const { return mConfig; }

    /// Replaces the tuning wholesale, keeping the sensor count and whatever
    /// far-side gate is currently armed -- neither is tuning, and a live gate
    /// silently disarming itself because a slider moved is the kind of thing
    /// that is only ever noticed during a match.
    void setConfig(const Config& config);

    void setAutoSync(bool enabled);

    const std::array<RclSensor, MAX_DISTANCE_SENSORS>& sensors() const { return mSensors; }
    Pose latestEstimate() const;
    int activeSensorCount() const { return mActiveSensorCount; }

    /// Worst axis spread from the last cycle, as a standing-in-for-sigma the
    /// auton-side beam reset can widen its jump gate with. RCL carries no
    /// covariance, so disagreement between sensors is the only self-report it
    /// has about how well it currently knows where it is.
    float spreadIn() const { return mSpreadIn; }

    /// Distance the last syncUpdate actually moved the pose, and the raw
    /// per-axis residuals behind it. Diagnostics for the sim plots.
    float lastSyncStepIn() const { return mLastSyncStep; }
    float residualX() const { return mLastResidualX; }
    float residualY() const { return mLastResidualY; }
    bool  turnGateHeld() const { return mTurnHeld; }
    float turnRateRadS() const { return mTurnRate; }
    /// Bit i set: slot i last answered PROS_ERR.
    std::uint32_t faultedMask() const { return mFaultedMask; }
    /// Measured period of the last update(), ms.
    float lastDtMs() const { return mDtMs; }

private:

    float intersectLineSegment(float rx, float ry, float rCos, float rSin,
                                float x1, float y1, float x2, float y2,
                                float maxRange) const;

    std::pair<int, float> nearestWall(const RclSensor& s) const;

    /// True when p is on the far side of an armed auton line.
    bool farSide(float px, float py) const;

    /// Gate arming with the lock already held. update() expires the gate from
    /// inside its own critical section, and pros::Mutex is not recursive.
    void armFarSideLocked(float nx, float ny, float d, float lifetimeMs);
    void clearFarSideLocked();
    void expireFarSideLocked(std::uint32_t nowMs);
    /// Flip the armed gate, if needed, so `p` is on the kept side.
    void orientFarSideLocked(const Pose& p);
    void expireDisableTimersLocked(std::uint32_t nowMs);
    void expireObstaclesLocked(std::uint32_t nowMs);

    struct RawSample {
        pros::Distance* dev{nullptr};   // null = not polled
        std::int32_t mm{0};
        std::int32_t conf{0};
    };
    /// One poll. `gen` is mSampleGen at poll time: a reset or reconfigure in
    /// between invalidates the whole batch.
    struct RawSamples {
        std::array<RawSample, MAX_DISTANCE_SENSORS> s{};
        std::uint32_t gen{0};
    };

    /// Decide which polled readings are new. Returns true when at least one
    /// sensor produced a sample this loop has not already seen; false means
    /// there is nothing to recompute and the previous cycle's verdicts still
    /// stand.
    bool ingestSamples(const RawSamples& raw);

    /// Read every enabled device, holding mMutex only to expire disable
    /// timers and copy the pointers. A device read under it would leave the
    /// mutex taken for good if the calling task were deleted mid-read, which
    /// PROS does on a mode change.
    void pollDevices(RawSamples& out);

    /// Classify every populated slot and record the result on the sensor.
    void classifyAll();

    /// How much a reading is worth this cycle: 2 = confirmed by its opposite
    /// number, 1 = a clean wall hit, 0 = rejected by the innovation gate but
    /// usable because the pose is known to be wrong, -1 = not usable. An axis
    /// only ever votes with its best available tier.
    int voteTier(int index) const;

    /// Cross-check anti-parallel beams against the field span.
    void checkPairs();

    /// The axis solve. `keptOut`, when given, receives MAX_DISTANCE_SENSORS
    /// flags marking which readings survived the median -- the const public
    /// query passes null, update() passes a buffer so it can record why a
    /// sensor that looked healthy did not end up voting.
    AxisEstimate solveAxes(bool* keptOut) const;

    static void taskLoop(void* param);
    void run();

    /// syncUpdate's body, with the lock already held. pros::Mutex is not
    /// recursive, so update() cannot simply call the public entry point.
    void syncUpdateLocked();

    /// Guards everything the background task writes against the pose setters,
    /// which run on whichever thread called chassis.setPose(). MCL and the EKF
    /// both carry one; RCL did not, which only became a real exposure when it
    /// was made the filter that owns the pose -- a hardReset landing halfway
    /// through a cycle could leave the recovery flag set against a pose that
    /// had just been declared correct.
    mutable pros::Mutex mMutex;

    Config mConfig;
    OdomTracking& mOdom;
    std::array<RclSensor, MAX_DISTANCE_SENSORS> mSensors;
    std::vector<LineObstacle> mLineObstacles;
    std::vector<CircleObstacle> mCircleObstacles;

    Pose mLatestEstimate;
    int mActiveSensorCount{0};

    bool mXMeasured{false};
    bool mYMeasured{false};

    float mResidualX{0.0f};
    float mResidualY{0.0f};

    bool mDisplaced{false};
    int mDisplaceCycles{0};
    int mCleanCycles{0};

    /// Absolute pros::millis() re-enable times; 0 = none.
    std::array<std::uint32_t, MAX_DISTANCE_SENSORS> mDisableUntilMs{};

    /// Absolute expiry; meaningful only while mFarSideTimed.
    std::uint32_t mFarSideDeadlineMs{0};
    bool mFarSideTimed{false};
    /// Armed from a line with no recent pose: the first hardReset() within
    /// kFarSideOrientWindowMs of mFarSideArmedMs re-picks the kept side.
    bool mFarSideOrientPending{false};
    std::uint32_t mFarSideArmedMs{0};
    std::uint32_t mLastHardResetMs{0};
    bool mHaveHardReset{false};
    std::atomic<bool> mFarSideClearRequested{false};

    /// Bumped by anything that invalidates an in-flight poll.
    std::uint32_t mSampleGen{0};

    std::uint32_t mFaultedMask{0};

    std::uint32_t mLastUpdateMs{0};
    bool  mHaveLastUpdate{false};
    float mDtMs{10.0f};

    // Motion state, tracked from successive odom poses rather than a delta
    // channel -- there are only two of those and they belong to MCL and the
    // EKF. Low-passed, because a 10 ms difference of a quantised pose is mostly
    // quantisation.
    Pose  mPrevPose;
    bool  mHavePrevPose{false};
    float mVelX{0.0f}, mVelY{0.0f};
    /// What the last syncUpdate added to the odometry pose, so the next cycle
    /// can subtract it back out before calling the remainder velocity.
    float mAppliedDx{0.0f}, mAppliedDy{0.0f};
    float mTurnRate{0.0f};
    bool  mTurnHeld{false};

    // What the solve was carried forward by, so syncUpdate compares like with
    // like: the axes are solved at sample time and the answer is advanced to
    // now before it is differenced against the current pose.
    float mLagShiftX{0.0f}, mLagShiftY{0.0f};

    float mSpreadIn{0.0f};
    float mLastSyncStep{0.0f};
    float mLastResidualX{0.0f};
    float mLastResidualY{0.0f};

    pros::Task* mTask{nullptr};
    std::atomic<bool> mRunning{false};
};
}
