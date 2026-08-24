// Ported from FBLib - https://github.com/jiazegao/FBLib
#include "chad/tracking/mcl_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

namespace chad {

MclTracking::MclTracking(OdomTracking& odom,
                         std::array<pros::Distance*, SENSOR_COUNT> distanceSensors,
                         const Pose& startPose,
                         const Config& config)
    : mCore(config), mOdom(odom), mDistanceSensors(distanceSensors) {
    mCore.setFieldMap(mclfield::map());
    for (int i = 0; i < SENSOR_COUNT; i++) {
        mGate.disabled[i] = (mDistanceSensors[i] == nullptr);
    }
    setPose(startPose);
}

MclTracking::~MclTracking() { stopTracking(); }

void MclTracking::startTracking() {
    if (mTask != nullptr) return;
    mRunning = true;
    mTask = new pros::Task([this]() { run(); });
}

void MclTracking::stopTracking() {
    mRunning = false;
    if (mTask == nullptr) return;

    const uint32_t start = pros::millis();
    while (pros::millis() - start < 200) {
        const uint32_t state = mTask->get_state();
        if (state == pros::E_TASK_STATE_DELETED || state == pros::E_TASK_STATE_INVALID) break;
        pros::delay(5);
    }
    const uint32_t state = mTask->get_state();
    if (state != pros::E_TASK_STATE_DELETED && state != pros::E_TASK_STATE_INVALID) {
        // Killing the worker mid-tick is why the published estimate is lock-free
        // rather than mutex-guarded: a task removed while holding a mutex never
        // gives it back, and there is no watchdog to notice.
        mTask->remove();
    }
    delete mTask;
    mTask = nullptr;
}

bool MclTracking::isTracking() const { return mRunning && mTask != nullptr; }

void MclTracking::run() {
    while (mRunning) {
        const uint32_t startTime = pros::millis();
        // Odometry freshness comes from the background odometry task, the sole
        // caller of mOdom.update(). Calling it here too would race on odometry
        // state and consume delta baselines.
        update();

        const int32_t elapsed = static_cast<int32_t>(pros::millis() - startTime);
        const int32_t remaining = static_cast<int32_t>(config().updatePeriodMs) - elapsed;
        pros::delay(remaining > 2 ? static_cast<uint32_t>(remaining) : 2u);
    }
}

void MclTracking::setPose(const Pose& pose) {
    std::lock_guard<pros::Mutex> lock(mLock);
    // Motion accumulated before this reposition predates the new distribution
    // and would otherwise be applied to freshly initialised particles.
    mOdom.consumeDelta();
    mCore.setPose(pose);
    mPublished.store(mCore.estimate());
}

void MclTracking::uniformReset() {
    std::lock_guard<pros::Mutex> lock(mLock);
    mOdom.consumeDelta();
    mCore.uniformReset();
    mPublished.store(mCore.estimate());
}

void MclTracking::refreshObstaclesLocked() {
    int nLines = 0;
    for (const auto& line : mclfield::kBlockers) {
        if (nLines >= BLOCKER_MAX) break;
        mBlockers[nLines++] = line;
    }
    if (mLineObstacles != nullptr) {
        for (const auto& line : *mLineObstacles) {
            if (nLines >= BLOCKER_MAX) break;
            mBlockers[nLines++] = line;
        }
    }
    mCore.setBlockers(nLines > 0 ? mBlockers : nullptr, nLines);

    int nCircles = 0;
    if (mCircleObstacles != nullptr) {
        for (const auto& circle : *mCircleObstacles) {
            if (nCircles >= CIRCLE_MAX) break;
            mCircles[nCircles++] = circle;
        }
    }
    mCore.setDeadzoneCircles(nCircles > 0 ? mCircles : nullptr, nCircles);
}

int MclTracking::readSensorsLocked(MclCore::RawSensor* out) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        const bool fitted = mDistanceSensors[i] != nullptr;
        const int mm = fitted ? mDistanceSensors[i]->get() : 0;
        const int conf = fitted ? mDistanceSensors[i]->get_confidence() : 0;
        out[i] = mcladapter::classifyRaw(fitted, mm, conf, mGate.disabled[i]);
        mReadingsInch[i] =
            out[i].present ? static_cast<float>(out[i].mm) * MM_TO_INCH : 0.0f;
    }
    return SENSOR_COUNT;
}

Pose MclTracking::referencePoseLocked() const {
    // Two-argument overload: the one-argument one returns a HELD value when the
    // IMU has faulted, and a hold is not a measurement.
    float imu = 0.0f;
    const bool ok = mOdom.imuHeadingRad(imu);
    return mcladapter::referencePose(mCore.rawEstimate(), imu, ok);
}

void MclTracking::predict() {
    std::lock_guard<pros::Mutex> lock(mLock);
    MclCore::MotionInput in;
    // consumeDelta() returns motion accumulated since our previous call and
    // resets the accumulator. getLastDelta() holds only the most recent 10 ms
    // odometry tick, so any tick that landed between MCL iterations would be
    // silently dropped and the cloud would fall behind the robot.
    in.delta = mOdom.consumeDelta();
    in.hasImu = mOdom.imuHeadingRad(in.imuHeadingRad);
    in.hasVertWheel = mOdom.hasVerticalTrackingWheel();
    in.hasHorizWheel = mOdom.hasHorizontalTracking();
    in.dtSeconds = std::max(mCore.config().updatePeriodMs, 1.0f) * 0.001f;
    mCore.predict(in);
}

void MclTracking::updateWeights() {
    std::lock_guard<pros::Mutex> lock(mLock);
    MclCore::RawSensor raw[SENSOR_COUNT];
    const int count = readSensorsLocked(raw);
    refreshObstaclesLocked();
    mBeamCount = mCore.buildBeams(raw, count, referencePoseLocked(), mBeams,
                                  mValidSensors.data());
    mCore.updateWeights(mBeams, mBeamCount);
}

void MclTracking::resample() {
    std::lock_guard<pros::Mutex> lock(mLock);
    float imu = 0.0f;
    const bool ok = mOdom.imuHeadingRad(imu);
    mCore.resample(imu, ok);
}

Pose MclTracking::update() {
    std::lock_guard<pros::Mutex> lock(mLock);
    return updateLocked();
}

Pose MclTracking::updateLocked() {
    const float periodMs = std::max(mCore.config().updatePeriodMs, 1.0f);
    mGate.tick(periodMs);

    MclCore::RawSensor raw[SENSOR_COUNT];
    const int rawCount = readSensorsLocked(raw);
    refreshObstaclesLocked();
    mBeamCount = mCore.buildBeams(raw, rawCount, referencePoseLocked(), mBeams,
                                  mValidSensors.data());

    MclCore::MotionInput in;
    in.delta = mOdom.consumeDelta();
    in.hasImu = mOdom.imuHeadingRad(in.imuHeadingRad);
    in.hasVertWheel = mOdom.hasVerticalTrackingWheel();
    in.hasHorizWheel = mOdom.hasHorizontalTracking();
    in.dtSeconds = periodMs * 0.001f;

    const Pose pose = mCore.step(in, mBeams, mBeamCount);
    mPublished.store(mCore.stepEstimate());

    if (mCore.config().autoSync) syncToOdometryLocked();
    return pose;
}

std::pair<Pose, float> MclTracking::getEstimate() const {
    const Estimate est = estimate();
    return {est.pose, est.nEff};
}

MclTracking::Estimate MclTracking::estimate() const {
    // Whatever the worker last published, copied coherently and without waiting.
    // Reading mCore here instead would walk a cloud that is halfway through a
    // propagation, and reading the published struct unlocked -- which is what
    // this used to do on a lock timeout -- hands back x from one tick with y
    // from the next.
    if (mRunning) return mPublished.load();

    // Stopped, so the cloud is the fresher answer -- but the wait is bounded.
    // stopTracking() removes a worker that would not exit, and a task removed
    // inside updateLocked() never gives the mutex back; a control loop asking
    // for the pose must degrade to the last published tick rather than block on
    // a lock that is never coming.
    if (mLock.take(5)) {
        const Estimate out = mCore.estimate();
        mLock.give();
        return out;
    }
    return mPublished.load();
}

Pose MclTracking::getRawEstimate() const {
    std::lock_guard<pros::Mutex> lock(mLock);
    return mCore.rawEstimate();
}

void MclTracking::syncToOdometry() {
    std::lock_guard<pros::Mutex> lock(mLock);
    syncToOdometryLocked();
}

void MclTracking::syncToOdometryLocked() {
    const Config& c = mCore.config();
    mOdom.setPose(mcladapter::syncStep(mOdom.getPose(), mCore.rawEstimate(),
                                       c.distSyncProp, c.thetaSyncProp));
}

void MclTracking::setDistanceSensors(
    const std::array<pros::Distance*, SENSOR_COUNT>& sensors) {
    std::lock_guard<pros::Mutex> lock(mLock);
    mDistanceSensors = sensors;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        mGate.disabled[i] = (sensors[i] == nullptr);
        mGate.timers[i] = 0.0f;
    }
}

void MclTracking::setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts) {
    std::lock_guard<pros::Mutex> lock(mLock);
    mCore.setSensorMounts(mounts);
}

void MclTracking::enableSensor(int index) {
    std::lock_guard<pros::Mutex> lock(mLock);
    mGate.enable(index);
}

void MclTracking::disableSensor(int index) {
    std::lock_guard<pros::Mutex> lock(mLock);
    mGate.disable(index);   // no timer: stays off until enableSensor()
}

void MclTracking::disableSensorFor(int index, float durationMs) {
    std::lock_guard<pros::Mutex> lock(mLock);
    mGate.disableFor(index, durationMs);
}

void MclTracking::setObstacles(const std::vector<LineObstacle>* lineObstacles,
                               const std::vector<CircleObstacle>* circleObstacles) {
    std::lock_guard<pros::Mutex> lock(mLock);
    mLineObstacles = lineObstacles;
    mCircleObstacles = circleObstacles;
    refreshObstaclesLocked();
}

void MclTracking::setDrift(float verticalDrift, float horizontalDrift) {
    std::lock_guard<pros::Mutex> lock(mLock);
    const float updatesPerSec = 1000.0f / std::max(mCore.config().updatePeriodMs, 1.0f);
    mCore.setDrift(verticalDrift / updatesPerSec, horizontalDrift / updatesPerSec);
}

MclTracking::Config MclTracking::config() const {
    std::lock_guard<pros::Mutex> lock(mLock);
    return mCore.config();
}

void MclTracking::setConfig(const Config& config) {
    // buildLut() rewrites 1024 floats the worker reads inside logBeamLikelihood.
    std::lock_guard<pros::Mutex> lock(mLock);
    mCore.setConfig(config);
}

void MclTracking::setSeed(std::uint32_t seed) {
    std::lock_guard<pros::Mutex> lock(mLock);
    mCore.setSeed(seed);
}

void MclTracking::setDistSyncProp(float prop) {
    std::lock_guard<pros::Mutex> lock(mLock);
    Config c = mCore.config();
    c.distSyncProp = prop;
    mCore.setConfig(c);
}

void MclTracking::setAutoSync(bool enabled) {
    std::lock_guard<pros::Mutex> lock(mLock);
    Config c = mCore.config();
    c.autoSync = enabled;
    mCore.setConfig(c);
}

float MclTracking::randomInjectFraction() const {
    std::lock_guard<pros::Mutex> lock(mLock);
    return mCore.injectFraction();
}

float MclTracking::likelihoodMismatch() const {
    std::lock_guard<pros::Mutex> lock(mLock);
    return mCore.likelihoodMismatch();
}

float MclTracking::fitSigmas() const {
    std::lock_guard<pros::Mutex> lock(mLock);
    return mCore.fitSigmas();
}

float MclTracking::searchMismatch() const {
    std::lock_guard<pros::Mutex> lock(mLock);
    return mCore.searchMismatch();
}

std::uint32_t MclTracking::resampleCount() const {
    std::lock_guard<pros::Mutex> lock(mLock);
    return mCore.resampleCount();
}

}  // namespace chad
