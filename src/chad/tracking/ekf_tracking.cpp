#include "chad/tracking/ekf_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

namespace chad {

EkfTracking::EkfTracking(OdomTracking& odom,
                         std::array<pros::Distance*, SENSOR_COUNT> distanceSensors,
                         const Pose& startPose,
                         const Config& config)
    : mCore(config),
      mOdom(odom),
      mDistanceSensors(distanceSensors) {
    setPose(startPose);
}

EkfTracking::~EkfTracking() {
    stopTracking();
}

std::uint32_t EkfTracking::nowMs() const {
    return mClock != nullptr ? mClock() : pros::millis();
}

// ============================================================================
// Lifecycle
// ============================================================================

void EkfTracking::startTracking() {
    if (mTask != nullptr) return;
    // Drop motion that accumulated while the filter was idle; it predates the
    // current state and would be integrated as though it had just happened.
    mOdom.consumeDelta(OdomTracking::CHANNEL_EKF);
    mRunning = true;
    mTask = new pros::Task([this]() { run(); });
}

void EkfTracking::stopTracking() {
    mRunning = false;
    if (mTask == nullptr) return;

    const uint32_t start = pros::millis();
    while (pros::millis() - start < 200) {
        const uint32_t state = mTask->get_state();
        if (state == pros::E_TASK_STATE_DELETED ||
            state == pros::E_TASK_STATE_INVALID) break;
        pros::delay(5);
    }
    const uint32_t state = mTask->get_state();
    if (state != pros::E_TASK_STATE_DELETED &&
        state != pros::E_TASK_STATE_INVALID) {
        mTask->remove();
    }
    delete mTask;
    mTask = nullptr;
}

bool EkfTracking::isTracking() const {
    return mRunning && mTask != nullptr;
}

void EkfTracking::run() {
    while (mRunning) {
        const uint32_t startTime = pros::millis();
        update();
        const int32_t elapsed = pros::millis() - startTime;
        const int32_t remaining =
            static_cast<int32_t>(config().updatePeriodMs) - elapsed;
        pros::delay(remaining > 2 ? remaining : 2);
    }
}

// ============================================================================
// Pose management
// ============================================================================

void EkfTracking::setPose(const Pose& pose) {
    setPose(pose, 1.0f, 0.02f);
}

void EkfTracking::setPose(const Pose& pose, float posSigma, float thetaSigma) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mOdom.consumeDelta(OdomTracking::CHANNEL_EKF);
    mCore.reset(pose, posSigma, thetaSigma, nowMs());
    mImuSkips = 0;
}

Pose EkfTracking::getEstimate() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.state();
}

EkfTracking::Uncertainty EkfTracking::uncertainty() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.uncertainty();
}

// ============================================================================
// Wiring
// ============================================================================

void EkfTracking::attachMcl(MclTracking* mcl) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mMcl = mcl;
}

void EkfTracking::setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mCore.setMounts(mounts);
}

void EkfTracking::setDistanceSensors(
        const std::array<pros::Distance*, SENSOR_COUNT>& sensors) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mDistanceSensors = sensors;
}

void EkfTracking::setObstacles(
        const std::vector<MclTracking::CircleObstacle>* circles) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mCircleObstacles = circles;
}

// ============================================================================
// Diagnostics
// ============================================================================

std::array<EkfTracking::BeamState, EkfTracking::SENSOR_COUNT>
EkfTracking::beamStates() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.beamStates();
}

std::array<float, EkfTracking::SENSOR_COUNT> EkfTracking::innovations() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.innovations();
}

int EkfTracking::beamsUsed() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.beamsUsed();
}

int EkfTracking::mclRejectStreak() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.mclRejectStreak();
}

int EkfTracking::rangeRejectStreak() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.rangeRejectStreak();
}

int EkfTracking::rangeBlindStreak() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.rangeBlindStreak();
}

std::uint32_t EkfTracking::rangeRejects() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.rangeRejects();
}

std::uint32_t EkfTracking::rangeNoReturns() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.rangeNoReturns();
}

std::uint32_t EkfTracking::rangeDeadzoned() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.rangeDeadzoned();
}

std::uint32_t EkfTracking::rangeGrazed() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.rangeGrazed();
}

std::uint32_t EkfTracking::headingRejects() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.headingRejects();
}

std::uint32_t EkfTracking::mclRejects() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.mclRejects();
}

std::uint32_t EkfTracking::covarianceRepairs() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.covarianceRepairs();
}

int EkfTracking::relocateCount() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.relocateCount();
}

EkfTracking::Config EkfTracking::config() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.config();
}

void EkfTracking::setConfig(const Config& config) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mCore.setConfig(config);
}

// ============================================================================
// update() — gather the hardware, hand it to the core, write the result back
// ============================================================================

Pose EkfTracking::update() {
    Config cfg;
    MclTracking* mcl = nullptr;
    const std::vector<MclTracking::CircleObstacle>* obstacles = nullptr;
    std::array<pros::Distance*, SENSOR_COUNT> sensors{};
    {
        std::lock_guard<pros::Mutex> lock(mMutex);
        cfg = mCore.config();
        mcl = mMcl;
        obstacles = mCircleObstacles;
        sensors = mDistanceSensors;
    }

    EkfCore::Inputs in;
    in.delta = mOdom.consumeDelta(OdomTracking::CHANNEL_EKF);
    in.hasHorizontalTracking = mOdom.hasHorizontalTracking();

    if (cfg.useImuHeading) {
        in.headingValid = mOdom.imuHeadingRad(in.headingRad);
        if (!in.headingValid) mImuSkips++;
    }

    if (cfg.useRangeUpdates) {
        for (int i = 0; i < SENSOR_COUNT; i++) {
            const std::size_t idx = static_cast<std::size_t>(i);
            if (sensors[idx] == nullptr) continue;
            in.beams[idx].present = true;
            in.beams[idx].mm = static_cast<float>(sensors[idx]->get());
            // V5 reports 0..63. A faulted port returns PROS_ERR, and the core
            // divides the range variance by this: unvalidated, one bad read gave
            // the beam a 2e-8 square inch variance and the filter snapped onto
            // it. get()'s own failure is caught by sentinelMm; this one was
            // caught by nothing. Out of range means the port faulted, so it is
            // reported as no confidence rather than as the top of the scale.
            const std::int32_t conf = sensors[idx]->get_confidence();
            in.beams[idx].confidence =
                (conf < 0 || conf > EkfCore::CONFIDENCE_MAX)
                    ? 0
                    : static_cast<int>(conf);
        }
    }

    if (cfg.useMclUpdates && mcl != nullptr && mcl->isTracking()) {
        const MclTracking::Estimate est = mcl->estimate();
        const int count = mcl->config().particleCount;
        if (est.nEff > 0.0f && count > 0) {
            in.mclValid = true;
            in.mclPose = est.pose;
            in.mclCovXX = est.covXX;
            in.mclCovXY = est.covXY;
            in.mclCovYY = est.covYY;
            in.mclNeffFrac = est.nEff / static_cast<float>(count);
        }
    }

    // The core keeps its own copy, so the caller's vector may change underneath
    // us between cycles.
    std::array<RangeObstacle, EkfCore::MAX_OBSTACLES> obsBuf{};
    int obsCount = 0;
    if (obstacles != nullptr) {
        for (const auto& c : *obstacles) {
            if (obsCount >= EkfCore::MAX_OBSTACLES) break;
            obsBuf[static_cast<std::size_t>(obsCount)] = {c.x, c.y, c.radius};
            obsCount++;
        }
    }

    const std::uint32_t now = nowMs();
    Pose fused;
    bool snap = false;
    {
        std::lock_guard<pros::Mutex> lock(mMutex);
        mCore.setObstacles(obsBuf.data(), obsCount);
        fused = mCore.step(now, in);
        snap = mCore.consumeRelocation();
    }

    // Written outside the lock: OdomTracking takes its own mutex, and holding
    // both in one order here while getPose() callers take them in the other is
    // how a deadlock gets built.
    if (cfg.autoSync) {
        const Pose odomPose = mOdom.getPose();
        if (snap) {
            mOdom.setPose(fused);
        } else {
            const float k = clamp(cfg.syncProp, 0.0f, 1.0f);
            const float kt = clamp(cfg.thetaSyncProp, 0.0f, 1.0f);
            const float dTheta = angleDiffRad(odomPose.theta, fused.theta);
            mOdom.setPose({odomPose.x + k * (fused.x - odomPose.x),
                           odomPose.y + k * (fused.y - odomPose.y),
                           wrapRad(odomPose.theta + kt * dTheta)});
        }
    }
    return fused;
}

}  // namespace chad
