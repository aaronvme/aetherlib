// Ported from FBLib - https://github.com/jiazegao/FBLib
#include "chad/tracking/odom_tracking.hpp"

#include <cmath>
#include <mutex>

#include "chad/drivetrain.hpp"
#include "pros/rtos.hpp"

namespace chad {

// ============================================================================
// TrackingWheel
// ============================================================================

// ADI encoder: 360 ticks per revolution
static constexpr float ADI_TICKS_PER_REV = 360.0f;

// Rotation sensor: reports centidegrees (100 per degree, 36000 per revolution)
static constexpr float ROTATION_UNITS_PER_REV = 36000.0f;

TrackingWheel::TrackingWheel(pros::adi::Encoder& enc,
                             float wheelDiamIn,
                             float offsetIn,
                             float gearRatio)
    : mType(SensorType::ADI),
      mAdi(&enc),
      mWheelDiamIn(wheelDiamIn),
      mOffsetIn(offsetIn),
      mGearRatio(gearRatio) {}

TrackingWheel::TrackingWheel(pros::Rotation& rot,
                             float wheelDiamIn,
                             float offsetIn,
                             float gearRatio)
    : mType(SensorType::Rotation),
      mRot(&rot),
      mWheelDiamIn(wheelDiamIn),
      mOffsetIn(offsetIn),
      mGearRatio(gearRatio) {}

float TrackingWheel::distanceIn() const {
    // Circumference of the tracking wheel
    float circumference = mWheelDiamIn * PI;

    // PROS reports INT32_MAX (PROS_ERR) when the sensor errors or is unplugged.
    // Integrating that would teleport odometry ~150,000 miles; hold the last
    // good distance instead so a transient disconnect just freezes position.
    constexpr int32_t kProsErr = INT32_MAX;

    // faulted() is what stops a held value being averaged in with live ones:
    // that halves the reported distance per dead wheel and trips no jump guard.
    if (mType == SensorType::ADI && mAdi != nullptr) {
        // ADI encoder: ticks → revolutions → distance
        int32_t rawTicks = mAdi->get_value();
        if (rawTicks == kProsErr) {
            mFaulted = true;
            ++mFaults;
            return mLastGoodDistIn;
        }
        float revolutions = static_cast<float>(rawTicks) / (ADI_TICKS_PER_REV * mGearRatio);
        mLastGoodDistIn = revolutions * circumference;
        mFaulted = false;
        return mLastGoodDistIn;
    } else if (mType == SensorType::Rotation && mRot != nullptr) {
        // Rotation sensor: centidegrees → revolutions → distance
        int32_t rawCentideg = mRot->get_position();
        if (rawCentideg == kProsErr) {
            mFaulted = true;
            ++mFaults;
            return mLastGoodDistIn;
        }
        float revolutions = static_cast<float>(rawCentideg) / (ROTATION_UNITS_PER_REV * mGearRatio);
        mLastGoodDistIn = revolutions * circumference;
        mFaulted = false;
        return mLastGoodDistIn;
    }
    mFaulted = true;
    return 0.0f;
}

float TrackingWheel::rawTicks() const {
    if (mType == SensorType::ADI && mAdi != nullptr) {
        return static_cast<float>(mAdi->get_value());
    }
    return 0.0f;
}

float TrackingWheel::rawDegrees() const {
    if (mType == SensorType::Rotation && mRot != nullptr) {
        return static_cast<float>(mRot->get_position()) * 0.01f;  // centidegrees → degrees
    }
    return 0.0f;
}

void TrackingWheel::reset() {
    if (mType == SensorType::ADI && mAdi != nullptr) {
        mAdi->reset();
    } else if (mType == SensorType::Rotation && mRot != nullptr) {
        mRot->reset_position();
    }
}

// ============================================================================
// OdomTracking
// ============================================================================

OdomTracking::OdomTracking() {
    mCore.rebaseline(currentVertDistance(), currentHorizDistance(),
                     currentHeadingRad());
}

OdomTracking::OdomTracking(const OdomSensors& sensors)
    : mSensors(sensors) {
    // Baseline from the current readings so the first update() computes a
    // near-zero delta. Without this the first update sees a jump equal to the
    // entire accumulated distance plus a 90 degree heading offset (IMU 0 VEX is
    // PI/2 standard radians).
    mCore.rebaseline(currentVertDistance(), currentHorizDistance(),
                     currentHeadingRad());
}

OdomTracking::~OdomTracking() {
    stopBackgroundOdom();
}

void OdomTracking::setSensors(const OdomSensors& sensors) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mSensors = sensors;
    mCore.rebaseline(currentVertDistance(), currentHorizDistance(),
                     currentHeadingRad());
}

// ============================================================================
// Sensor read helpers — each has a fallback chain
// ============================================================================

float OdomTracking::currentVertDistance() const {
    // 1. External tracking wheels
    if (!mSensors.vertWheelCollection.empty()) {
        return averageDistance(mSensors.vertWheelCollection);
    }
    // 2. Integrated motor encoders (drivetrain fallback)
    if (mSensors.drivetrain != nullptr) {
        const EncoderAverage avg = mSensors.drivetrain->averagePositionDegLive();
        if (avg.faulted > 0) ++mDistanceFaults;
        if (!avg.valid()) {
            // Every encoder is dead. The scalar accessor returns 0.0 here,
            // which a consumer cannot tell from standing at the origin: the
            // first tick reads as one big jump and gets rejected, and every
            // tick after reads a consistent zero, so the pose sits still while
            // the robot drives. Hold instead, and count it.
            ++mDistanceOutages;
            return mLastGoodMotorVertIn;
        }
        // Motor encoders measure the MOTOR shaft. With an external gear ratio
        // (e.g. 600 RPM blue cartridge driving a 450 RPM wheel), one motor rev
        // is only wheelRPM/cartridgeRPM wheel revs. Omitting this factor
        // overestimated distance by 33% on a 600→450 geared drive.
        const float wheelRevolutions = (avg.value / 360.0f) *
            mSensors.drivetrain->externalGearRatio();
        mLastGoodMotorVertIn =
            wheelRevolutions * mSensors.drivetrain->wheelDiameter * PI;
        return mLastGoodMotorVertIn;
    }
    // 3. No vertical tracking source
    return 0.0f;
}

float OdomTracking::currentHorizDistance() const {
    // 1. External tracking wheels
    if (!mSensors.horizWheelCollection.empty()) {
        return averageDistance(mSensors.horizWheelCollection);
    }
    // 2. Tank drives cannot measure lateral movement from motor encoders
    return 0.0f;
}

bool OdomTracking::hasImu() const {
    return !mSensors.imuCollection.empty() && mSensors.imuCollection[0] != nullptr;
}

bool OdomTracking::imuHeadingRad(float& out) const {
    if (!hasImu()) {
        out = mCore.prevHeadingRad();
        return false;
    }
    const float rawHeading = mSensors.imuCollection[0]->get_heading();
    // PROS returns PROS_ERR_F (== INFINITY) on an IMU fault (disconnect,
    // transient error, mid-calibration). Feeding that downstream corrupts the
    // pose; hold the last good heading instead of integrating garbage.
    if (!std::isfinite(rawHeading)) {
        ++mImuFaults;
        out = mCore.prevHeadingRad();
        return false;
    }
    // Calibration scale factor: V5 IMUs typically read ~354.25 degrees for a
    // 360 degree physical turn.
    out = vexToStdRad(rawHeading * mSensors.imuScaleFactor);
    return true;
}

float OdomTracking::currentHeadingRad() const {
    float heading = 0.0f;
    imuHeadingRad(heading);
    return heading;
}

// ============================================================================
// update() — one odometry tick
// ============================================================================

void OdomTracking::update(bool force) {
    if (!mBackgroundUpdatesEnabled && !force) {
        return;
    }

    std::lock_guard<pros::Mutex> lock(mMutex);
    float headingRad = 0.0f;
    const bool headingValid = imuHeadingRad(headingRad);
    mCore.step(currentVertDistance(), currentHorizDistance(), headingRad,
               headingValid);
}

OdomDelta OdomTracking::consumeDelta(int channel) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.consumeDelta(channel);
}

void OdomTracking::setBackgroundUpdatesEnabled(bool enabled) {
    mBackgroundUpdatesEnabled = enabled;
}

bool OdomTracking::backgroundUpdatesEnabled() const {
    return mBackgroundUpdatesEnabled;
}

void OdomTracking::startBackgroundOdom() {
    if (mTask != nullptr) {
        mBackgroundUpdatesEnabled = true;
        return;
    }

    mBackgroundUpdatesEnabled = true;
    mRunning = true;
    mTask = new pros::Task([this]() { run(); });
}

void OdomTracking::stopBackgroundOdom() {
    mBackgroundUpdatesEnabled = false;
    mRunning = false;
    if (mTask != nullptr) {
        pros::delay(20);
        mTask->remove();
        delete mTask;
        mTask = nullptr;
    }
}

bool OdomTracking::isBackgroundOdomRunning() const {
    return mRunning && mTask != nullptr;
}

void OdomTracking::taskLoop(void* param) {
    auto* odom = static_cast<OdomTracking*>(param);
    if (odom != nullptr) {
        odom->run();
    }
}

void OdomTracking::run() {
    while (mRunning) {
        if (mBackgroundUpdatesEnabled) {
            update(false);
        }
        pros::delay(10);
    }
}

void OdomTracking::setPose(const Pose& pose) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    // Pose only. Re-baselining the encoders here deletes the distance driven
    // since the last tick, and with a filter writing the pose back every cycle
    // that is a standing fraction of every inch travelled. The heading baseline
    // must not move either: it tracks the raw IMU, not the pose.
    mCore.setPose(pose);
}

Pose OdomTracking::getPose() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.pose();
}

void OdomTracking::reset() {
    std::lock_guard<pros::Mutex> lock(mMutex);
    // Reset all tracking wheels (if present)
    for (auto* wheel : mSensors.vertWheelCollection) {
        if (wheel != nullptr) wheel->reset();
    }
    for (auto* wheel : mSensors.horizWheelCollection) {
        if (wheel != nullptr) wheel->reset();
    }
    mCore.setPose(Pose{});
    // Re-baseline: the wheels really were zeroed above, and motor encoders
    // cannot be, so store the current reading either way. Reading the IMU keeps
    // the next update()'s delta near zero; baselining it at 0.0f would cause a
    // 90 degree spurious rotation (IMU 0 VEX is PI/2 rad).
    mCore.rebaseline(currentVertDistance(), currentHorizDistance(),
                     currentHeadingRad());
    // Drop any motion accumulated before the reset: it belongs to the old
    // baseline and would otherwise be applied to consumers after re-zeroing.
    mCore.clearDeltas();
}

void OdomTracking::calibrate() {
    // Calibrate all IMUs.
    // Use blocking mode (true) so we don't need a blind delay — the call returns
    // only after calibration completes (~2s).  Without blocking, reset(false)
    // starts calibration but returns immediately, leading to premature reads.
    for (auto* imu : mSensors.imuCollection) {
        if (imu != nullptr) {
            imu->reset(true);
        }
    }
    reset();
}

float OdomTracking::imuHeadingDeg() const {
    // Route through currentHeadingRad() so this shares its PROS_ERR_F guard.
    // Unguarded, one bad read became vexToStdRad(inf) == 0, a plausible-looking
    // heading that MCL then clamps every particle to.
    return stdRadToVexDeg(currentHeadingRad());
}

float OdomTracking::imuHeadingRad() const {
    return currentHeadingRad();
}

uint32_t OdomTracking::jumpRejects() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.jumpRejects();
}

uint32_t OdomTracking::headingJumpRejects() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.headingJumpRejects();
}

uint32_t OdomTracking::distanceFaults() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mDistanceFaults;
}

uint32_t OdomTracking::distanceOutages() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mDistanceOutages;
}

OdomCore::Config OdomTracking::coreConfig() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mCore.config();
}

void OdomTracking::setCoreConfig(const OdomCore::Config& config) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mCore.setConfig(config);
}

float OdomTracking::averageDistance(const std::vector<TrackingWheel*>& wheels) const {
    if (wheels.empty()) return 0.0f;

    float values[8];
    bool bad[8];
    float held = 0.0f;
    int n = 0;
    for (const auto* wheel : wheels) {
        if (wheel == nullptr || n >= 8) continue;
        values[n] = wheel->distanceIn();
        bad[n] = wheel->faulted();
        held += values[n];
        n++;
    }
    if (n == 0) return 0.0f;

    const DistanceAverage avg = averageLiveDistance(values, bad, n);
    if (avg.faulted > 0) ++mDistanceFaults;
    if (!avg.valid()) {
        // Nothing live. Hold, so odometry freezes for the tick instead of
        // stepping to a different sensor set.
        ++mDistanceOutages;
        return held / static_cast<float>(n);
    }
    return avg.value;
}

float OdomTracking::averageOffset(const std::vector<TrackingWheel*>& wheels) {
    if (wheels.empty()) return 0.0f;

    float sum = 0.0f;
    int count = 0;
    for (const auto* wheel : wheels) {
        if (wheel != nullptr) {
            sum += wheel->offsetIn();
            ++count;
        }
    }
    return (count > 0) ? (sum / static_cast<float>(count)) : 0.0f;
}

}  // namespace chad
