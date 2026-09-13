#include "chad/tracking/odom_tracking.hpp"

#include <cmath>
#include <mutex>

#include "chad/drivetrain.hpp"
#include "pros/rtos.hpp"

namespace chad {
static constexpr float ADI_TICKS_PER_REV = 360.0f;

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
    float circumference = mWheelDiamIn * PI;

    constexpr int32_t kProsErr = INT32_MAX;

    if (mType == SensorType::ADI && mAdi != nullptr) {
        int32_t rawTicks = mAdi->get_value();
        if (rawTicks == kProsErr) return mLastGoodDistIn;
        float revolutions = static_cast<float>(rawTicks) / (ADI_TICKS_PER_REV * mGearRatio);
        mLastGoodDistIn = revolutions * circumference;
        return mLastGoodDistIn;
    } else if (mType == SensorType::Rotation && mRot != nullptr) {
        int32_t rawCentideg = mRot->get_position();
        if (rawCentideg == kProsErr) return mLastGoodDistIn;
        float revolutions = static_cast<float>(rawCentideg) / (ROTATION_UNITS_PER_REV * mGearRatio);
        mLastGoodDistIn = revolutions * circumference;
        return mLastGoodDistIn;
    }
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
        return static_cast<float>(mRot->get_position()) * 0.01f;
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

OdomTracking::OdomTracking()
    : mPose() {
    mPrevVertDist   = currentVertDistance();
    mPrevHorizDist  = currentHorizDistance();
    mPrevHeadingRad = currentHeadingRad();
}

OdomTracking::OdomTracking(const OdomSensors& sensors)
    : mSensors(sensors), mPose() {
    mPrevVertDist   = currentVertDistance();
    mPrevHorizDist  = currentHorizDistance();
    mPrevHeadingRad = currentHeadingRad();
}

OdomTracking::~OdomTracking() {
    stopBackgroundOdom();
}

void OdomTracking::setSensors(const OdomSensors& sensors) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mSensors = sensors;
    mPrevVertDist  = currentVertDistance();
    mPrevHorizDist = currentHorizDistance();
    mPrevHeadingRad = currentHeadingRad();
}

float OdomTracking::currentVertDistance() const {
    if (!mSensors.vertWheelCollection.empty()) {
        return averageDistance(mSensors.vertWheelCollection);
    }

    if (mSensors.drivetrain != nullptr) {
        float avgDeg = mSensors.drivetrain->averagePositionDeg();
        float motorRevolutions = avgDeg / 360.0f;

        float wheelRevolutions = motorRevolutions *
            mSensors.drivetrain->externalGearRatio();
        return wheelRevolutions * mSensors.drivetrain->wheelDiameter * PI;
    }

    return 0.0f;
}

float OdomTracking::currentHorizDistance() const {
    if (!mSensors.horizWheelCollection.empty()) {
        return averageDistance(mSensors.horizWheelCollection);
    }

    return 0.0f;
}

float OdomTracking::rawHeadingRad() const {
    if (!mSensors.imuCollection.empty() && mSensors.imuCollection[0] != nullptr) {
        float rawHeading = mSensors.imuCollection[0]->get_heading();

        if (!std::isfinite(rawHeading)) {
            ++mImuFaults;
            return NAN;
        }

        float scaledHeading = rawHeading * mSensors.imuScaleFactor;
        return vexToStdRad(scaledHeading);
    }
    return 0.0f;
}

float OdomTracking::currentHeadingRad() const {
    const float raw = rawHeadingRad();
    if (!std::isfinite(raw)) return mPrevHeadingRad;
    return wrapRad(raw + mHeadingOffsetRad);
}

void OdomTracking::update(bool force) {
    if (!mBackgroundUpdatesEnabled && !force) {
        return;
    }

    std::lock_guard<pros::Mutex> lock(mMutex);
    float headingRad = currentHeadingRad();
    float vertDist   = currentVertDistance();
    float horizDist  = currentHorizDistance();

    float dThetaRad = headingRad - mPrevHeadingRad;
    dThetaRad = wrapRad(dThetaRad);

    float dVert  = vertDist  - mPrevVertDist;
    float dHoriz = horizDist - mPrevHorizDist;

    float midHeadingRad = mPrevHeadingRad + dThetaRad * 0.5f;

    float dX = 0.0f;
    float dY = 0.0f;

    if (std::fabs(dThetaRad) < 1e-6f) {
        dX = dVert * std::cos(mPrevHeadingRad) - dHoriz * std::sin(mPrevHeadingRad);
        dY = dVert * std::sin(mPrevHeadingRad) + dHoriz * std::cos(mPrevHeadingRad);
    } else {
        dX = dVert * std::cos(midHeadingRad) - dHoriz * std::sin(midHeadingRad);
        dY = dVert * std::sin(midHeadingRad) + dHoriz * std::cos(midHeadingRad);
    }

    mPose.x += dX;
    mPose.y += dY;
    mPose.theta = headingRad;

    mPrevVertDist   = vertDist;
    mPrevHorizDist  = horizDist;
    mPrevHeadingRad = headingRad;

    mLastDelta = {dVert, dHoriz, dThetaRad};

    for (auto& acc : mAccumDelta) {
        acc.dVert  += dVert;
        acc.dHoriz += dHoriz;
        acc.dTheta += dThetaRad;
    }
}

OdomDelta OdomTracking::consumeDelta(int channel) {
    if (channel < 0 || channel >= DELTA_CHANNELS) return {};
    std::lock_guard<pros::Mutex> lock(mMutex);
    OdomDelta out = mAccumDelta[channel];
    mAccumDelta[channel] = {};
    return out;
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
    mPose = pose;

    // The IMU cannot be told where north is, so carry the difference here
    // instead. Without it update() overwrites theta from the boot frame on
    // the next tick and the heading asked for is gone.
    const float raw = rawHeadingRad();
    if (std::isfinite(raw)) mHeadingOffsetRad = wrapRad(pose.theta - raw);

    // Rebase, or the next tick reads the offset jump as a real rotation.
    mPrevHeadingRad = currentHeadingRad();

    mPrevVertDist  = currentVertDistance();
    mPrevHorizDist = currentHorizDistance();
}

void OdomTracking::syncPose(const Pose& pose) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mPose.x = pose.x;
    mPose.y = pose.y;

    mPrevVertDist  = currentVertDistance();
    mPrevHorizDist = currentHorizDistance();
}

Pose OdomTracking::getPose() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mPose;
}

void OdomTracking::reset() {
    std::lock_guard<pros::Mutex> lock(mMutex);

    for (auto* wheel : mSensors.vertWheelCollection) {
        if (wheel != nullptr) wheel->reset();
    }
    for (auto* wheel : mSensors.horizWheelCollection) {
        if (wheel != nullptr) wheel->reset();
    }
    mPose = Pose{};
    mHeadingOffsetRad = 0.0f;

    mPrevVertDist   = currentVertDistance();
    mPrevHorizDist  = currentHorizDistance();

    mPrevHeadingRad = currentHeadingRad();
    mPose.theta     = mPrevHeadingRad;

    mLastDelta = {};
    for (auto& acc : mAccumDelta) acc = {};
}

void OdomTracking::calibrate() {
    for (auto* imu : mSensors.imuCollection) {
        if (imu != nullptr) {
            imu->reset(true);
        }
    }
    reset();
}

float OdomTracking::imuHeadingDeg() const {
    return stdRadToVexDeg(currentHeadingRad());
}

float OdomTracking::imuHeadingRad() const {
    return vexToStdRad(imuHeadingDeg());
}

float OdomTracking::averageDistance(const std::vector<TrackingWheel*>& wheels) {
    if (wheels.empty()) return 0.0f;

    float sum = 0.0f;
    int count = 0;
    for (const auto* wheel : wheels) {
        if (wheel != nullptr) {
            sum += wheel->distanceIn();
            ++count;
        }
    }
    return (count > 0) ? (sum / static_cast<float>(count)) : 0.0f;
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
}
