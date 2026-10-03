#include "chad/tracking/odom_tracking.hpp"

#include <cmath>
#include <cstdint>
#include <mutex>
#include <utility>

#include "chad/drivetrain.hpp"
#include "pros/error.h"
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

float TrackingWheel::readDistanceIn() const {
    constexpr int32_t kProsErr = INT32_MAX;
    const float circumference = mWheelDiamIn * PI;

    if (mType == SensorType::ADI && mAdi != nullptr) {
        const int32_t rawTicks = mAdi->get_value();
        if (rawTicks == kProsErr) return NAN;
        return static_cast<float>(rawTicks) / (ADI_TICKS_PER_REV * mGearRatio) * circumference;
    } else if (mType == SensorType::Rotation && mRot != nullptr) {
        const int32_t rawCentideg = mRot->get_position();
        if (rawCentideg == kProsErr) return NAN;
        return static_cast<float>(rawCentideg) / (ROTATION_UNITS_PER_REV * mGearRatio) * circumference;
    }
    return 0.0f;
}

float TrackingWheel::distanceIn() const {
    const float d = readDistanceIn();
    if (std::isfinite(d)) mLastGoodDistIn = d;
    return mLastGoodDistIn;
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
    Sample s;
    readSample(mDevices, s);
    mHeading.seed(s.rawHeadingRad);
    rebase(s);
}

OdomTracking::OdomTracking(const OdomSensors& sensors)
    : mSensors(sensors), mPose(), mDevices(devicesOf(sensors)) {
    Sample s;
    readSample(mDevices, s);
    mHeading.seed(s.rawHeadingRad);
    rebase(s);
}

OdomTracking::~OdomTracking() {
    stopBackgroundOdom();
}

void OdomTracking::setSensors(const OdomSensors& sensors) {
    OdomSensors next = sensors;
    Devices dev = devicesOf(next);
    Sample s;
    readSample(dev, s);

    std::lock_guard<pros::Mutex> lock(mMutex);
    std::swap(mSensors, next);
    dev.gen = mDevices.gen + 1;
    mDevices = dev;
    mHeading.anchor(s.rawHeadingRad, mHeading.headingRad);
    rebase(s);
}

OdomTracking::Devices OdomTracking::devicesOf(const OdomSensors& sensors) {
    Devices d;
    for (auto* w : sensors.vertWheelCollection) {
        if (d.nVert == EncoderDeltaMean::kMaxEncoders) break;
        d.vert[d.nVert++] = w;
    }
    for (auto* w : sensors.horizWheelCollection) {
        if (d.nHoriz == EncoderDeltaMean::kMaxEncoders) break;
        d.horiz[d.nHoriz++] = w;
    }
    d.imu = sensors.imuCollection.empty() ? nullptr : sensors.imuCollection[0];
    d.drivetrain = sensors.drivetrain;
    return d;
}

OdomTracking::Devices OdomTracking::devices() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mDevices;
}

void OdomTracking::readSample(const Devices& dev, Sample& out) const {
    out.us = pros::micros();
    out.rawHeadingRad = rawHeadingRad(dev);

    out.nVert = readWheels(dev.vert, dev.nVert, out.vert);
    out.nHoriz = readWheels(dev.horiz, dev.nHoriz, out.horiz);
    if (out.nVert == 0 && dev.drivetrain != nullptr) {
        const Drivetrain& d = *dev.drivetrain;
        out.inPerMotorDeg = d.externalGearRatio() / 360.0f * d.wheelDiameter * PI;
        out.nLeft = Drivetrain::readPositionsDeg(d.leftSense(), out.left,
                                                 EncoderDeltaMean::kMaxEncoders);
        out.nRight = Drivetrain::readPositionsDeg(d.rightSense(), out.right,
                                                  EncoderDeltaMean::kMaxEncoders);
    }
}

void OdomTracking::rebase(const Sample& s) {
    mVertWheels.rebase(s.vert, s.nVert);
    mHorizWheels.rebase(s.horiz, s.nHoriz);
    mLeftDrive.rebase(s.left, s.nLeft);
    mRightDrive.rebase(s.right, s.nRight);
    mLastSampleUs = s.us;
}

float OdomTracking::rawHeadingRad(const Devices& dev) const {
    if (dev.imu != nullptr) {
        const float rawHeading = dev.imu->get_heading();

        if (!std::isfinite(rawHeading)) {
            ++mImuFaults;
            return NAN;
        }
        return vexToStdRad(rawHeading);
    }
    return 0.0f;
}

float OdomTracking::currentHeadingRad() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mHeading.headingRad;
}

float OdomTracking::driveTravelIn(const Sample& s, float dtSec, float dThetaRad) {
    const double maxStep = EncoderDeltaMean::maxStepDeg(dtSec);
    const double left  = mLeftDrive.update(s.left, s.nLeft, maxStep);
    const double right = mRightDrive.update(s.right, s.nRight, maxStep);
    const double arc = 0.5 * mDevices.drivetrain->trackWidth * dThetaRad;
    return static_cast<float>(differentialCentreTravel(
        left * s.inPerMotorDeg, mLeftDrive.contributing() > 0,
        right * s.inPerMotorDeg, mRightDrive.contributing() > 0, arc));
}

void OdomTracking::update(bool force) {
    if (!mBackgroundUpdatesEnabled && !force) {
        return;
    }

    const Devices dev = devices();
    Sample s;
    readSample(dev, s);
    std::lock_guard<pros::Mutex> lock(mMutex);
    if (dev.gen == mDevices.gen) integrate(s);
}

void OdomTracking::integrate(const Sample& s) {
    // Zero, not negative, when two updaters read in one order and lock in
    // the other: the guards then allow only a sample period's worth of change.
    const float dtSec = (s.us > mLastSampleUs)
        ? static_cast<float>(s.us - mLastSampleUs) * 1e-6f : 0.0f;
    mLastSampleUs = s.us;

    const float prevHeadingRad = mHeading.headingRad;
    if (!mHeading.update(s.rawHeadingRad, dtSec) && std::isfinite(s.rawHeadingRad)) {
        ++mImuFaults;
    }
    const float headingRad = mHeading.headingRad;
    const float dThetaRad = wrapRad(headingRad - prevHeadingRad);

    const double maxWheelStep = EncoderDeltaMean::maxStepIn(dtSec);
    float dVert = 0.0f;
    if (s.nVert > 0) {
        dVert = static_cast<float>(mVertWheels.update(s.vert, s.nVert, maxWheelStep));
    } else if (mDevices.drivetrain != nullptr) {
        dVert = driveTravelIn(s, dtSec, dThetaRad);
    }
    const float dHoriz =
        static_cast<float>(mHorizWheels.update(s.horiz, s.nHoriz, maxWheelStep));

    const float midHeadingRad = prevHeadingRad + dThetaRad * 0.5f;
    const float c = std::cos(midHeadingRad);
    const float sn = std::sin(midHeadingRad);

    mPose.x += dVert * c - dHoriz * sn;
    mPose.y += dVert * sn + dHoriz * c;
    mPose.theta = headingRad;

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

    // Above the default so the filters and motion loop that consume the pose
    // never read one a tick stale. run() sleeps every period.
    constexpr std::uint32_t kOdomTaskPriority = 8 + 2;   // TASK_PRIORITY_DEFAULT + 2

    mBackgroundUpdatesEnabled = true;
    mRunning = true;
    mTask = new pros::Task([this]() { run(); }, kOdomTaskPriority);
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
    constexpr std::uint32_t kPeriodMs = 10;
    std::uint32_t wake = pros::millis();
    while (mRunning) {
        if (mBackgroundUpdatesEnabled) {
            update(false);
        }
        // After an overrun sleep a full period instead of skipping the sleep
        // to catch up: this task outranks the ones it feeds.
        const std::uint32_t now = pros::millis();
        if (now - wake >= kPeriodMs) wake = now;
        pros::Task::delay_until(&wake, kPeriodMs);
    }
}

void OdomTracking::setPose(const Pose& pose) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) return;

    for (;;) {
        const Devices dev = devices();
        Sample s;
        readSample(dev, s);

        std::lock_guard<pros::Mutex> lock(mMutex);
        if (dev.gen != mDevices.gen) continue;
        mPose = pose;

        // The IMU cannot be told where north is, so the guard carries the
        // difference. Without it update() overwrites theta from the boot frame on
        // the next tick and the heading asked for is gone.
        mHeading.anchor(s.rawHeadingRad, pose.theta);

        // Rebase, or the next tick reads the jump as travel.
        rebase(s);
        return;
    }
}

void OdomTracking::syncPose(const Pose& pose) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y)) return;
    std::lock_guard<pros::Mutex> lock(mMutex);
    mPose.x = pose.x;
    mPose.y = pose.y;

    // No encoder rebase here, unlike setPose: this corrects the integrated
    // pose, it does not redefine where the wheels were last read. Rebasing
    // drops whatever was travelled since the last update() tick.
}

Pose OdomTracking::getPose() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mPose;
}

void OdomTracking::reset() {
    for (;;) {
        const Devices dev = devices();
        for (int i = 0; i < dev.nVert; ++i) {
            if (dev.vert[i] != nullptr) dev.vert[i]->reset();
        }
        for (int i = 0; i < dev.nHoriz; ++i) {
            if (dev.horiz[i] != nullptr) dev.horiz[i]->reset();
        }
        Sample s;
        readSample(dev, s);

        std::lock_guard<pros::Mutex> lock(mMutex);
        if (dev.gen != mDevices.gen) continue;
        mPose = Pose{};
        mHeading = ImuHeadingGuard{};
        mHeading.seed(s.rawHeadingRad);
        rebase(s);
        mPose.theta = mHeading.headingRad;

        mLastDelta = {};
        for (auto& acc : mAccumDelta) acc = {};
        return;
    }
}

void OdomTracking::calibrate() {
    for (auto* imu : mSensors.imuCollection) {
        if (imu != nullptr) {
            // PROS bounds a blocking reset at 3 s; a failed one leaves the
            // IMU uncalibrated for the whole match.
            for (int attempt = 0; attempt < 3; attempt++) {
                if (imu->reset(true) != PROS_ERR) break;
            }
            // 5 ms is the fastest the IMU streams; at the 10 ms default a
            // tick can read a sample a whole period old.
            imu->set_data_rate(5);
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

int OdomTracking::readWheels(TrackingWheel* const* wheels, int n, double* out) {
    for (int i = 0; i < n; ++i) {
        out[i] = (wheels[i] != nullptr) ? wheels[i]->readDistanceIn() : NAN;
    }
    return n;
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
