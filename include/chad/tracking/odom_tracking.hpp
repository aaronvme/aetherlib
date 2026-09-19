#pragma once

#include <atomic>
#include <vector>

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

#include "chad/util/util.hpp"

namespace chad {
class Drivetrain;

class TrackingWheel {
public:
    enum class SensorType { ADI, Rotation };

    TrackingWheel(pros::adi::Encoder& enc,
                  float wheelDiamIn,
                  float offsetIn,
                  float gearRatio = 1.0f);

    TrackingWheel(pros::Rotation& rot,
                  float wheelDiamIn,
                  float offsetIn,
                  float gearRatio = 1.0f);

    float distanceIn() const;

    float offsetIn() const { return mOffsetIn; }

    float wheelDiamIn() const { return mWheelDiamIn; }

    float gearRatio() const { return mGearRatio; }

    SensorType sensorType() const { return mType; }

    void reset();

    float rawTicks() const;
    float rawDegrees() const;

private:
    SensorType mType;
    pros::adi::Encoder* mAdi{nullptr};
    pros::Rotation* mRot{nullptr};

    float mWheelDiamIn{0.0f};
    float mOffsetIn{0.0f};
    float mGearRatio{1.0f};

    mutable float mLastGoodDistIn{0.0f};
};

struct OdomSensors {
    std::vector<TrackingWheel*> vertWheelCollection;
    std::vector<TrackingWheel*> horizWheelCollection;
    std::vector<pros::Imu*> imuCollection;

    Drivetrain* drivetrain{nullptr};

    float imuScaleFactor{1.0f};
};

class OdomTracking {
public:
    OdomTracking();
    OdomTracking(const OdomSensors& sensors);
    ~OdomTracking();

    void setSensors(const OdomSensors& sensors);

    void update(bool force = false);

    void setBackgroundUpdatesEnabled(bool enabled);

    bool backgroundUpdatesEnabled() const;

    void startBackgroundOdom();

    void stopBackgroundOdom();

    bool isBackgroundOdomRunning() const;

    OdomDelta getLastDelta() const { return mLastDelta; }

    enum DeltaChannel { CHANNEL_MCL = 0, CHANNEL_EKF = 1, DELTA_CHANNELS = 2 };

    OdomDelta consumeDelta(int channel = CHANNEL_MCL);

    bool hasHorizontalTracking() const {
        return !mSensors.horizWheelCollection.empty();
    }

    bool hasVerticalTrackingWheel() const {
        return !mSensors.vertWheelCollection.empty();
    }

    void setPose(const Pose& pose);

    /// Position-only correction from a filter. Which way zero points is
    /// setPose's business, not an estimator's.
    void syncPose(const Pose& pose);
    Pose getPose() const;

    void reset();

    void calibrate();

    const OdomSensors& sensors() const { return mSensors; }

    float imuHeadingDeg() const;

    uint32_t imuFaults() const { return mImuFaults; }

    float imuHeadingRad() const;

private:
    OdomSensors mSensors;
    Pose mPose;

    mutable pros::Mutex mMutex;

    float mPrevVertDist{0.0f};
    float mPrevHorizDist{0.0f};
    float mPrevHeadingRad{0.0f};

    // Rotation from the IMU's boot zero to the field frame, set by setPose.
    float mHeadingOffsetRad{0.0f};

    mutable uint32_t mImuFaults{0};
    std::atomic<bool> mBackgroundUpdatesEnabled{false};

    OdomDelta mLastDelta;

    OdomDelta mAccumDelta[DELTA_CHANNELS];

    static float averageDistance(const std::vector<TrackingWheel*>& wheels);
    static float averageOffset(const std::vector<TrackingWheel*>& wheels);

    float currentVertDistance() const;

    float currentHorizDistance() const;

    float rawHeadingRad() const;

    float currentHeadingRad() const;

    static void taskLoop(void* param);
    void run();

    pros::Task* mTask{nullptr};

    std::atomic<bool> mRunning{false};
};
}
