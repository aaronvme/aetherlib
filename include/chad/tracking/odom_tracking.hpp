#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

#include "chad/tracking/odom_inputs.hpp"
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

    float distanceIn() const;   // holds the last good value on a sensor error

    /// NaN when the sensor cannot be read.
    float readDistanceIn() const;

    float offsetIn() const { return mOffsetIn; }

    float wheelDiamIn() const { return mWheelDiamIn; }

    float gearRatio() const { return mGearRatio; }

    SensorType sensorType() const { return mType; }

    /// The rotation sensor behind a Rotation wheel, nullptr for ADI.
    const pros::Rotation* rotation() const { return mRot; }

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

    /// Guarded heading as of the last update(), setPose() or reset(), VEX degrees.
    float imuHeadingDeg() const;

    /// Unreadable IMU reads, plus readings re-anchored after a gap or a step
    /// the gyro cannot make (a restart after a brown-out).
    uint32_t imuFaults() const { return mImuFaults.load(); }

    float imuHeadingRad() const;

private:
    OdomSensors mSensors;
    Pose mPose;

    // Guards the state below. Sensors are never read while it is held: a
    // task deleted mid-read (a competition mode change) would orphan it.
    mutable pros::Mutex mMutex;

    // The devices of mSensors, copied out under mMutex so a reader never
    // walks the vectors setSensors() replaces. gen changes with every
    // setSensors(); a sample read against an older set is not integrated.
    struct Devices {
        TrackingWheel* vert[EncoderDeltaMean::kMaxEncoders]{};
        TrackingWheel* horiz[EncoderDeltaMean::kMaxEncoders]{};
        int nVert{0};
        int nHoriz{0};
        float vertOffsetIn{0.0f};
        float horizOffsetIn{0.0f};
        pros::Imu* imu{nullptr};
        Drivetrain* drivetrain{nullptr};
        std::uint32_t gen{0};
    };
    Devices mDevices;

    struct Sample {
        std::uint64_t us{0};
        float rawHeadingRad{0.0f};   // NaN when the IMU cannot be read
        float inPerMotorDeg{0.0f};
        int nVert{0};
        int nHoriz{0};
        int nLeft{0};
        int nRight{0};
        double vert[EncoderDeltaMean::kMaxEncoders];    // inches, NaN if unreadable
        double horiz[EncoderDeltaMean::kMaxEncoders];
        double left[EncoderDeltaMean::kMaxEncoders];    // motor degrees
        double right[EncoderDeltaMean::kMaxEncoders];
    };

    EncoderDeltaMean mVertWheels;
    EncoderDeltaMean mHorizWheels;
    EncoderDeltaMean mLeftDrive;
    EncoderDeltaMean mRightDrive;
    std::uint64_t mLastSampleUs{0};

    // Heading from the IMU's boot zero to the field frame, set by setPose.
    ImuHeadingGuard mHeading;

    mutable std::atomic<uint32_t> mImuFaults{0};
    std::atomic<bool> mBackgroundUpdatesEnabled{false};

    OdomDelta mLastDelta;

    OdomDelta mAccumDelta[DELTA_CHANNELS];

    static Devices devicesOf(const OdomSensors& sensors);
    Devices devices() const;
    static int readWheels(TrackingWheel* const* wheels, int n, double* out);
    static float averageOffset(const std::vector<TrackingWheel*>& wheels);

    void readSample(const Devices& dev, Sample& out) const;
    void rebase(const Sample& s);
    void integrate(const Sample& s);
    float driveTravelIn(const Sample& s, float dtSec, float dThetaRad);

    float rawHeadingRad(const Devices& dev) const;

    float currentHeadingRad() const;

    static void taskLoop(void* param);
    void run();

    pros::Task* mTask{nullptr};

    std::atomic<bool> mRunning{false};
};
}
