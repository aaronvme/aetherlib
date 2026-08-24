#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "chad/util/geometry.hpp"

namespace chad {

/// Fixed-capacity history of timestamped poses, interpolated on read. Nothing
/// allocates, so it is safe to keep in a control task.
template <int Capacity>
class PoseBuffer {
    static_assert(Capacity >= 2, "a history needs at least two samples");

public:
    void clear() {
        mCount = 0;
        mHead = 0;
    }

    bool empty() const { return mCount == 0; }
    int size() const { return mCount; }

    uint32_t newestMs() const { return mCount == 0 ? 0 : at(mCount - 1).timeMs; }
    uint32_t oldestMs() const { return mCount == 0 ? 0 : at(0).timeMs; }

    /// Samples must arrive in order; one that goes backwards in time drops the
    /// history after it, because an out-of-order odometry pose is a reset.
    void add(uint32_t timeMs, const Pose& pose) {
        while (mCount > 0 && !isAfter(timeMs, at(mCount - 1).timeMs)) mCount--;
        if (mCount == Capacity) {
            mHead = (mHead + 1) % Capacity;
            mCount--;
        }
        entry((mHead + mCount) % Capacity) = {timeMs, pose};
        mCount++;
    }

    /// Drop everything older than `windowMs` before the newest sample.
    void prune(uint32_t windowMs) {
        if (mCount == 0) return;
        const uint32_t newest = newestMs();
        while (mCount > 1 && newest - at(0).timeMs > windowMs) {
            mHead = (mHead + 1) % Capacity;
            mCount--;
        }
    }

    /// Pose at `timeMs`, interpolated along the arc between the bracketing
    /// samples. Clamps to the ends of the history; false only when empty.
    bool sample(uint32_t timeMs, Pose& out) const {
        if (mCount == 0) return false;
        if (!isAfter(timeMs, at(0).timeMs)) {
            out = at(0).pose;
            return true;
        }
        if (!isAfter(at(mCount - 1).timeMs, timeMs)) {
            out = at(mCount - 1).pose;
            return true;
        }

        int lo = 0, hi = mCount - 1;
        while (hi - lo > 1) {
            const int mid = (lo + hi) / 2;
            if (isAfter(at(mid).timeMs, timeMs)) {
                hi = mid;
            } else {
                lo = mid;
            }
        }

        const Entry& a = at(lo);
        const Entry& b = at(hi);
        const uint32_t span = b.timeMs - a.timeMs;
        if (span == 0) {
            out = b.pose;
            return true;
        }
        const float t = static_cast<float>(timeMs - a.timeMs) / static_cast<float>(span);
        out = a.pose.interpolate(b.pose, t);
        return true;
    }

    /// True if a measurement stamped `timeMs` is inside the retained history.
    bool covers(uint32_t timeMs) const {
        return mCount > 0 && !isAfter(oldestMs(), timeMs);
    }

private:
    struct Entry {
        uint32_t timeMs{0};
        Pose pose{};
    };

    /// Unsigned difference, so a millisecond counter that wraps is still ordered.
    static bool isAfter(uint32_t a, uint32_t b) {
        return a != b && (a - b) < 0x80000000u;
    }

    const Entry& at(int i) const { return mEntries[(mHead + i) % Capacity]; }
    Entry& entry(int i) { return mEntries[i]; }

    std::array<Entry, Capacity> mEntries{};
    int mHead{0};
    int mCount{0};
};

/// Fuses dead reckoning with absolute pose measurements that are reported late.
///
/// A wall-range or vision fix describes where the robot WAS when the reading
/// was taken. Applying it to the present pose drags in every inch driven since,
/// which on a moving robot is the dominant error. This rewinds to the stamped
/// instant, applies the correction there, then replays the odometry that
/// followed, so a late measurement costs nothing but its own noise.
class LatencyPoseEstimator {
public:
    static constexpr int HISTORY_CAPACITY = 192;

    struct Config {
        /// How far back a measurement may be stamped and still be used.
        uint32_t historyMs{1500};
        /// Trust in dead reckoning, as a standard deviation per axis:
        /// x and y in inches, heading in radians. Larger means the filter
        /// leans harder on measurements.
        float stateStdDev[3]{1.0f, 1.0f, 0.10f};
    };

    LatencyPoseEstimator() { setConfig(Config{}); }
    explicit LatencyPoseEstimator(const Config& config) { setConfig(config); }

    void setConfig(const Config& config) {
        mConfig = config;
        for (int i = 0; i < 3; i++) {
            const float s = mConfig.stateStdDev[i];
            mQ[i] = s > 0.0f ? s * s : 0.0f;
        }
    }
    const Config& config() const { return mConfig; }

    void reset(const Pose& pose, uint32_t nowMs) {
        mOdom = pose;
        mEstimate = pose;
        mHistory.clear();
        mHistory.add(nowMs, pose);
    }

    /// One dead-reckoning tick, as the robot-frame twist just travelled.
    void update(uint32_t nowMs, const Twist2d& twist) {
        mOdom = mOdom.exp(twist);
        mEstimate = mEstimate.exp(twist);
        mHistory.add(nowMs, mOdom);
        mHistory.prune(mConfig.historyMs);
    }

    /// Absolute pose measurement valid at `stampMs`, with its own per-axis
    /// standard deviations (inches, inches, radians). Returns false if the
    /// stamp is outside the retained history or the deviations are unusable.
    bool addMeasurement(uint32_t stampMs, const Pose& measured, const float stdDev[3]) {
        if (!std::isfinite(measured.x) || !std::isfinite(measured.y) ||
            !std::isfinite(measured.theta)) {
            return false;
        }
        if (!mHistory.covers(stampMs)) return false;

        Pose odomAtStamp{};
        if (!mHistory.sample(stampMs, odomAtStamp)) return false;

        float gain[3];
        for (int i = 0; i < 3; i++) {
            const float r = (stdDev != nullptr && stdDev[i] > 0.0f) ? stdDev[i] * stdDev[i]
                                                                    : 0.0f;
            if (mQ[i] <= 0.0f) {
                gain[i] = 0.0f;          // this axis is not to be corrected
            } else if (r <= 0.0f) {
                gain[i] = 1.0f;          // a measurement with no noise wins outright
            } else {
                gain[i] = mQ[i] / (mQ[i] + std::sqrt(mQ[i] * r));
            }
        }

        // Rewind, correct, replay: the transform from the present odometry pose
        // back to the stamped one carries the estimate with it.
        const Transform2d backToStamp = mOdom.transformTo(odomAtStamp);
        const Pose estimateAtStamp = mEstimate.transformBy(backToStamp);

        const Twist2d error = estimateAtStamp.log(measured);
        const Pose corrected = estimateAtStamp.exp(
            {error.dx * gain[0], error.dy * gain[1], error.dtheta * gain[2]});

        mEstimate = corrected.transformBy(backToStamp.inverse());
        return true;
    }

    Pose pose() const { return mEstimate; }
    Pose odometryPose() const { return mOdom; }
    int historySize() const { return mHistory.size(); }

private:
    Config mConfig{};
    float mQ[3]{};
    Pose mOdom{};
    Pose mEstimate{};
    PoseBuffer<HISTORY_CAPACITY> mHistory;
};

}  // namespace chad
