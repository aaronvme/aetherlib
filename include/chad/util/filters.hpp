#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace chad {
class SlewRateLimiter {
public:
    SlewRateLimiter() = default;
    SlewRateLimiter(float risePerSec, float fallPerSec, float initial = 0.0f)
        : mRise(risePerSec), mFall(fallPerSec), mValue(initial) {}

    explicit SlewRateLimiter(float ratePerSec) : SlewRateLimiter(ratePerSec, ratePerSec) {}

    float calculate(float input, float dt) {
        if (!std::isfinite(input) || !(dt > 0.0f)) return mValue;
        const float delta = input - mValue;
        // Which way this step drives the magnitude, not where the target
        // ends up: comparing |input| to |mValue| put the whole of a sign
        // reversal on the rise rate.
        const bool growing = (mValue == 0.0f) || (delta * mValue > 0.0f);
        const float limit = (growing ? mRise : mFall) * dt;
        if (limit <= 0.0f) return mValue = input;
        mValue += std::clamp(delta, -limit, limit);
        return mValue;
    }

    void reset(float value) { mValue = value; }
    float value() const { return mValue; }

    void setRates(float risePerSec, float fallPerSec) {
        mRise = risePerSec;
        mFall = fallPerSec;
    }

private:
    float mRise{0.0f};
    float mFall{0.0f};
    float mValue{0.0f};
};

template <std::size_t N>
class MovingAverage {
    static_assert(N > 0, "window must be non-empty");

public:
    float calculate(float input) {
        if (!std::isfinite(input)) return value();
        if (mCount < N) {
            mSum += input;
            mBuf[mHead] = input;
            mCount++;
        } else {
            mSum += input - mBuf[mHead];
            mBuf[mHead] = input;
        }
        mHead = (mHead + 1) % N;
        return value();
    }

    float value() const { return mCount == 0 ? 0.0f : mSum / static_cast<float>(mCount); }
    bool full() const { return mCount >= N; }
    std::size_t count() const { return mCount; }

    void reset() {
        mSum = 0.0f;
        mHead = 0;
        mCount = 0;
        mBuf.fill(0.0f);
    }

private:
    std::array<float, N> mBuf{};
    float mSum{0.0f};
    std::size_t mHead{0};
    std::size_t mCount{0};
};

template <std::size_t N>
class MedianFilter {
    static_assert(N > 0, "window must be non-empty");

public:
    float calculate(float input) {
        if (!std::isfinite(input)) return mLast;
        mBuf[mHead] = input;
        mHead = (mHead + 1) % N;
        if (mCount < N) mCount++;

        std::array<float, N> sorted = mBuf;
        const auto begin = sorted.begin();
        const auto end = begin + static_cast<std::ptrdiff_t>(mCount);
        std::sort(begin, end);

        const std::size_t mid = mCount / 2;
        mLast = (mCount % 2 == 1) ? sorted[mid] : 0.5f * (sorted[mid - 1] + sorted[mid]);
        return mLast;
    }

    float value() const { return mLast; }
    bool full() const { return mCount >= N; }

    void reset() {
        mBuf.fill(0.0f);
        mHead = 0;
        mCount = 0;
        mLast = 0.0f;
    }

private:
    std::array<float, N> mBuf{};
    std::size_t mHead{0};
    std::size_t mCount{0};
    float mLast{0.0f};
};

class LowPassFilter {
public:
    LowPassFilter() = default;
    explicit LowPassFilter(float timeConstantSeconds) : mTau(timeConstantSeconds) {}

    float calculate(float input, float dt) {
        if (!std::isfinite(input)) return mValue;
        // No time constant is no filtering, which really is a pass-through.
        if (mTau <= 0.0f) return mValue = input;
        // No elapsed time is no new information -- alpha would be zero, so
        // hold. Handing back the raw input wiped the state every time two
        // samples landed inside one millisecond tick.
        if (!(dt > 0.0f)) return mValue;
        if (!mPrimed) {
            mPrimed = true;
            return mValue = input;
        }
        const float alpha = dt / (mTau + dt);
        mValue += alpha * (input - mValue);
        return mValue;
    }

    void setTimeConstant(float seconds) { mTau = seconds; }
    float timeConstant() const { return mTau; }
    float value() const { return mValue; }

    /// Drop the state; the next sample is taken as the new output.
    void reset() {
        mValue = 0.0f;
        mPrimed = false;
    }

    /// Seed the output. Unlike reset(), the next sample is filtered against
    /// this value instead of replacing it -- otherwise the seed could never
    /// reach an output.
    void reset(float value) {
        mValue = value;
        mPrimed = true;
    }

private:
    float mTau{0.0f};
    float mValue{0.0f};
    bool mPrimed{false};
};

class HighPassFilter {
public:
    HighPassFilter() = default;
    explicit HighPassFilter(float timeConstantSeconds) : mLow(timeConstantSeconds) {}

    float calculate(float input, float dt) {
        return input - mLow.calculate(input, dt);
    }

    void setTimeConstant(float seconds) { mLow.setTimeConstant(seconds); }
    void reset() { mLow.reset(); }
    void reset(float value) { mLow.reset(value); }

private:
    LowPassFilter mLow;
};

class Derivative {
public:
    Derivative() = default;
    explicit Derivative(float timeConstantSeconds) : mSmooth(timeConstantSeconds) {}

    float calculate(float input, float dt) {
        if (!std::isfinite(input) || !(dt > 0.0f)) return mValue;
        if (!mPrimed) {
            mPrev = input;
            mPrimed = true;
            return mValue = 0.0f;
        }
        const float raw = (input - mPrev) / dt;
        mPrev = input;
        mValue = mSmooth.timeConstant() > 0.0f ? mSmooth.calculate(raw, dt) : raw;
        return mValue;
    }

    float value() const { return mValue; }

    void reset() {
        mPrimed = false;
        mValue = 0.0f;
        mSmooth.reset();
    }

private:
    LowPassFilter mSmooth;
    float mPrev{0.0f};
    float mValue{0.0f};
    bool mPrimed{false};
};

class Debouncer {
public:
    enum class Edge { Rising, Falling, Both };

    Debouncer() = default;
    explicit Debouncer(float debounceSeconds, Edge edge = Edge::Rising,
                       bool initial = false)
        : mDebounce(debounceSeconds), mEdge(edge), mState(initial) {}

    bool calculate(bool input, float dt) {
        if (input == mState) {
            mElapsed = 0.0f;
            return mState;
        }
        const bool watched = (mEdge == Edge::Both) ||
                             (mEdge == Edge::Rising && input) ||
                             (mEdge == Edge::Falling && !input);
        if (!watched) {
            mState = input;
            mElapsed = 0.0f;
            return mState;
        }
        if (dt > 0.0f) mElapsed += dt;
        if (mElapsed >= mDebounce) {
            mState = input;
            mElapsed = 0.0f;
        }
        return mState;
    }

    bool value() const { return mState; }

    void reset(bool state) {
        mState = state;
        mElapsed = 0.0f;
    }

private:
    float mDebounce{0.0f};
    Edge mEdge{Edge::Rising};
    bool mState{false};
    float mElapsed{0.0f};
};
}
