#include "chad/util/relay_core.hpp"

#include <cmath>

namespace chad::tuning {

namespace {
constexpr float kPi = 3.14159265358979323846f;

float mean(const float* v, int n) {
    if (n <= 0) return 0.0f;
    float s = 0.0f;
    for (int i = 0; i < n; i++) s += v[i];
    return s / static_cast<float>(n);
}
}  // namespace

RelayIdentifier::RelayIdentifier(const RelayIdentifyConfig& config)
    : mConfig(config) {}

float RelayIdentifier::update(float elapsedMs, float error) {
    if (mOutcome != RelayOutcome::Cycling) return 0.0f;

    if (std::fabs(error) > mConfig.abort) {
        mOutcome = RelayOutcome::RanAway;
        return 0.0f;
    }
    if (std::fabs(error) > mPeak) mPeak = std::fabs(error);

    // Opposes the displacement, or it runs away instead of cycling.
    float next = mSign;
    if (error > mConfig.hysteresis) next = -1.0f;
    else if (error < -mConfig.hysteresis) next = 1.0f;

    const float sinceSwitch =
        elapsedMs - (mLastSwitchMs >= 0.0f ? mLastSwitchMs : 0.0f);
    if (next != mSign && sinceSwitch >= mConfig.minHalfPeriodMs) {
        mSign = next;
        mSwitches++;
        // The first two switches are the start transient.
        if (mLastSwitchMs >= 0.0f && mSwitches > 2 &&
            mCount < kMaxHalfPeriods) {
            mHalfPeriods[mCount] = elapsedMs - mLastSwitchMs;
            mPeaks[mCount] = mPeak;
            mCount++;
        }
        mLastSwitchMs = elapsedMs;
        mPeak = 0.0f;
        const int want = mConfig.cycles * 2;
        if (mCount >= want || mCount >= kMaxHalfPeriods) {
            computeFit();
            return 0.0f;
        }
    }
    return mSign * mConfig.amplitude;
}

void RelayIdentifier::finish() {
    if (mOutcome == RelayOutcome::Cycling) computeFit();
}

void RelayIdentifier::computeFit() {
    if (mCount < 2) {
        mOutcome = RelayOutcome::NoCycle;
        return;
    }
    const float halfMs = mean(mHalfPeriods, mCount);
    const float amplitude = mean(mPeaks, mCount);

    // A period resting on its own floor measures the floor.
    if (halfMs < mConfig.minHalfPeriodMs * mConfig.dwellSlack) {
        mOutcome = RelayOutcome::DwellFloor;
        return;
    }
    const float tu = 2.0f * halfMs / 1000.0f;
    if (!(tu > 0.0f) ||
        amplitude < mConfig.minSwingRatio * mConfig.hysteresis) {
        mOutcome = RelayOutcome::Chatter;
        mFit.amplitude = amplitude;
        mFit.tu = tu;
        return;
    }
    const float ku = 4.0f * mConfig.amplitude / (kPi * amplitude);
    mFit = relayGains(ku, tu, amplitude, mCount / 2, mConfig);
    mOutcome = RelayOutcome::Fitted;
}

RelayFit relayGains(float ku, float tu, float amplitude, int cycles,
                    const RelayIdentifyConfig& config) {
    RelayFit fit;
    fit.ku = ku;
    fit.tu = tu;
    fit.amplitude = amplitude;
    fit.cycles = cycles;
    fit.kP = config.kpFactor * ku;
    fit.kD = fit.kP * tu / config.tdDivisor;
    fit.kI = config.includeIntegral ? fit.kP / (config.tiFactor * tu) : 0.0f;
    fit.valid = true;
    return fit;
}

}  // namespace chad::tuning
