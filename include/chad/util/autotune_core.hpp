#pragma once

#include "chad/util/pid.hpp"
#include "chad/util/step_response.hpp"

namespace chad::tuning {
struct GainSearchConfig {
    float kpGrowth{1.5f};
    float overshootLimitPct{20.0f};
    int oscillationLimit{3};

    float kdSeedRatio{0.075f};

    float kdRatioStep{0.05f};

    float kiRatioStep{0.5f};
    bool includeIntegral{false};
    float steadyStateTol{1.5f};

    int maxIterations{6};
};

enum class GainPhase { Kp, Kd, Ki, Done };

struct TrialRequest {
    PIDGains gains;
    GainPhase phase{GainPhase::Done};
};

class MetricsAverage {
public:
    void reset() {
        mAcc = StepMetrics{};
        mCount = 0;
        mFirstRejectInitial = 0.0f;
        mHaveReject = false;
    }

    void add(const StepMetrics& m) {
        if (!m.valid) {
            if (!mHaveReject) {
                mFirstRejectInitial = m.initial;
                mHaveReject = true;
            }
            return;
        }
        mAcc.overshootPct += m.overshootPct;
        mAcc.steadyState += m.steadyState;
        mAcc.settleMs += m.settleMs;
        mAcc.oscillations += m.oscillations;
        mAcc.cost += m.cost;
        mAcc.initial += m.initial;
        mCount++;
    }

    StepMetrics result() const {
        StepMetrics out;
        if (mCount == 0) {
            out.initial = mFirstRejectInitial;
            return out;
        }
        const float inv = 1.0f / static_cast<float>(mCount);
        out.overshootPct = mAcc.overshootPct * inv;
        out.steadyState = mAcc.steadyState * inv;
        out.settleMs = mAcc.settleMs * inv;
        out.oscillations =
            static_cast<int>(static_cast<float>(mAcc.oscillations) * inv + 0.5f);
        out.cost = mAcc.cost * inv;
        out.initial = mAcc.initial * inv;
        out.valid = true;
        return out;
    }

    int accepted() const { return mCount; }

private:
    StepMetrics mAcc{};
    int mCount{0};
    float mFirstRejectInitial{0.0f};
    bool mHaveReject{false};
};

class GainSearch {
public:
    void reset(const GainSearchConfig& cfg, PIDGains start) {
        mCfg = cfg;
        mBest = start;
        mBest.kI = 0.0f;
        mBest.kD = 0.0f;
        mBestMetrics = StepMetrics{};
        mBestCost = 1e30f;
        mPhase = GainPhase::Done;
        mIter = 0;
        mTrials = 0;
        mKpFloor = start.kP;
    }

    void beginStep() {
        mPhase = GainPhase::Kp;
        mIter = 0;
        mBestCost = 1e30f;
        mBestMetrics = StepMetrics{};
        mKpFloor = mBest.kP;
        mBestKp = mBest.kP;
        mCandidateKp = mBest.kP;
        mBest.kD = 0.0f;
        mBest.kI = 0.0f;
    }

    bool stepDone() const { return mPhase == GainPhase::Done; }

    TrialRequest request() const {
        TrialRequest t;
        t.phase = mPhase;
        switch (mPhase) {
            case GainPhase::Kp:
                t.gains = {mCandidateKp, 0.0f, mCandidateKp * mCfg.kdSeedRatio};
                break;
            case GainPhase::Kd:
                t.gains = {mBestKp, 0.0f,
                           mBestKp * (mCfg.kdSeedRatio +
                                      mCfg.kdRatioStep * static_cast<float>(mIter))};
                break;
            case GainPhase::Ki:
                t.gains = {mBest.kP,
                           mBest.kP * mCfg.kiRatioStep * static_cast<float>(mIter),
                           mBest.kD};
                break;
            case GainPhase::Done:
                t.gains = mBest;
                break;
        }
        return t;
    }

    void submit(const StepMetrics& m) {
        const TrialRequest t = request();
        mTrials++;

        if (!m.valid) {
            advancePhase();
            return;
        }

        switch (mPhase) {
            case GainPhase::Kp: {
                if (m.cost < mBestCost) {
                    mBestCost = m.cost;
                    mBestKp = mCandidateKp;
                    mBestMetrics = m;
                }
                const bool stop = m.overshootPct > mCfg.overshootLimitPct ||
                                  m.oscillations >= mCfg.oscillationLimit ||
                                  mIter + 1 >= mCfg.maxIterations;
                if (stop) {
                    if (mBestKp < mKpFloor) mBestKp = mKpFloor;
                    mBest.kP = mBestKp;
                    mBest.kD = mBestKp * mCfg.kdSeedRatio;
                    mPhase = GainPhase::Kd;
                    mIter = 1;
                } else {
                    mCandidateKp *= mCfg.kpGrowth;
                    mIter++;
                }
                break;
            }
            case GainPhase::Kd: {
                if (m.cost < mBestCost) {
                    mBestCost = m.cost;
                    mBest.kD = t.gains.kD;
                    mBestMetrics = m;
                    if (mIter >= mCfg.maxIterations) {
                        enterKiOrFinish();
                    } else {
                        mIter++;
                    }
                } else {
                    enterKiOrFinish();
                }
                break;
            }
            case GainPhase::Ki: {
                if (m.cost < mBestCost) {
                    mBestCost = m.cost;
                    mBest.kI = t.gains.kI;
                    mBestMetrics = m;
                }
                if (m.steadyState <= mCfg.steadyStateTol ||
                    mIter >= mCfg.maxIterations) {
                    mPhase = GainPhase::Done;
                } else {
                    mIter++;
                }
                break;
            }
            case GainPhase::Done:
                break;
        }
    }

    PIDGains best() const { return mBest; }

    const StepMetrics& bestMetrics() const { return mBestMetrics; }

    GainPhase phase() const { return mPhase; }
    int trials() const { return mTrials; }

private:
    void enterKiOrFinish() {
        if (mCfg.includeIntegral && mBestMetrics.valid &&
            mBestMetrics.steadyState > mCfg.steadyStateTol) {
            mPhase = GainPhase::Ki;
            mIter = 1;
        } else {
            mPhase = GainPhase::Done;
        }
    }

    void advancePhase() {
        switch (mPhase) {
            case GainPhase::Kp:
                if (mBestKp < mKpFloor) mBestKp = mKpFloor;
                mBest.kP = mBestKp;
                mBest.kD = mBestKp * mCfg.kdSeedRatio;
                mPhase = GainPhase::Kd;
                mIter = 1;
                break;
            case GainPhase::Kd:
                enterKiOrFinish();
                break;
            default:
                mPhase = GainPhase::Done;
                break;
        }
    }

    GainSearchConfig mCfg{};
    PIDGains mBest{};
    StepMetrics mBestMetrics{};
    float mBestCost{1e30f};
    float mCandidateKp{0.0f};
    float mBestKp{0.0f};
    float mKpFloor{0.0f};
    GainPhase mPhase{GainPhase::Done};
    int mIter{0};
    int mTrials{0};
};
}
