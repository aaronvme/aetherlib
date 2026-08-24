#pragma once

namespace chad::tuning {

/// Least-squares fit of volts = kS + kV*speed + kA*accel, in whatever speed
/// units were fed in. Standard errors come with it because on a single
/// constant-slope ramp the accel column converges to a constant and goes
/// collinear with the intercept: kA is then unidentifiable, and the fit reports
/// a confident-looking number for it regardless.
struct FeedforwardFitStats {
    float kS{0.0f};
    float kV{0.0f};
    float kA{0.0f};
    float r2{0.0f};
    float seKS{0.0f};
    float seKV{0.0f};
    float seKA{0.0f};
    /// |kA| / seKA, infinite on an exact fit. Below ~2 the noise swamps kA.
    float kaT{0.0f};
    int samples{0};
    bool solved{false};
};

/// Accumulates the normal equations as samples arrive, so the fit needs no
/// sample storage and no heap.
class FeedforwardAccumulator {
public:
    void reset();
    void add(float speed, float accel, float volts);
    int samples() const { return mN; }
    bool solve(FeedforwardFitStats& out) const;

    static constexpr int kMinSamples = 16;

private:
    double mXtX[3][3]{};
    double mXty[3]{};
    double mYty{0.0};
    double mSy{0.0};
    int mN{0};
};

/// Emits the span difference of speed against the span MEAN of speed and
/// voltage. Integrating the plant over the window and dividing by its length
/// gives kS + kV*mean(v) + kA*dv/dt exactly, so the mean is the pairing the
/// model asks for -- and it divides the velocity noise by root(span) instead of
/// carrying one sample of it. Pairing with the newest sample instead misdates
/// every row by half a span, which biases kA, the only term whose information
/// lives in the transient.
class RampSampler {
public:
    static constexpr int kMaxSpan = 64;

    /// `span` is the number of ticks the difference spans.
    RampSampler(int span, float dtSec);

    void reset();

    struct Sample {
        float speed;
        float accel;
        float volts;
    };

    /// True once `span` ticks have been seen; `out` is then the centred sample.
    bool push(float speed, float volts, Sample& out);

private:
    float mSpeed[kMaxSpan + 1]{};
    float mVolts[kMaxSpan + 1]{};
    int mSpan;
    float mDt;
    int mCount{0};
};

}  // namespace chad::tuning
