#pragma once

#include <cmath>

namespace chad::tuning {

// The maths behind Characterizer::feedforward(), separated from PROS so a whole
// fit can run on the host against a plant model.
//
// A voltage ramp cannot identify kA. Solving kS + kV*v + kA*a = c*t for a ramp
// rate c gives a = c/kV once the transient has passed, so the acceleration
// column is constant and its only variation is the noise in a differenced
// encoder velocity -- which attenuates kA toward zero and dumps kA*c/kV into
// kS. Against a first-order plant the joint ramp fit returned kA with the wrong
// sign in 100 of 100 trials and kS 32% high, at r2 = 0.997. Hence a quasistatic
// staircase for kS and kV, where a is zero by construction, and separate steps
// for kA, where a is large and cleanly measurable.

constexpr int MAX_STEP_REPS = 16;

/// One dwell held long enough to reach steady state: a-priori a = 0.
struct VelocityPoint {
    float speed{0.0f};   // in/s, mean over the hold window
    float volts{0.0f};
};

/// One step test: the slope of speed against time, and the speed it happened at.
struct StepSample {
    float speed{0.0f};   // in/s, mean over the slope window
    float accel{0.0f};   // in/s^2
    float volts{0.0f};
};

struct VelocityFit {
    float kS{0.0f};        // volts
    float kV{0.0f};        // volts per in/s
    float r2{0.0f};
    float speedSpan{0.0f}; // in/s covered; a short span extrapolates kS badly
    int levels{0};
    bool valid{false};
};

struct AccelFit {
    float kA{0.0f};      // volts per in/s^2, median over reps
    float spread{0.0f};  // (max - min) / |kA|
    int reps{0};
    bool valid{false};
};

/// Trapezoidal mean, which is the average the dwell integral implies.
inline float meanOverWindow(const float* speed, int n) {
    if (speed == nullptr || n < 2) return (speed != nullptr && n == 1) ? speed[0] : 0.0f;
    double sum = 0.5 * (static_cast<double>(speed[0]) + speed[n - 1]);
    for (int i = 1; i < n - 1; i++) sum += speed[i];
    return static_cast<float>(sum / static_cast<double>(n - 1));
}

/// Least squares slope over a uniform time grid. A least squares slope beats an
/// endpoint difference by sqrt(n/5) or so on noise, which is what makes kA
/// measurable at all through 2.8 in/s of velocity quantisation.
inline float slopeOverWindow(const float* speed, int n, float dt) {
    if (speed == nullptr || n < 3 || dt <= 0.0f) return 0.0f;
    const double nd = static_cast<double>(n);
    double sy = 0.0, sty = 0.0;
    for (int i = 0; i < n; i++) {
        sy += speed[i];
        sty += static_cast<double>(i) * speed[i];
    }
    const double st = 0.5 * nd * (nd - 1.0);
    const double stt = (nd - 1.0) * nd * (2.0 * nd - 1.0) / 6.0;
    const double den = nd * stt - st * st;
    if (den <= 0.0) return 0.0f;
    return static_cast<float>((nd * sty - st * sy) / den / dt);
}

/// volts = kS + kV * speed across the staircase.
inline VelocityFit fitVelocity(const VelocityPoint* pts, int n, float minR2) {
    VelocityFit fit;
    if (pts == nullptr || n < 4) return fit;

    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    float lo = pts[0].speed, hi = pts[0].speed;
    for (int i = 0; i < n; i++) {
        const double x = pts[i].speed, y = pts[i].volts;
        sx += x; sy += y; sxx += x * x; sxy += x * y;
        if (pts[i].speed < lo) lo = pts[i].speed;
        if (pts[i].speed > hi) hi = pts[i].speed;
    }
    const double nd = static_cast<double>(n);
    const double den = nd * sxx - sx * sx;
    if (std::fabs(den) < 1e-9) return fit;

    const double kv = (nd * sxy - sx * sy) / den;
    const double ks = (sy - kv * sx) / nd;

    double ssRes = 0.0, ssTot = 0.0;
    const double mean = sy / nd;
    for (int i = 0; i < n; i++) {
        const double pred = ks + kv * pts[i].speed;
        const double d = pts[i].volts - pred;
        ssRes += d * d;
        ssTot += (pts[i].volts - mean) * (pts[i].volts - mean);
    }

    fit.kS = static_cast<float>(ks);
    fit.kV = static_cast<float>(kv);
    fit.r2 = ssTot > 1e-9 ? static_cast<float>(1.0 - ssRes / ssTot) : 0.0f;
    fit.speedSpan = hi - lo;
    fit.levels = n;
    fit.valid = fit.kV > 0.0f && fit.r2 >= minR2;
    return fit;
}

/// kA per rep, then the median. Every rep is an independent estimate, so a rep
/// that slipped or never broke away moves the median one place instead of
/// dragging a mean.
inline AccelFit fitAccel(const StepSample* s, int n, float kS, float kV,
                         float maxSpread) {
    AccelFit fit;
    if (s == nullptr || n < 2) return fit;

    float per[MAX_STEP_REPS];
    int m = 0;
    for (int i = 0; i < n && m < MAX_STEP_REPS; i++) {
        if (s[i].accel <= 0.0f) continue;   // never moved, or already at speed
        const float excess = s[i].volts - kS - kV * s[i].speed;
        if (excess <= 0.0f) continue;       // model says it should not accelerate
        per[m++] = excess / s[i].accel;
    }
    if (m < 2) return fit;

    for (int i = 1; i < m; i++) {           // insertion sort, m <= 16
        const float x = per[i];
        int j = i - 1;
        while (j >= 0 && per[j] > x) { per[j + 1] = per[j]; j--; }
        per[j + 1] = x;
    }
    fit.kA = (m % 2 == 1) ? per[m / 2] : 0.5f * (per[m / 2 - 1] + per[m / 2]);
    fit.reps = m;
    if (fit.kA > 0.0f) fit.spread = (per[m - 1] - per[0]) / fit.kA;
    fit.valid = fit.kA > 0.0f && fit.spread <= maxSpread;
    return fit;
}

/// Fitted no-load speed over what the gear ratio and wheel say it should be.
///
/// This is the only check that catches a drivetrain which is merely weaker than
/// it should be: a dead motor, a wrong externalGearRatio or a wrong
/// wheelDiameter all leave a perfectly straight line, so r2 stays near 0.98 and
/// nothing inside the fit objects. Measured 0.66-1.00 across healthy plants and
/// 0.39-0.40 with half the motors pulling or the wheel size doubled.
inline float freeSpeedRatio(float kS, float kV, float ratedInPerSec,
                            float railVolts = 12.0f) {
    if (kV <= 0.0f || ratedInPerSec <= 0.0f) return 0.0f;
    return (railVolts - kS) / kV / ratedInPerSec;
}

}  // namespace chad::tuning
