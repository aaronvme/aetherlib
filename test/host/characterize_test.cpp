#include "chad/util/characterize_core.hpp"

#include <cmath>
#include <cstdio>

#include "chad_test.hpp"

using chad::tuning::FeedforwardAccumulator;
using chad::tuning::FeedforwardFitStats;
using chad::tuning::RampSampler;

namespace {

// volts = kS + kV*v + kA*a, inches. kV from a 60 in/s free speed at 12 V and
// kA from a 0.15 s mechanical time constant; both land in the middle of what a
// V5 drivetrain measures.
struct Plant {
    static constexpr double kS = 0.6;
    static constexpr double kV = 0.19;
    static constexpr double kA = 0.0285;

    double v = 0.0;

    // Sub-steps, so the plant is the reference rather than the integrator.
    void step(double volts, double dt) {
        constexpr int kSub = 20;
        const double h = dt / kSub;
        for (int i = 0; i < kSub; i++) {
            const double drive = volts - kS - kV * v;
            if (v <= 0.0 && drive <= 0.0) { v = 0.0; continue; }
            v += (drive / kA) * h;
            if (v < 0.0) v = 0.0;
        }
    }
};

enum class Pairing { Newest, Midpoint, Mean, Sampler };

struct RunOpts {
    double rateA = 2.0;      // V/s below legVolts
    double rateB = 2.0;      // V/s above it
    double legVolts = 4.5;
    double maxVolts = 9.0;
    double minSpeed = 3.0;
    int span = 5;
    Pairing pairing = Pairing::Mean;
    double quantIn = 0.0;    // velocity quantisation, in/s
};

FeedforwardFitStats runRamp(const RunOpts& o) {
    Plant plant;
    FeedforwardAccumulator acc;
    RampSampler sampler(o.span, 0.01f);
    const double dt = 0.01;
    const int span = o.span;

    double hv[RampSampler::kMaxSpan + 1] = {};
    double hu[RampSampler::kMaxSpan + 1] = {};
    int held = 0;

    double volts = 0.0;
    unsigned seed = 12345u;
    while (volts <= o.maxVolts) {
        plant.step(volts, dt);
        double speed = plant.v;
        if (o.quantIn > 0.0) {
            seed = seed * 1664525u + 1013904223u;
            const double u = static_cast<double>(seed >> 8) / 16777216.0 - 0.5;
            speed = std::round(speed / o.quantIn + u) * o.quantIn;
        }

        if (o.pairing == Pairing::Sampler) {
            RampSampler::Sample s;
            if (sampler.push(static_cast<float>(speed),
                             static_cast<float>(volts), s) &&
                s.speed > o.minSpeed) {
                acc.add(s.speed, s.accel, s.volts);
            }
        } else {
            if (held == span + 1) {
                for (int i = 0; i < span; i++) { hv[i] = hv[i + 1]; hu[i] = hu[i + 1]; }
                hv[span] = speed;
                hu[span] = volts;
            } else {
                hv[held] = speed;
                hu[held] = volts;
                held++;
            }
            if (held == span + 1) {
                const double accel = (hv[span] - hv[0]) / (dt * span);
                double sv = 0.0, su = 0.0;
                if (o.pairing == Pairing::Newest) {
                    sv = hv[span];
                    su = hu[span];
                } else if (o.pairing == Pairing::Midpoint) {
                    sv = hv[span / 2];
                    su = hu[span / 2];
                } else {
                    sv = 0.5 * (hv[0] + hv[span]);
                    su = 0.5 * (hu[0] + hu[span]);
                    for (int i = 1; i < span; i++) { sv += hv[i]; su += hu[i]; }
                    sv /= span;
                    su /= span;
                }
                if (sv > o.minSpeed) {
                    acc.add(static_cast<float>(sv), static_cast<float>(accel),
                            static_cast<float>(su));
                }
            }
        }
        volts += (volts < o.legVolts ? o.rateA : o.rateB) * dt;
    }

    FeedforwardFitStats fit;
    acc.solve(fit);
    return fit;
}

double kaErrPct(const FeedforwardFitStats& f) {
    return 100.0 * (f.kA - Plant::kA) / Plant::kA;
}

void show(const char* label, const FeedforwardFitStats& f) {
    std::printf(
        "    %-24s kS %+6.1f%%  kV %+6.1f%%  kA %+8.1f%%  r2 %8.5f  "
        "t(kA) %9.2f  n %d\n",
        label, 100.0 * (f.kS - Plant::kS) / Plant::kS,
        100.0 * (f.kV - Plant::kV) / Plant::kV, kaErrPct(f), f.r2, f.kaT,
        f.samples);
}

}  // namespace

CHAD_TEST(characterize, sweepPairingSpanAndExcitation) {
    for (double q : {0.0, 0.3, 0.6}) {
        std::printf("  velocity quantisation %.2f in/s:\n", q);
        for (Pairing p : {Pairing::Newest, Pairing::Midpoint, Pairing::Mean}) {
            RunOpts o;
            o.quantIn = q;
            o.pairing = p;
            char label[48];
            std::snprintf(label, sizeof label, "1 rate, span 5, %s",
                          p == Pairing::Newest ? "newest"
                          : p == Pairing::Midpoint ? "midpoint" : "mean");
            show(label, runRamp(o));
        }
        for (int span : {5, 20, 40, 64}) {
            RunOpts o;
            o.quantIn = q;
            o.span = span;
            char label[48];
            std::snprintf(label, sizeof label, "1 rate, mean, span %d", span);
            show(label, runRamp(o));
            o.rateA = 1.0;
            o.rateB = 6.0;
            std::snprintf(label, sizeof label, "1/6 V/s, mean, span %d", span);
            show(label, runRamp(o));
        }
    }
}

// The plant is noiseless here, so any kA error is the pairing and nothing else.
CHAD_TEST(characterize, windowMeanPairingRemovesTheHalfSpanBias) {
    RunOpts o;
    o.span = 5;
    o.pairing = Pairing::Newest;
    const double newest = kaErrPct(runRamp(o));
    o.pairing = Pairing::Mean;
    const double mean = kaErrPct(runRamp(o));
    // A span-5 backward difference is dated 25 ms late against the sample it
    // was paired with, and on this plant that is a 16% error in kA.
    CHECK(std::fabs(newest) > 10.0);
    CHECK(std::fabs(mean) < 1.0);
}

CHAD_TEST(characterize, samplerMatchesTheMeanPairing) {
    for (int span : {1, 2, 5, 20}) {
        RunOpts a;
        a.span = span;
        a.quantIn = 0.3;
        a.pairing = Pairing::Mean;
        RunOpts b = a;
        b.pairing = Pairing::Sampler;
        const FeedforwardFitStats fa = runRamp(a);
        const FeedforwardFitStats fb = runRamp(b);
        CHECK(fa.samples == fb.samples);
        CHECK_NEAR(fa.kV, fb.kV, 1e-6);
        CHECK_NEAR(fa.kA, fb.kA, 1e-7);
    }
}

CHAD_TEST(characterize, oneSlopeCannotIdentifyKaUnderQuantisation) {
    RunOpts one;
    one.quantIn = 0.6;
    one.span = 40;
    const FeedforwardFitStats f = runRamp(one);
    // kA collapses to nothing while kV survives, because kS and kV can carry
    // the whole ramp voltage between them.
    CHECK(std::fabs(kaErrPct(f)) > 50.0);
    CHECK(std::fabs(100.0 * (f.kV - Plant::kV) / Plant::kV) < 2.0);
    // The shipped gate rejects it.
    CHECK(f.kaT < 25.0f);
    // And r2 does not: this is why the gate cannot be r2.
    CHECK(f.r2 > 0.999f);
}

CHAD_TEST(characterize, twoSlopesIdentifyKa) {
    for (double q : {0.3, 0.6, 1.0}) {
        RunOpts two;
        two.quantIn = q;
        two.span = 40;
        two.rateA = 1.0;
        two.rateB = 6.0;
        const FeedforwardFitStats f = runRamp(two);
        CHECK(f.kaT >= 25.0f);
        CHECK(std::fabs(kaErrPct(f)) < 15.0);
        CHECK(std::fabs(100.0 * (f.kS - Plant::kS) / Plant::kS) < 5.0);
        CHECK(std::fabs(100.0 * (f.kV - Plant::kV) / Plant::kV) < 2.0);
    }
}

// Slow-then-fast, not fast-then-slow: the long low-speed leg is what pins kS.
CHAD_TEST(characterize, slowFirstBeatsFastFirstOnKs) {
    RunOpts slow;
    slow.quantIn = 0.6;
    slow.span = 20;
    slow.rateA = 1.0;
    slow.rateB = 6.0;
    RunOpts fast = slow;
    fast.rateA = 6.0;
    fast.rateB = 1.0;
    const double slowErr =
        std::fabs(100.0 * (runRamp(slow).kS - Plant::kS) / Plant::kS);
    const double fastErr =
        std::fabs(100.0 * (runRamp(fast).kS - Plant::kS) / Plant::kS);
    CHECK(slowErr < 5.0);
    CHECK(fastErr > 10.0);
}

CHAD_TEST(characterize, anExactFitReportsKaAsFullyIdentified) {
    // Zero residual leaves no scale for a standard error, which is kA at its
    // most identified, not its least. Reporting 0 there would read as unfit.
    RunOpts clean;
    clean.span = 20;
    const FeedforwardFitStats f = runRamp(clean);
    CHECK(f.solved);
    CHECK(std::isinf(f.kaT));
    CHECK(f.kaT > 25.0f);
}

CHAD_TEST(characterize, aCollinearDesignIsRefusedRatherThanFitted) {
    FeedforwardAccumulator acc;
    // Constant acceleration makes the accel column a multiple of the intercept,
    // so kS and kA are not separately identifiable at any sample count.
    for (int i = 0; i < 200; i++) {
        const float v = 5.0f + 0.1f * static_cast<float>(i);
        acc.add(v, 4.0f, 0.6f + 0.19f * v + 0.0285f * 4.0f);
    }
    FeedforwardFitStats f;
    CHECK(!acc.solve(f));
    CHECK(!f.solved);
    CHECK(f.samples == 200);
}

CHAD_TEST(characterize, tooFewSamplesIsRefused) {
    FeedforwardAccumulator acc;
    for (int i = 0; i < FeedforwardAccumulator::kMinSamples - 1; i++) {
        acc.add(10.0f + static_cast<float>(i), 3.0f - 0.1f * static_cast<float>(i),
                7.0f);
    }
    FeedforwardFitStats f;
    CHECK(!acc.solve(f));
    acc.add(30.0f, 0.5f, 8.0f);
    CHECK(acc.samples() == FeedforwardAccumulator::kMinSamples);
    CHECK(acc.solve(f));
}

CHAD_TEST(characterize, resetClearsTheAccumulator) {
    FeedforwardAccumulator acc;
    for (int i = 0; i < 40; i++) {
        const float v = 5.0f + static_cast<float>(i);
        const float a = (i % 3 == 0) ? 4.0f : (i % 3 == 1) ? 1.0f : -2.0f;
        acc.add(v, a, 0.6f + 0.19f * v + 0.0285f * a);
    }
    FeedforwardFitStats before;
    CHECK(acc.solve(before));
    acc.reset();
    CHECK(acc.samples() == 0);
    FeedforwardFitStats after;
    CHECK(!acc.solve(after));
}
