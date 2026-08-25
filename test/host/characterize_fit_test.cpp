#include "chad/util/characterize_fit.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include "chad_test.hpp"

using namespace chad::tuning;

namespace {

constexpr float kDt = 0.010f;
constexpr float kMinSpeed = 3.0f;

// Truth for the plant every fit here is run against.
constexpr float kS = 0.85f;
constexpr float kV = 0.185f;
constexpr float kA = 0.0220f;

// One rpm of a 200 rpm cartridge on a 3.25 in wheel through a 12:1 -- the step
// the velocity reading actually moves in, and the reason a differenced velocity
// is nearly all noise.
constexpr float kQuant = 200.0f / 60.0f * 3.14159265f * 3.25f / 12.0f;

// Deterministic, so a failure is reproducible. Marsaglia xorshift plus a
// Box-Muller pair for the gaussian.
struct Rng {
    std::uint32_t s;
    explicit Rng(std::uint32_t seed) : s(seed ? seed : 1u) {}
    float uniform() {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        return static_cast<float>(s % 1000000u) / 1000000.0f;
    }
    float gauss(float sigma) {
        const float u1 = 0.000001f + uniform() * 0.999998f;
        const float u2 = uniform();
        return sigma * std::sqrt(-2.0f * std::log(u1)) *
               std::cos(6.2831853f * u2);
    }
};

struct Plant {
    float ks{kS}, kv{kV}, ka{kA}, noise{0.25f};
    float speed{0.0f};

    void advance(float volts) {
        const float drive = volts - ks > 0.0f ? volts - ks : 0.0f;
        if (speed <= 0.0f && drive <= 0.0f) { speed = 0.0f; return; }
        speed += (drive - kv * speed) / ka * kDt;
        if (speed < 0.0f) speed = 0.0f;
    }
    float measure(Rng& rng) const {
        return std::round(speed / kQuant) * kQuant + rng.gauss(noise);
    }
};

// The staircase: dwell, then average the tail.
std::vector<VelocityPoint> staircase(Plant& p, Rng& rng, int levels, float vLo,
                                     float vHi, int dwellN, int holdN) {
    std::vector<VelocityPoint> out;
    for (int i = 0; i < levels; i++) {
        const float f = levels > 1 ? static_cast<float>(i) / (levels - 1) : 0.0f;
        const float v = vLo + f * (vHi - vLo);
        std::vector<float> hold;
        p.speed = 0.0f;
        for (int k = 0; k < dwellN; k++) {
            p.advance(v);
            if (k >= dwellN - holdN) hold.push_back(p.measure(rng));
        }
        const float mean = meanOverWindow(hold.data(), static_cast<int>(hold.size()));
        if (mean > kMinSpeed) out.push_back({mean, v});
    }
    return out;
}

std::vector<StepSample> stepTests(Plant& p, Rng& rng, int reps, float volts,
                                  int stepN, int slopeN) {
    std::vector<StepSample> out;
    for (int r = 0; r < reps; r++) {
        std::vector<float> trace;
        p.speed = 0.0f;
        for (int k = 0; k < stepN; k++) {
            p.advance(volts);
            trace.push_back(p.measure(rng));
        }
        int first = -1;
        for (int k = 0; k < static_cast<int>(trace.size()); k++) {
            if (trace[k] > kMinSpeed) { first = k; break; }
        }
        if (first < 0 || first + slopeN > static_cast<int>(trace.size())) continue;
        const float* w = trace.data() + first;
        out.push_back({meanOverWindow(w, slopeN), slopeOverWindow(w, slopeN, kDt),
                       volts});
    }
    return out;
}

}  // namespace

CHAD_TEST(characterizeFit, slopeOverWindowRecoversAKnownRamp) {
    float s[40];
    for (int i = 0; i < 40; i++) s[i] = 5.0f + 120.0f * (i * kDt);
    CHECK_NEAR(slopeOverWindow(s, 40, kDt), 120.0, 1e-2);
    // Degenerate inputs return 0 rather than dividing by zero.
    CHECK(slopeOverWindow(s, 2, kDt) == 0.0f);
    CHECK(slopeOverWindow(s, 40, 0.0f) == 0.0f);
    CHECK(slopeOverWindow(nullptr, 40, kDt) == 0.0f);
}

CHAD_TEST(characterizeFit, slopeBeatsAnEndpointDifferenceOnNoise) {
    // Same ramp, same noise, both estimators. This margin is the whole reason
    // kA is measurable through velocity quantisation.
    Rng rng(7u);
    double slopeErr = 0.0, endErr = 0.0;
    constexpr int kN = 20;
    for (int trial = 0; trial < 400; trial++) {
        float s[kN];
        for (int i = 0; i < kN; i++) {
            s[i] = 5.0f + 200.0f * (i * kDt) + rng.gauss(0.9f);
        }
        const float bySlope = slopeOverWindow(s, kN, kDt);
        const float byEnds = (s[kN - 1] - s[0]) / ((kN - 1) * kDt);
        slopeErr += (bySlope - 200.0) * (bySlope - 200.0);
        endErr += (byEnds - 200.0) * (byEnds - 200.0);
    }
    CHECK(slopeErr < endErr);
    // var(endpoint) / var(slope) = [2/(n-1)^2] / [12/(n(n^2-1))] = 3.68 at
    // n = 20, so the sd ratio is 1.92. Bracket it rather than asserting a
    // bound, so a change to either estimator shows up.
    const double ratio = std::sqrt(endErr / slopeErr);
    CHECK(ratio > 1.7);
    CHECK(ratio < 2.2);
}

CHAD_TEST(characterizeFit, meanOverWindowIsTheTrapezoid) {
    const float s[] = {0.0f, 10.0f, 20.0f, 30.0f};
    // Trapezoidal mean of a straight line is the midpoint, 15, not 15 either way
    // here -- use an asymmetric case to tell them apart.
    CHECK_NEAR(meanOverWindow(s, 4), 15.0, 1e-4);
    const float t[] = {0.0f, 0.0f, 0.0f, 60.0f};
    CHECK_NEAR(meanOverWindow(t, 4), 10.0, 1e-4);   // (0/2 + 0 + 0 + 30)/3
    CHECK(meanOverWindow(nullptr, 4) == 0.0f);
}

CHAD_TEST(characterizeFit, staircaseRecoversKsAndKv) {
    Plant p;
    Rng rng(11u);
    const auto pts = staircase(p, rng, 11, 1.1f, 9.0f, 90, 40);
    const VelocityFit f = fitVelocity(pts.data(), static_cast<int>(pts.size()), 0.9f);
    CHECK(f.valid);
    CHECK(f.levels >= 8);
    CHECK_NEAR(f.kV, kV, 0.02);      // within ~10%
    CHECK_NEAR(f.kS, kS, 0.15);      // within ~18%
    CHECK(f.r2 > 0.99f);
    CHECK(f.speedSpan > 30.0f);
}

CHAD_TEST(characterizeFit, stepsRecoverKa) {
    Plant p;
    Rng rng(23u);
    const auto pts = staircase(p, rng, 11, 1.1f, 9.0f, 90, 40);
    const VelocityFit v = fitVelocity(pts.data(), static_cast<int>(pts.size()), 0.9f);
    const auto st = stepTests(p, rng, 6, 7.0f, 55, 20);
    const AccelFit a = fitAccel(st.data(), static_cast<int>(st.size()), v.kS, v.kV,
                                0.5f);
    CHECK(a.valid);
    CHECK(a.reps >= 4);
    CHECK(a.kA > 0.0f);
    CHECK_NEAR(a.kA, kA, 0.006);     // within ~25%
    CHECK(a.spread < 0.5f);
}

// The regression this file exists for. Fitting kS, kV and kA jointly off a
// single voltage ramp gives kA the WRONG SIGN, because a ramp holds
// acceleration constant at rampRate/kV and the only variation left in that
// column is differencing noise.
CHAD_TEST(characterizeFit, aSingleRampCannotIdentifyKa) {
    Plant p;
    Rng rng(31u);
    constexpr float kRampVoltsPerSec = 2.0f;

    // Reproduce the old sampler: span-5 endpoint difference, endpoint pairing.
    std::vector<float> hist;
    double A[3][4] = {};
    int n = 0;
    p.speed = 0.0f;
    for (float v = 0.0f; v <= 9.0f; v += kRampVoltsPerSec * kDt) {
        p.advance(v);
        const float sp = p.measure(rng);
        hist.push_back(sp);
        if (static_cast<int>(hist.size()) > 5 && sp > kMinSpeed) {
            const float accel = (sp - hist[hist.size() - 6]) / (kDt * 5);
            const double x[3] = {1.0, sp, accel};
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) A[i][j] += x[i] * x[j];
                A[i][3] += x[i] * v;
            }
            n++;
        }
    }
    CHECK(n > 200);
    for (int c = 0; c < 3; c++) {
        int piv = c;
        for (int r = c + 1; r < 3; r++) {
            if (std::fabs(A[r][c]) > std::fabs(A[piv][c])) piv = r;
        }
        for (int k = 0; k < 4; k++) { const double t = A[c][k]; A[c][k] = A[piv][k]; A[piv][k] = t; }
        const double d = A[c][c];
        for (int k = 0; k < 4; k++) A[c][k] /= d;
        for (int r = 0; r < 3; r++) {
            if (r == c) continue;
            const double f = A[r][c];
            for (int k = 0; k < 4; k++) A[r][k] -= f * A[c][k];
        }
    }
    const double rampKs = A[0][3], rampKv = A[1][3], rampKa = A[2][3];

    // kV survives -- it is the one term the ramp does excite.
    CHECK(std::fabs(rampKv - kV) < 0.02);
    // kA does not: wrong sign, and kS absorbs the kA*accel it dropped.
    CHECK(rampKa < 0.0);
    CHECK(rampKs > kS * 1.15);

    // The shipped design gets the same plant right.
    Rng r2(31u);
    const auto pts = staircase(p, r2, 11, 1.1f, 9.0f, 90, 40);
    const VelocityFit vf = fitVelocity(pts.data(), static_cast<int>(pts.size()), 0.9f);
    const auto st = stepTests(p, r2, 6, 7.0f, 55, 20);
    const AccelFit af = fitAccel(st.data(), static_cast<int>(st.size()), vf.kS, vf.kV, 0.5f);
    CHECK(af.kA > 0.0f);
    CHECK(std::fabs(af.kA - kA) < std::fabs(static_cast<float>(rampKa) - kA));
    CHECK(std::fabs(vf.kS - kS) < std::fabs(static_cast<float>(rampKs) - kS));
}

CHAD_TEST(characterizeFit, freeSpeedRatioCatchesAWeakDrivetrain) {
    const float rated = 450.0f / 60.0f * 3.14159265f * 3.25f;   // 76.6 in/s
    // Healthy: fitted no-load speed lands inside the band.
    const float healthy = freeSpeedRatio(kS, kV, rated);
    CHECK(healthy > 0.55f);
    CHECK(healthy < 1.15f);

    // Half the motors pulling shows up as a proportionally higher kS and kV.
    const float weak = freeSpeedRatio(kS / 0.55f, kV / 0.55f, rated);
    CHECK(weak < 0.55f);

    // A wheel diameter set to twice the truth does the same.
    CHECK(freeSpeedRatio(kS, kV * 2.0f, rated) < 0.55f);

    CHECK(freeSpeedRatio(kS, 0.0f, rated) == 0.0f);
    CHECK(freeSpeedRatio(kS, kV, 0.0f) == 0.0f);
}

CHAD_TEST(characterizeFit, gatesRejectWhatTheyShould) {
    // Too few levels to fit at all.
    const VelocityPoint few[] = {{10.0f, 2.0f}, {20.0f, 4.0f}, {30.0f, 6.0f}};
    CHECK(!fitVelocity(few, 3, 0.9f).valid);
    CHECK(!fitVelocity(nullptr, 8, 0.9f).valid);

    // All at one speed: no slope to find.
    const VelocityPoint flat[] = {{10.0f, 2.0f}, {10.0f, 3.0f}, {10.0f, 4.0f},
                                  {10.0f, 5.0f}, {10.0f, 6.0f}};
    CHECK(!fitVelocity(flat, 5, 0.9f).valid);

    // A negative slope is not a drivetrain.
    const VelocityPoint backwards[] = {{40.0f, 2.0f}, {30.0f, 4.0f}, {20.0f, 6.0f},
                                       {10.0f, 8.0f}};
    CHECK(!fitVelocity(backwards, 4, 0.9f).valid);

    // Steps that never moved contribute nothing.
    const StepSample dead[] = {{0.0f, 0.0f, 7.0f}, {0.0f, 0.0f, 7.0f}};
    CHECK(!fitAccel(dead, 2, kS, kV, 0.5f).valid);

    // Reps that disagree wildly are reported, not averaged into a number that
    // looks fine.
    const StepSample messy[] = {{10.0f, 200.0f, 7.0f}, {10.0f, 40.0f, 7.0f},
                               {10.0f, 300.0f, 7.0f}, {10.0f, 25.0f, 7.0f}};
    const AccelFit m = fitAccel(messy, 4, kS, kV, 0.5f);
    CHECK(m.reps == 4);
    CHECK(m.spread > 0.5f);
    CHECK(!m.valid);

    // A median, not a mean: one wild rep must not move it much.
    const StepSample oneBad[] = {{10.0f, 200.0f, 7.0f}, {10.0f, 205.0f, 7.0f},
                                 {10.0f, 198.0f, 7.0f}, {10.0f, 2.0f, 7.0f},
                                 {10.0f, 202.0f, 7.0f}};
    const AccelFit med = fitAccel(oneBad, 5, kS, kV, 10.0f);
    const float clean = (7.0f - kS - kV * 10.0f) / 201.0f;
    CHECK_NEAR(med.kA, clean, 0.0002);
}
