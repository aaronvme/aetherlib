#include "chad/tracking/ekf_core.hpp"

#include <cstring>
#include <limits>
#include <memory>
#include <random>

#include "chad_test.hpp"

using namespace chad;

namespace {

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kWall = Field::FIELD_HALF_WALL;

/// Fourth-order central difference, so the comparison against an analytic
/// Jacobian is limited by float32 rather than by the stencil.
template <typename F>
double deriv(F f, float x, float h = 5e-3f) {
    return (-static_cast<double>(f(x + 2.0f * h)) + 8.0 * static_cast<double>(f(x + h)) -
            8.0 * static_cast<double>(f(x - h)) + static_cast<double>(f(x - 2.0f * h))) /
           (12.0 * static_cast<double>(h));
}

EkfCore::Config baseConfig() {
    EkfCore::Config c;
    // Rewind off, so a test that plants a reading is compared against the pose
    // it planted it for. The shipped default is nonzero -- see
    // ekfLatency/compensationIsOnByDefault, which pins that.
    c.rangeLatencyMs = 0.0f;
    c.imuLatencyMs = 0.0f;
    return c;
}

/// The confidence at which the range sigma scaling is the identity, so a test
/// that plants a reading gets the sigma it asked for.
constexpr int kConf = EkfCore::CONFIDENCE_NOMINAL;

float mmFor(float inches) { return inches * INCH_TO_MM; }

EkfCore::Beam beamAt(float inches) { return {true, mmFor(inches), kConf}; }

/// Leading principal minors: all positive is exactly positive-definiteness for
/// a symmetric matrix, and does not need an eigensolver.
bool positiveDefinite(const float* P) {
    const double m1 = P[0];
    const double m2 = static_cast<double>(P[0]) * P[4] - static_cast<double>(P[1]) * P[3];
    const double m3 =
        static_cast<double>(P[0]) * (static_cast<double>(P[4]) * P[8] - static_cast<double>(P[5]) * P[7]) -
        static_cast<double>(P[1]) * (static_cast<double>(P[3]) * P[8] - static_cast<double>(P[5]) * P[6]) +
        static_cast<double>(P[2]) * (static_cast<double>(P[3]) * P[7] - static_cast<double>(P[4]) * P[6]);
    return m1 > 0.0 && m2 > 0.0 && m3 > 0.0;
}

float asymmetry(const float* P) {
    float worst = 0.0f;
    for (int r = 0; r < 3; r++) {
        for (int c = r + 1; c < 3; c++) {
            worst = std::fmax(worst, std::fabs(P[r * 3 + c] - P[c * 3 + r]));
        }
    }
    return worst;
}

float trace(const float* P) { return P[0] + P[4] + P[8]; }

/// The textbook covariance update, P = (I - KH)P, written exactly as a naive
/// implementation would: no symmetrization, no floor. Present so the Joseph
/// form has something to be measured against.
void textbookUpdate(float* P, const float H[3], float R) {
    float PHt[3];
    for (int r = 0; r < 3; r++) {
        PHt[r] = P[r * 3 + 0] * H[0] + P[r * 3 + 1] * H[1] + P[r * 3 + 2] * H[2];
    }
    const float S = H[0] * PHt[0] + H[1] * PHt[1] + H[2] * PHt[2] + R;
    const float K[3] = {PHt[0] / S, PHt[1] / S, PHt[2] / S};

    float out[9];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            float acc = 0.0f;
            for (int k = 0; k < 3; k++) {
                const float a = (r == k ? 1.0f : 0.0f) - K[r] * H[k];
                acc += a * P[k * 3 + c];
            }
            out[r * 3 + c] = acc;
        }
    }
    std::memcpy(P, out, sizeof out);
}

/// Range from a robot at (x, y) with heading 0 to each of the four field walls,
/// derived from the field box rather than from the model under test.
float wallRangeForward(float x) { return kWall - x; }

struct FourBeams {
    std::array<Pose, EkfCore::SENSOR_COUNT> mounts{};
    FourBeams() {
        mounts[0] = {0.0f, 0.0f, 0.0f};        // +X
        mounts[1] = {0.0f, 0.0f, HALF_PI};     // +Y
        mounts[2] = {0.0f, 0.0f, PI};          // -X
        mounts[3] = {0.0f, 0.0f, -HALF_PI};    // -Y
    }
    /// Truth readings for a robot at (x, y) facing +X, from the field geometry.
    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> readings(float x, float y) const {
        std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
        b[0] = beamAt(kWall - x);
        b[1] = beamAt(kWall - y);
        b[2] = beamAt(kWall + x);
        b[3] = beamAt(kWall + y);
        return b;
    }
};

}  // namespace

// ============================================================================
// Process Jacobian against numerical differentiation
// ============================================================================

CHAD_TEST(ekfJacobian, processMatchesNumericalDifferentiation) {
    struct Point { Pose pose; OdomDelta delta; };
    const Point points[] = {
        {{0.0f, 0.0f, 0.0f},        {1.0f, 0.0f, 0.0f}},
        {{12.0f, -8.0f, 0.7f},      {0.9f, 0.0f, 0.11f}},
        {{-30.0f, 44.0f, -2.6f},    {0.4f, 0.25f, -0.35f}},
        {{5.0f, 5.0f, 3.05f},       {2.5f, -1.0f, 0.9f}},
        {{0.0f, 0.0f, 1.2f},        {0.0f, 0.0f, 0.6f}},
    };

    for (const Point& p : points) {
        float F[9];
        EkfCore::processJacobian(p.pose, p.delta, F);

        // d/dx and d/dy are structural: a translation of the start translates
        // the end by the same amount.
        for (int i = 0; i < 2; i++) {
            const int col = i;
            const double dx = deriv([&](float v) {
                Pose q = p.pose;
                (col == 0 ? q.x : q.y) = v;
                return EkfCore::propagate(q, p.delta).x;
            }, col == 0 ? p.pose.x : p.pose.y, 1.0f);
            const double dy = deriv([&](float v) {
                Pose q = p.pose;
                (col == 0 ? q.x : q.y) = v;
                return EkfCore::propagate(q, p.delta).y;
            }, col == 0 ? p.pose.x : p.pose.y, 1.0f);
            CHECK_NEAR(F[0 * 3 + col], dx, 2e-4);
            CHECK_NEAR(F[1 * 3 + col], dy, 2e-4);
        }

        const double dxdth = deriv([&](float v) {
            Pose q = p.pose;
            q.theta = v;
            return EkfCore::propagate(q, p.delta).x;
        }, p.pose.theta);
        const double dydth = deriv([&](float v) {
            Pose q = p.pose;
            q.theta = v;
            return EkfCore::propagate(q, p.delta).y;
        }, p.pose.theta);

        CHECK_NEAR(F[2], dxdth, 1e-3);
        CHECK_NEAR(F[5], dydth, 1e-3);
        CHECK_NEAR(F[6], 0.0, 1e-9);
        CHECK_NEAR(F[7], 0.0, 1e-9);
        CHECK_NEAR(F[8], 1.0, 1e-9);
    }
}

CHAD_TEST(ekfJacobian, transformMatchesNumericalDifferentiation) {
    const Transform2d t{{-6.0f, 2.5f}, Rotation2d::fromRadians(-0.4f)};
    const Pose points[] = {{0.0f, 0.0f, 0.0f}, {10.0f, -20.0f, 1.1f}, {-5.0f, 5.0f, 2.9f}};

    for (const Pose& p : points) {
        float J[9];
        EkfCore::transformJacobian(p, t, J);
        const double dxdth = deriv([&](float v) {
            Pose q = p;
            q.theta = v;
            return q.transformBy(t).x;
        }, p.theta);
        const double dydth = deriv([&](float v) {
            Pose q = p;
            q.theta = v;
            return q.transformBy(t).y;
        }, p.theta);
        CHECK_NEAR(J[2], dxdth, 1e-3);
        CHECK_NEAR(J[5], dydth, 1e-3);
        // All nine entries, differentiated: the four off-diagonal zeros used to
        // go unchecked, so a J that leaked x into y passed.
        for (int col = 0; col < 2; col++) {
            const double dx = deriv([&](float v) {
                Pose q = p;
                (col == 0 ? q.x : q.y) = v;
                return q.transformBy(t).x;
            }, col == 0 ? p.x : p.y, 1.0f);
            const double dy = deriv([&](float v) {
                Pose q = p;
                (col == 0 ? q.x : q.y) = v;
                return q.transformBy(t).y;
            }, col == 0 ? p.x : p.y, 1.0f);
            const double dth = deriv([&](float v) {
                Pose q = p;
                (col == 0 ? q.x : q.y) = v;
                return q.transformBy(t).theta;
            }, col == 0 ? p.x : p.y, 1.0f);
            CHECK_NEAR(J[0 * 3 + col], dx, 2e-4);
            CHECK_NEAR(J[1 * 3 + col], dy, 2e-4);
            CHECK_NEAR(J[2 * 3 + col], dth, 2e-4);
        }
        CHECK_NEAR(J[8], 1.0, 1e-9);
    }
}

// ============================================================================
// Range measurement Jacobian against numerical differentiation
// ============================================================================

CHAD_TEST(ekfJacobian, rangeMatchesNumericalDifferentiation) {
    EkfCore core(baseConfig());

    struct Point { Pose pose; Pose mount; };
    const Point points[] = {
        {{0.0f, 0.0f, 0.0f},        {0.0f, 0.0f, 0.0f}},          // +X wall, centred
        {{10.0f, -15.0f, 0.4f},     {6.0f, 3.0f, -0.3f}},         // +X wall, offset mount
        {{5.0f, 20.0f, 1.3f},       {0.0f, 0.0f, 0.0f}},          // +Y wall
        {{-30.0f, -40.0f, 3.0f},    {2.0f, -1.0f, 0.2f}},         // -X wall
        {{0.0f, -10.0f, -1.4f},     {-3.0f, 1.5f, 0.1f}},         // -Y wall
    };

    for (const Point& p : points) {
        const EkfCore::RangePrediction rp = core.predictRange(p.pose, p.mount);
        CHECK(rp.valid());
        if (!rp.valid()) continue;

        auto rangeAt = [&](const Pose& q) {
            const EkfCore::RangePrediction r = core.predictRange(q, p.mount);
            return r.range;
        };

        const double drdx = deriv([&](float v) { Pose q = p.pose; q.x = v; return rangeAt(q); },
                                  p.pose.x);
        const double drdy = deriv([&](float v) { Pose q = p.pose; q.y = v; return rangeAt(q); },
                                  p.pose.y);
        const double drdt = deriv([&](float v) { Pose q = p.pose; q.theta = v; return rangeAt(q); },
                                  p.pose.theta);

        CHECK_NEAR(rp.H[0], drdx, 2e-3);
        CHECK_NEAR(rp.H[1], drdy, 2e-3);
        CHECK_NEAR(rp.H[2], drdt, 4e-2);
    }
}

CHAD_TEST(ekfJacobian, rewoundRangeJacobianComposes) {
    EkfCore core(baseConfig());
    const Pose mount{4.0f, -2.0f, 0.25f};
    const Pose now{8.0f, -12.0f, 0.35f};
    // The rewind is a rigid transform in the robot frame, exactly what the
    // odometry trail hands back.
    const Transform2d back{{-9.0f, 1.2f}, Rotation2d::fromRadians(-0.07f)};

    const EkfCore::RangePrediction rp = core.predictRange(now.transformBy(back), mount);
    CHECK(rp.valid());
    float J[9];
    EkfCore::transformJacobian(now, back, J);

    float H[3];
    for (int c = 0; c < 3; c++) {
        H[c] = rp.H[0] * J[0 * 3 + c] + rp.H[1] * J[1 * 3 + c] + rp.H[2] * J[2 * 3 + c];
    }

    auto composite = [&](const Pose& q) {
        return core.predictRange(q.transformBy(back), mount).range;
    };
    const double dx = deriv([&](float v) { Pose q = now; q.x = v; return composite(q); }, now.x);
    const double dy = deriv([&](float v) { Pose q = now; q.y = v; return composite(q); }, now.y);
    const double dt = deriv([&](float v) { Pose q = now; q.theta = v; return composite(q); },
                            now.theta);

    CHECK_NEAR(H[0], dx, 2e-3);
    CHECK_NEAR(H[1], dy, 2e-3);
    CHECK_NEAR(H[2], dt, 5e-2);
}

// ============================================================================
// Joseph form vs the textbook form
// ============================================================================

CHAD_TEST(ekfCovariance, josephKeepsWhatTheTextbookFormRoundsAway) {
    // A single near-certain measurement against a very uncertain prior. The
    // posterior variance is R, which is 8 orders of magnitude below the prior:
    // the textbook form has to find it by subtracting two numbers that agree to
    // more digits than float32 has, and gets zero.
    EkfCore core(baseConfig());
    core.reset({0.0f, 0.0f, 0.0f}, 100.0f, 0.05f, 0);

    float ref[9];
    std::memcpy(ref, core.covariance(), sizeof ref);
    CHECK(positiveDefinite(ref));
    CHECK_NEAR(ref[0], 1e4f, 1.0f);

    const float H[3] = {1.0f, 0.0f, 0.0f};
    const float R = 1e-4f;
    core.fuseScalar(H, 0.0f, R, 0.0f);
    textbookUpdate(ref, H, R);

    CHECK_NEAR(core.covariance()[0], R, 1e-6);
    CHECK(positiveDefinite(core.covariance()));
    // The textbook form has lost the variance entirely, so P is singular and no
    // Cholesky of it exists -- and the next update divides by it.
    CHECK(!positiveDefinite(ref));
    CHECK(ref[0] < 0.5f * R);
}

CHAD_TEST(ekfCovariance, josephHoldsWhereTextbookFails) {
    EkfCore core(baseConfig());
    core.reset({10.0f, 20.0f, 0.3f}, 60.0f, 0.4f, 0);
    // A tick of motion, so P has real correlations rather than a diagonal.
    core.predict({1.0f, 0.0f, 0.05f}, false, 10);

    float ref[9];
    std::memcpy(ref, core.covariance(), sizeof ref);
    CHECK(positiveDefinite(ref));

    // Near-certain measurements along slowly rotating directions: the update
    // drives P toward singular, which is where the two forms part company.
    const float R = 1e-4f;
    int textbookFailedAt = -1;
    for (int i = 0; i < 4000; i++) {
        const float a = 0.017f * static_cast<float>(i);
        const float H[3] = {std::cos(a), std::sin(a), 0.35f * std::cos(0.5f * a)};

        core.fuseScalar(H, 0.0f, R, 0.0f);
        textbookUpdate(ref, H, R);

        if (textbookFailedAt < 0 && (!positiveDefinite(ref) || asymmetry(ref) > 1e-6f)) {
            textbookFailedAt = i;
        }
    }

    CHECK(positiveDefinite(core.covariance()));
    CHECK_NEAR(asymmetry(core.covariance()), 0.0, 1e-12);
    CHECK_FINITE(trace(core.covariance()));
    CHECK(trace(core.covariance()) > 0.0f);
    CHECK_EQ(core.covarianceRepairs(), 0u);
    // The point of the exercise: the cheap form does not survive this.
    CHECK(textbookFailedAt >= 0);
}

CHAD_TEST(ekfCovariance, staysPositiveDefiniteOverALongRun) {
    EkfCore core(baseConfig());
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);
    FourBeams rig;
    core.setMounts(rig.mounts);

    // Driven off a TRUTH pose, not off the filter's own state. Feeding
    // updateHeading(core.state().theta) makes the heading innovation identically
    // zero, so no heading-measurement defect is reachable and the range readings
    // never disagree with the prediction either.
    Pose truth{0.0f, 0.0f, 0.0f};
    const OdomDelta tick{0.35f, 0.0f, 0.010f};   // a 35 inch radius circle
    std::uint32_t t = 0;
    int headingMoved = 0;
    for (int i = 0; i < 3000; i++) {
        t += 10;
        truth = EkfCore::propagate(truth, tick);
        core.predict(tick, false, t);
        const float before = core.state().theta;
        core.updateHeading(truth.theta, t);
        if (std::fabs(angleDiffRad(before, core.state().theta)) > 1e-7f) headingMoved++;
        core.updateRanges(rig.readings(truth.x, truth.y), t);
        CHECK(positiveDefinite(core.covariance()));
        if (!positiveDefinite(core.covariance())) break;
    }
    // The filter tracked the truth all the way round, and the heading update
    // really did have something to correct on most ticks.
    CHECK_NEAR(core.state().x, truth.x, 1.0f);
    CHECK_NEAR(core.state().y, truth.y, 1.0f);
    CHECK_NEAR(angleDiffRad(core.state().theta, truth.theta), 0.0f, 0.02f);
    CHECK(headingMoved > 1500);
    CHECK_NEAR(asymmetry(core.covariance()), 0.0, 1e-12);
    CHECK_EQ(core.covarianceRepairs(), 0u);
}

CHAD_TEST(ekfCovariance, shrinksUnderMeasurementAndGrowsUnderPrediction) {
    EkfCore core(baseConfig());
    core.reset({0.0f, 0.0f, 0.0f}, 4.0f, 0.05f, 0);
    FourBeams rig;
    core.setMounts(rig.mounts);

    const float startTrace = trace(core.covariance());
    std::uint32_t t = 0;
    for (int i = 0; i < 40; i++) {
        t += 10;
        core.predict({0.0f, 0.0f, 0.0f}, false, t);
        core.updateHeading(0.0f, t);
        core.updateRanges(rig.readings(0.0f, 0.0f), t);
    }
    const float afterMeasurement = trace(core.covariance());
    CHECK(afterMeasurement < startTrace);
    CHECK(core.beamsUsed() == 4);

    for (int i = 0; i < 40; i++) {
        t += 10;
        core.predict({0.0f, 0.0f, 0.0f}, false, t);
    }
    CHECK(trace(core.covariance()) > afterMeasurement);
}

CHAD_TEST(ekfCovariance, parkedPredictionGrowsByTheNoiseFloor) {
    EkfCore::Config cfg = baseConfig();
    EkfCore core(cfg);
    core.reset({0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0);
    core.predict({0.0f, 0.0f, 0.0f}, false, 10);
    // Floors only, since nothing moved.
    CHECK_NEAR(core.covariance()[0], cfg.posNoiseFloor * cfg.posNoiseFloor, 1e-8);
    CHECK_NEAR(core.covariance()[4], cfg.slipNoiseFloor * cfg.slipNoiseFloor, 1e-8);
    CHECK_NEAR(core.covariance()[8], cfg.thetaNoiseFloor * cfg.thetaNoiseFloor, 2e-9);
}

// ============================================================================
// Process noise frame
// ============================================================================

CHAD_TEST(ekfProcessNoise, isAnisotropicInTheBodyFrame) {
    EkfCore::Config cfg = baseConfig();
    // Isolate the frame question from the bias term.
    cfg.vertBiasProp = 0.0f;
    cfg.slipBiasProp = 0.0f;

    const float ds = 4.0f;
    const float along = cfg.vertNoiseProp * ds + cfg.posNoiseFloor;
    const float lateral = cfg.slipNoiseProp * ds + cfg.slipNoiseFloor;
    CHECK(along > lateral);

    EkfCore facingX(cfg);
    facingX.reset({0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0);
    facingX.predict({ds, 0.0f, 0.0f}, false, 10);
    CHECK_NEAR(facingX.covariance()[0], along * along, 1e-6);
    CHECK_NEAR(facingX.covariance()[4], lateral * lateral, 1e-6);
    CHECK_NEAR(facingX.covariance()[1], 0.0, 1e-6);

    EkfCore facingY(cfg);
    facingY.reset({0.0f, 0.0f, HALF_PI}, 0.0f, 0.0f, 0);
    facingY.predict({ds, 0.0f, 0.0f}, false, 10);
    // Same body-frame noise, rotated: the axes swap.
    CHECK_NEAR(facingY.covariance()[0], lateral * lateral, 1e-6);
    CHECK_NEAR(facingY.covariance()[4], along * along, 1e-6);
    CHECK_NEAR(facingY.covariance()[1], 0.0, 1e-6);

    EkfCore diagonal(cfg);
    diagonal.reset({0.0f, 0.0f, QUARTER_PI}, 0.0f, 0.0f, 0);
    diagonal.predict({ds, 0.0f, 0.0f}, false, 10);
    // At 45 degrees the variances are equal and the correlation is maximal.
    const float half = 0.5f * (along * along + lateral * lateral);
    CHECK_NEAR(diagonal.covariance()[0], half, 1e-6);
    CHECK_NEAR(diagonal.covariance()[4], half, 1e-6);
    CHECK_NEAR(diagonal.covariance()[1], 0.5f * (along * along - lateral * lateral), 1e-6);
}

CHAD_TEST(ekfProcessNoise, midpointFrameSplitsTheRotationOfATurn) {
    EkfCore::Config cfg = baseConfig();
    cfg.vertBiasProp = 0.0f;
    cfg.slipBiasProp = 0.0f;
    EkfCore core(cfg);
    core.reset({0.0f, 0.0f, -QUARTER_PI}, 0.0f, 0.0f, 0);
    // Start at -45 and turn to +45: the midpoint is 0, so the noise ellipse
    // should land axis-aligned even though neither end of the tick is.
    core.predict({4.0f, 0.0f, HALF_PI}, false, 10);
    CHECK_NEAR(core.covariance()[1], 0.0, 1e-6);
    CHECK(core.covariance()[0] > core.covariance()[4]);
}

CHAD_TEST(ekfProcessNoise, systematicBiasGrowsLinearlyInDistance) {
    EkfCore::Config cfg = baseConfig();
    EkfCore many(cfg), one(cfg);
    many.reset({0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0);
    one.reset({0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0);

    // 40 inches as 400 ticks against 40 inches as 1 tick: the white term is
    // 100x smaller when split up, the bias term is not. That difference is the
    // whole reason the bias term exists.
    std::uint32_t t = 0;
    for (int i = 0; i < 400; i++) {
        t += 10;
        many.predict({0.1f, 0.0f, 0.0f}, false, t);
    }
    one.predict({40.0f, 0.0f, 0.0f}, false, 10);

    const float bias = cfg.vertBiasProp * cfg.vertBiasProp * 40.0f * cfg.biasDecorrIn;
    CHECK(many.covariance()[0] > 0.5f * bias);
    CHECK(many.covariance()[0] < one.covariance()[0]);
}

// ============================================================================
// Gating
// ============================================================================

CHAD_TEST(ekfGate, acceptsATruthfulRangeAndRejectsAPlantedOutlier) {
    EkfCore core(baseConfig());
    FourBeams rig;
    core.setMounts(rig.mounts);
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);

    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> good{};
    good[0] = beamAt(wallRangeForward(0.0f));
    core.updateRanges(good, 10);
    CHECK(core.beamStates()[0] == EkfCore::BeamState::Used);
    CHECK_EQ(core.beamsUsed(), 1);
    CHECK_EQ(core.rangeRejects(), 0u);
    CHECK_EQ(core.rangeRejectStreak(), 0);

    // A beam that ranged another robot instead of the wall.
    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> bad{};
    bad[0] = beamAt(wallRangeForward(0.0f) - 35.0f);
    const Pose before = core.state();
    core.updateRanges(bad, 20);
    CHECK(core.beamStates()[0] == EkfCore::BeamState::Rejected);
    CHECK_EQ(core.beamsUsed(), 0);
    CHECK_EQ(core.rangeRejects(), 1u);
    CHECK_EQ(core.rangeRejectStreak(), 1);
    CHECK_NEAR(core.state().x, before.x, 1e-6);
    CHECK_NEAR(core.state().y, before.y, 1e-6);
    // The innovation is still reported, so a rejection is diagnosable.
    CHECK_NEAR(core.innovations()[0], -35.0f, 0.05);
}

CHAD_TEST(ekfGate, rejectStreakMakesASilentlyDeafFilterVisible) {
    EkfCore core(baseConfig());
    FourBeams rig;
    core.setMounts(rig.mounts);
    core.reset({0.0f, 0.0f, 0.0f}, 0.05f, 0.001f, 0);

    // Overconfident by 30 inches: every beam is outside the gate for ever, and
    // from the outside that is indistinguishable from having no sensors.
    std::uint32_t t = 0;
    for (int i = 0; i < 25; i++) {
        t += 10;
        core.predict({0.0f, 0.0f, 0.0f}, false, t);
        core.updateRanges(rig.readings(30.0f, 25.0f), t);
    }
    CHECK_EQ(core.beamsUsed(), 0);
    CHECK(core.rangeRejectStreak() >= 25);
    CHECK(core.rangeRejects() >= 100u);
    CHECK_NEAR(core.state().x, 0.0f, 0.5f);
}

CHAD_TEST(ekfGate, chiSquareThresholdIsTheDocumentedSigmaCount) {
    // Through the CONFIG path, not by handing fuseScalar a literal: the old
    // version set cfg.rangeGateChi2 = 9.0f and then passed 9.0f by hand, so
    // rangeGateChi2 could be 300 and every range assertion in the suite still
    // passed. With rangeSigmaProp zeroed, R is exactly rangeSigmaBase^2 and
    // S = P00 + R is known, so the boundary is arithmetic.
    EkfCore::Config cfg = baseConfig();       // DEFAULT rangeGateChi2
    cfg.rangeSigmaProp = 0.0f;
    cfg.rangeSigmaBase = 0.5f;

    auto stateFor = [&cfg](float innovation) {
        EkfCore core(cfg);
        std::array<Pose, EkfCore::SENSOR_COUNT> mounts{};
        core.setMounts(mounts);               // beam 0 forward from the centre
        core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);
        std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
        b[0] = beamAt(wallRangeForward(0.0f) + innovation);
        core.updateRanges(b, 10);
        return core.beamStates()[0];
    };

    const float S = 1.0f + 0.25f;
    const float edge = std::sqrt(9.0f * S);   // 3 sigma
    CHECK(stateFor(0.98f * edge) == EkfCore::BeamState::Used);
    CHECK(stateFor(1.02f * edge) == EkfCore::BeamState::Rejected);
    CHECK(stateFor(-0.98f * edge) == EkfCore::BeamState::Used);
    CHECK(stateFor(-1.02f * edge) == EkfCore::BeamState::Rejected);
    // Which pins the shipped default into (8.64, 9.36).
    CHECK(cfg.rangeGateChi2 > 9.0f * 0.98f * 0.98f);
    CHECK(cfg.rangeGateChi2 < 9.0f * 1.02f * 1.02f);

    // The arithmetic on its own, still worth having: fuseScalar with an explicit
    // threshold and a known S.
    EkfCore direct(cfg);
    direct.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);
    const float H[3] = {0.0f, 0.0f, 1.0f};
    const float R = 0.01f;
    const float Sh = direct.covariance()[8] + R;
    const float edgeH = std::sqrt(9.0f * Sh);
    CHECK(!direct.fuseScalar(H, edgeH * 1.001f, R, 9.0f));
    CHECK(direct.fuseScalar(H, edgeH * 0.999f, R, 9.0f));
}

CHAD_TEST(ekfGate, headingGateIsOffByDefaultAndCountsWhenOn) {
    EkfCore::Config cfg = baseConfig();
    EkfCore core(cfg);
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.001f, 0);
    // Default: the IMU is the only absolute heading source, so a wild reading
    // still lands. That is a deliberate trade, and it is worth pinning down.
    core.updateHeading(0.9f, 10);
    CHECK(core.state().theta > 0.0f);
    CHECK_EQ(core.headingRejects(), 0u);

    cfg.imuGateChi2 = 9.0f;
    EkfCore gated(cfg);
    gated.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.001f, 0);
    gated.updateHeading(0.9f, 10);
    CHECK_NEAR(gated.state().theta, 0.0f, 1e-6);
    CHECK_EQ(gated.headingRejects(), 1u);
}

// ============================================================================
// Angle handling
// ============================================================================

CHAD_TEST(ekfAngles, headingInnovationCrossesTheBranchCut) {
    EkfCore core(baseConfig());
    core.reset({0.0f, 0.0f, 3.10f}, 1.0f, 0.05f, 0);

    const float measured = -3.10f;   // 0.083 rad away, the short way round
    const float before = core.state().theta;
    core.updateHeading(measured, 10);
    const float after = core.state().theta;

    // Unwrapped, the innovation would have been -6.20 rad and the state would
    // have swung most of a turn.
    CHECK(std::fabs(angleDiffRad(before, after)) < 0.09f);
    CHECK(std::fabs(after) > 3.0f);
    CHECK(std::fabs(angleDiffRad(after, measured)) < std::fabs(angleDiffRad(before, measured)));
}

CHAD_TEST(ekfAngles, headingConvergesAcrossPiFromEitherSide) {
    for (int dir = 0; dir < 2; dir++) {
        const float start = dir == 0 ? 3.13f : -3.13f;
        const float target = dir == 0 ? -3.10f : 3.10f;
        EkfCore core(baseConfig());
        core.reset({0.0f, 0.0f, start}, 1.0f, 0.05f, 0);
        std::uint32_t t = 0;
        for (int i = 0; i < 60; i++) {
            t += 10;
            core.predict({0.0f, 0.0f, 0.0f}, false, t);
            core.updateHeading(target, t);
        }
        CHECK_NEAR(angleDiffRad(core.state().theta, target), 0.0f, 1e-3f);
        // Still on the branch it belongs on, not unwrapped past it.
        CHECK(core.state().theta >= -PI && core.state().theta <= PI);
    }
}

CHAD_TEST(ekfAngles, predictedHeadingAccumulatesOnTheRightBranch) {
    // The old version only asserted |theta| <= PI, which is a property of
    // atan2's range and holds however wrongly the heading integrates. Assert the
    // TOTAL rotation instead: 500 ticks of 0.05 rad is 25 rad, and the wrapped
    // state has to agree with it at every step.
    EkfCore core(baseConfig());
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);
    std::uint32_t t = 0;
    float turned = 0.0f;
    for (int i = 0; i < 500; i++) {
        t += 10;
        core.predict({0.1f, 0.0f, 0.05f}, false, t);
        turned += 0.05f;
        CHECK_NEAR(angleDiffRad(core.state().theta, wrapRad(turned)), 0.0f, 5e-3f);
        CHECK(core.state().theta >= -PI - 1e-4f);
        CHECK(core.state().theta <= PI + 1e-4f);
        if (std::fabs(angleDiffRad(core.state().theta, wrapRad(turned))) > 5e-3f) break;
    }
    CHECK_NEAR(turned, 25.0f, 1e-3f);
    // 0.1 in per 0.05 rad is a 2 inch radius circle, so where it ends is the
    // closed form of the arc, not wherever repeated wrapping left it.
    const float R = 0.1f / 0.05f;
    CHECK_NEAR(core.state().x, R * std::sin(25.0f), 0.02f);
    CHECK_NEAR(core.state().y, R * (1.0f - std::cos(25.0f)), 0.02f);
}

// ============================================================================
// Convergence
// ============================================================================

CHAD_TEST(ekfConvergence, findsTheRobotFromAWrongInitialPose) {
    EkfCore core(baseConfig());
    FourBeams rig;
    core.setMounts(rig.mounts);
    // Wrong by (8, -6), and honest about it: a 10 inch sigma opens the gate
    // wide enough for the beams to be heard.
    core.reset({8.0f, -6.0f, 0.05f}, 10.0f, 0.10f, 0);

    std::uint32_t t = 0;
    for (int i = 0; i < 200; i++) {
        t += 10;
        core.predict({0.0f, 0.0f, 0.0f}, false, t);
        core.updateHeading(0.0f, t);
        core.updateRanges(rig.readings(0.0f, 0.0f), t);
    }

    CHECK_NEAR(core.state().x, 0.0f, 0.15f);
    CHECK_NEAR(core.state().y, 0.0f, 0.15f);
    CHECK_NEAR(core.state().theta, 0.0f, 0.01f);
    CHECK(core.uncertainty().sigmaX < 1.0f);
    CHECK(core.uncertainty().sigmaY < 1.0f);
}

CHAD_TEST(ekfConvergence, trackksAMovingRobotWhoseOdometryUnderreports) {
    EkfCore core(baseConfig());
    FourBeams rig;
    core.setMounts(rig.mounts);
    core.reset({-40.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);

    // Odometry says 0.5 in per tick, the robot really covers 0.53: a 6% scale
    // error, the exact failure the bias term in Q is there to keep gateable.
    float truthX = -40.0f;
    std::uint32_t t = 0;
    for (int i = 0; i < 150; i++) {
        t += 10;
        truthX += 0.53f;
        core.predict({0.5f, 0.0f, 0.0f}, false, t);
        core.updateHeading(0.0f, t);
        core.updateRanges(rig.readings(truthX, 0.0f), t);
    }

    CHECK_NEAR(core.state().x, truthX, 1.0f);
    CHECK(core.beamsUsed() > 0);
    CHECK(core.rangeRejectStreak() == 0);
}

CHAD_TEST(ekfConvergence, sequentialBeamsOnOneWallDoNotOvershoot) {
    EkfCore::Config cfg = baseConfig();
    // Near-certain beams, which is where stale linearisation stops merely
    // converging slowly and starts walking the estimate the wrong way.
    cfg.rangeSigmaBase = 0.02f;
    cfg.rangeSigmaProp = 0.0f;
    cfg.rangeGateChi2 = 0.0f;
    EkfCore core(cfg);

    std::array<Pose, EkfCore::SENSOR_COUNT> mounts{};
    for (int i = 0; i < 4; i++) {
        mounts[static_cast<std::size_t>(i)] = {0.0f, static_cast<float>(i) - 1.5f, 0.0f};
    }
    core.setMounts(mounts);
    core.reset({4.0f, 0.0f, 0.0f}, 3.0f, 1e-3f, 0);

    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> beams{};
    for (int i = 0; i < 4; i++) {
        beams[static_cast<std::size_t>(i)] = beamAt(wallRangeForward(0.0f));
    }

    core.updateRanges(beams, 10);
    CHECK_EQ(core.beamsUsed(), 4);
    // Four beams that agree must move the estimate toward the truth and stop
    // there. Re-using one linearisation for all four sends it past zero.
    CHECK(core.state().x >= -1e-3f);
    CHECK(core.state().x < 4.0f);
}

// ============================================================================
// Latency compensation
// ============================================================================

CHAD_TEST(ekfLatency, compensationIsOnByDefault) {
    // rangeLatencyMs used to default to zero, which is the one value guaranteed
    // wrong: fusing a reading taken 50 ms ago against the present pose is a bias
    // of speed*latency -- 3.00 inches at 60 in/s -- and more beams do not average
    // a bias away. The default is now a documented ESTIMATE of the port latency,
    // and imuLatencyMs stays zero on purpose because the state's heading is
    // propagated from the same IMU stream.
    EkfCore::Config shipped;
    CHECK(shipped.rangeLatencyMs > 0.0f);
    CHECK(shipped.rangeLatencyMs < 100.0f);
    CHECK_NEAR(shipped.imuLatencyMs, 0.0f, 1e-9);

    // With the rewind switched off, the staleness lands as innovation. baseConfig
    // zeroes it so the rest of the suite can plant readings against the present
    // pose without a rewind in the way.
    EkfCore::Config off = baseConfig();
    CHECK_NEAR(off.rangeLatencyMs, 0.0f, 1e-9);
    EkfCore core(off);
    FourBeams rig;
    core.setMounts(rig.mounts);
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);

    std::uint32_t t = 0;
    for (int i = 0; i < 20; i++) {
        t += 10;
        core.predict({1.0f, 0.0f, 0.0f}, false, t);
    }
    // 20 inches driven; a reading that describes the pose 100 ms ago is 10
    // inches stale, and with the rewind off the filter sees that as innovation.
    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = beamAt(wallRangeForward(10.0f));
    core.updateRanges(b, t);
    CHECK_NEAR(core.innovations()[0], 10.0f, 0.2f);

    // The same trace with the shipped default rewinds part of it away, which is
    // the whole point: the residual is speed * (true - configured) rather than
    // speed * true.
    EkfCore on(shipped);
    on.setMounts(rig.mounts);
    on.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);
    t = 0;
    for (int i = 0; i < 20; i++) {
        t += 10;
        on.predict({1.0f, 0.0f, 0.0f}, false, t);
    }
    on.updateRanges(b, t);
    CHECK_NEAR(on.innovations()[0], 10.0f - 0.1f * shipped.rangeLatencyMs, 0.2f);
    CHECK(std::fabs(on.innovations()[0]) < std::fabs(core.innovations()[0]));
}

CHAD_TEST(ekfLatency, rewindRemovesTheStalenessOfARangeReading) {
    EkfCore::Config cfg = baseConfig();
    cfg.rangeLatencyMs = 100.0f;
    EkfCore core(cfg);
    FourBeams rig;
    core.setMounts(rig.mounts);
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);

    std::uint32_t t = 0;
    for (int i = 0; i < 20; i++) {
        t += 10;
        core.predict({1.0f, 0.0f, 0.0f}, false, t);
    }
    CHECK_NEAR(core.state().x, 20.0f, 1e-3f);

    // The same stale reading, now compared against where the robot was when the
    // beam left rather than where it is now.
    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = beamAt(wallRangeForward(10.0f));
    core.updateRanges(b, t);
    CHECK(core.beamStates()[0] == EkfCore::BeamState::Used);
    CHECK_NEAR(core.innovations()[0], 0.0f, 0.05f);
    CHECK_NEAR(core.state().x, 20.0f, 0.05f);
}

CHAD_TEST(ekfLatency, rewindIsSkippedUntilTheTrailIsLongEnough) {
    EkfCore::Config cfg = baseConfig();
    cfg.rangeLatencyMs = 100.0f;
    EkfCore core(cfg);
    FourBeams rig;
    core.setMounts(rig.mounts);
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 1000);

    // Two ticks in, there is nothing 100 ms back to rewind to, so the filter
    // falls back to the present pose instead of extrapolating off the end.
    core.predict({1.0f, 0.0f, 0.0f}, false, 1010);
    core.predict({1.0f, 0.0f, 0.0f}, false, 1020);
    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = beamAt(wallRangeForward(2.0f));
    core.updateRanges(b, 1020);
    CHECK_NEAR(core.innovations()[0], 0.0f, 0.05f);
}

CHAD_TEST(ekfLatency, rewindHandlesAMillisecondCounterThatWraps) {
    EkfCore::Config cfg = baseConfig();
    cfg.rangeLatencyMs = 100.0f;
    EkfCore core(cfg);
    FourBeams rig;
    core.setMounts(rig.mounts);

    std::uint32_t t = 0xFFFFFF00u;
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, t);
    for (int i = 0; i < 40; i++) {
        t += 10;   // wraps through zero partway through
        core.predict({1.0f, 0.0f, 0.0f}, false, t);
    }
    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = beamAt(wallRangeForward(30.0f));
    core.updateRanges(b, t);
    CHECK_NEAR(core.innovations()[0], 0.0f, 0.05f);
}

// ============================================================================
// MCL as a 2-dof position measurement
// ============================================================================

CHAD_TEST(ekfMcl, foldsInAConsistentFixAndGatesAnInconsistentOne) {
    EkfCore core(baseConfig());
    core.reset({0.0f, 0.0f, 0.0f}, 2.0f, 0.02f, 0);

    EkfCore::Inputs in;
    in.mclValid = true;
    in.mclPose = {2.0f, -1.0f, 0.0f};
    in.mclCovXX = in.mclCovYY = 1.0f;
    in.mclNeffFrac = 0.8f;
    core.updateMcl(in);
    CHECK(core.state().x > 0.3f);
    CHECK(core.state().y < -0.1f);
    CHECK_EQ(core.mclRejects(), 0u);

    const Pose before = core.state();
    in.mclPose = {60.0f, 60.0f, 0.0f};
    core.updateMcl(in);
    CHECK_NEAR(core.state().x, before.x, 1e-6);
    CHECK_EQ(core.mclRejects(), 1u);
    CHECK_EQ(core.mclRejectStreak(), 1);

    // The two assertions above span NIS 0.76 to 1097, so on their own they hold
    // for any threshold in between -- including 9.0, the ONE-dof value, which is
    // exactly the confusion a 2-dof gate has to be protected against. These two
    // probes straddle 9.0 and 11.8.
    //
    // reset(posSigma 2) gives P00 = P11 = 4 and P01 = 0; mclCov 1.0 scaled by
    // mclSigmaScale^2 = 2.56 clears the 1.44 floor, so S = 6.56 on each axis and
    // NIS = (vx^2 + vy^2) / 6.56 exactly.
    auto accepts = [](float vx, float vy) {
        EkfCore probe(baseConfig());
        probe.reset({0.0f, 0.0f, 0.0f}, 2.0f, 0.02f, 0);
        EkfCore::Inputs fix;
        fix.mclValid = true;
        fix.mclPose = {vx, vy, 0.0f};
        fix.mclCovXX = fix.mclCovYY = 1.0f;
        fix.mclNeffFrac = 0.8f;
        probe.updateMcl(fix);
        return probe.mclRejects() == 0u;
    };
    const float S = 4.0f + 2.56f;
    CHECK_NEAR(8.30f * 8.30f / S, 10.50, 0.05);   // between 9.0 and 11.8
    CHECK_NEAR(9.24f * 9.24f / S, 13.01, 0.05);   // past 11.8
    CHECK(accepts(8.30f, 0.0f));
    CHECK(!accepts(9.24f, 0.0f));
    // Off-axis too, so a gate that only looks at x fails as well.
    CHECK(accepts(0.0f, 8.30f));
    CHECK(!accepts(6.55f, 6.55f));
    const EkfCore::Config cfg = baseConfig();
    CHECK(cfg.mclGateChi2 > 8.30f * 8.30f / S);
    CHECK(cfg.mclGateChi2 < 9.24f * 9.24f / S);
}

/// A filter at the origin, MCL insisting on `mclPose`, and beams that are the
/// truth readings from `truth`. The three are independent on purpose: which of
/// the two poses the readings support is the whole question.
struct RelocRig {
    FourBeams rig;
    EkfCore::Config cfg{baseConfig()};
    EkfCore::Inputs in;

    RelocRig(const Pose& mclPose, float truthX, float truthY) {
        in.mclValid = true;
        in.mclPose = mclPose;
        in.mclCovXX = in.mclCovYY = 1.0f;
        // Whatever a lost cloud happens to report. Nothing may depend on it.
        in.mclNeffFrac = 0.9f;
        in.beams = rig.readings(truthX, truthY);
    }
    std::unique_ptr<EkfCore> core() {
        auto c = std::make_unique<EkfCore>(cfg);
        c->setMounts(rig.mounts);
        c->reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);
        return c;
    }
};

CHAD_TEST(ekfMcl, relocatesOnlyWhenTheRangesBackMclAndNotTheFilter) {
    // The readings adjudicate, not the cloud's self-report. Both branches below
    // present an identical persistent 25 in disagreement with an identical
    // mclNeffFrac; only the beams differ, and only one relocates. The gate this
    // replaced tested mclNeffFrac >= 0.20, which is the same in both, so it
    // relocated in both -- including onto a pose the walls contradict.
    const Pose mcl{20.0f, -15.0f, 0.0f};

    // MCL is right: the readings are the ones a robot at (20, -15) produces.
    {
        RelocRig r(mcl, 20.0f, -15.0f);
        auto core = r.core();
        float self = 0.0f, other = 0.0f;
        const int scored = core->compareRangeFit(r.in.beams, mcl, &self, &other);
        CHECK_EQ(scored, 4);
        CHECK(self > other * r.cfg.mclRelocEvidenceRatio);
        std::printf("      mcl right: chi2 self %.1f mcl %.3f\n",
                    static_cast<double>(self), static_cast<double>(other));

        for (int i = 0; i < r.cfg.mclResetCycles - 1; i++) core->updateMcl(r.in);
        CHECK_EQ(core->relocateCount(), 0);   // persistence is still required
        core->updateMcl(r.in);
        CHECK_EQ(core->relocateCount(), 1);
        CHECK_NEAR(core->state().x, 20.0f, 1e-4f);
        CHECK_NEAR(core->state().y, -15.0f, 1e-4f);
        CHECK(core->consumeRelocation());
        CHECK(!core->consumeRelocation());
        CHECK_EQ(core->mclRejectStreak(), 0);
        CHECK(positiveDefinite(core->covariance()));
    }

    // MCL is wrong: the readings are the ones the filter's own pose predicts.
    // This is the case that matters -- relocating here throws a correct filter
    // 25 in off and gates out every beam that would have argued.
    {
        RelocRig r(mcl, 0.0f, 0.0f);
        auto core = r.core();
        float self = 0.0f, other = 0.0f;
        CHECK_EQ(core->compareRangeFit(r.in.beams, mcl, &self, &other), 4);
        CHECK(other > self);
        std::printf("      mcl wrong: chi2 self %.3f mcl %.1f\n",
                    static_cast<double>(self), static_cast<double>(other));

        for (int i = 0; i < 10 * r.cfg.mclResetCycles; i++) core->updateMcl(r.in);
        CHECK_EQ(core->relocateCount(), 0);
        CHECK(!core->consumeRelocation());
        CHECK_NEAR(core->state().x, 0.0f, 1e-6f);
        CHECK(core->mclRejectStreak() >= 10 * r.cfg.mclResetCycles);
    }
}

CHAD_TEST(ekfMcl, aTieInTheRangeEvidenceIsNotEnoughToRelocate) {
    // Truth halfway between the two hypotheses, so both explain the readings
    // equally badly. Snapping to MCL here is a coin flip dressed up as an
    // inference, and it costs the filter the 12.5 in it already had right.
    const Pose mcl{20.0f, -15.0f, 0.0f};
    RelocRig r(mcl, 10.0f, -7.5f);
    auto core = r.core();

    float self = 0.0f, other = 0.0f;
    CHECK_EQ(core->compareRangeFit(r.in.beams, mcl, &self, &other), 4);
    // Equal, because sigma is a function of the READING and so is the same for
    // both hypotheses: the comparison is purely between the two residuals.
    CHECK(self > 4.0f);              // both fit badly, so the absolute test passes
    CHECK(other > 4.0f);
    CHECK(self < other * r.cfg.mclRelocEvidenceRatio);
    std::printf("      tie: chi2 self %.1f mcl %.1f  ratio %.2f\n",
                static_cast<double>(self), static_cast<double>(other),
                static_cast<double>(self / other));

    for (int i = 0; i < 10 * r.cfg.mclResetCycles; i++) core->updateMcl(r.in);
    CHECK_EQ(core->relocateCount(), 0);
}

CHAD_TEST(ekfMcl, relocationNeedsTwoBeamsThatCanSeeTheDisplacement) {
    // Opposed mounts measure one axis twice. A displacement along the other one
    // is invisible to them: both hypotheses predict the same pair of ranges, so
    // there is no evidence and the ratio would be comparing two roundoff errors.
    EkfCore::Config cfg = baseConfig();
    std::array<Pose, EkfCore::SENSOR_COUNT> mounts{};
    mounts[0] = {0.0f, 0.0f, 0.0f};    // +X
    mounts[1] = {0.0f, 0.0f, PI};      // -X

    EkfCore core(cfg);
    core.setMounts(mounts);
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);

    EkfCore::Inputs in;
    in.mclValid = true;
    in.mclPose = {0.0f, -30.0f, 0.0f};   // pure y, which neither beam can see
    in.mclCovXX = in.mclCovYY = 1.0f;
    in.mclNeffFrac = 0.9f;
    in.beams[0] = beamAt(kWall);
    in.beams[1] = beamAt(kWall);

    float self = 0.0f, other = 0.0f;
    CHECK_EQ(core.compareRangeFit(in.beams, in.mclPose, &self, &other), 2);
    CHECK_NEAR(self, other, 1e-4f);
    for (int i = 0; i < 10 * cfg.mclResetCycles; i++) core.updateMcl(in);
    CHECK_EQ(core.relocateCount(), 0);

    // One beam is never enough either, whatever it says.
    EkfCore lone(cfg);
    lone.setMounts(mounts);
    lone.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);
    EkfCore::Inputs solo = in;
    solo.beams[1] = {};
    solo.mclPose = {30.0f, 0.0f, 0.0f};
    solo.beams[0] = beamAt(kWall - 30.0f);   // agrees with MCL, not the filter
    float s1 = 0.0f, o1 = 0.0f;
    CHECK_EQ(lone.compareRangeFit(solo.beams, solo.mclPose, &s1, &o1), 1);
    CHECK(s1 > o1 * cfg.mclRelocEvidenceRatio);   // the evidence does favour MCL
    for (int i = 0; i < 10 * cfg.mclResetCycles; i++) lone.updateMcl(solo);
    CHECK_EQ(lone.relocateCount(), 0);            // and one beam still cannot act
}

CHAD_TEST(ekfMcl, aBetterFitInsideTheNoiseIsNotEvidenceOfAnything) {
    // Two beams on the same wall, both reading 0.01 in long. MCL is 30 in away
    // along that wall -- which the beams cannot see at all -- plus 0.01 in
    // towards it, which explains the discrepancy perfectly. So MCL fits
    // infinitely better as a ratio, on a 1/200 sigma difference, and a ratio
    // test alone would fly the filter 30 in sideways on it. Chi-square per beam
    // is what says there was nothing to explain.
    EkfCore::Config cfg = baseConfig();
    std::array<Pose, EkfCore::SENSOR_COUNT> mounts{};
    mounts[0] = {0.0f, 0.0f, 0.0f};
    mounts[1] = {0.0f, 5.0f, 0.0f};   // same wall, 5 in along it

    EkfCore core(cfg);
    core.setMounts(mounts);
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);

    EkfCore::Inputs in;
    in.mclValid = true;
    in.mclPose = {-0.01f, -30.0f, 0.0f};
    in.mclCovXX = in.mclCovYY = 1.0f;
    in.mclNeffFrac = 0.9f;
    in.beams[0] = in.beams[1] = beamAt(kWall + 0.01f);

    float self = 0.0f, other = 0.0f;
    CHECK_EQ(core.compareRangeFit(in.beams, in.mclPose, &self, &other), 2);
    CHECK(self > other * cfg.mclRelocEvidenceRatio);   // the ratio test passes
    CHECK(self < 2.0f);                                // and the chi-square does not
    std::printf("      inside noise: chi2 self %.3g mcl %.3g\n",
                static_cast<double>(self), static_cast<double>(other));

    for (int i = 0; i < 10 * cfg.mclResetCycles; i++) core.updateMcl(in);
    CHECK_EQ(core.relocateCount(), 0);
    CHECK_NEAR(core.state().y, 0.0f, 1e-6f);
}

CHAD_TEST(ekfMcl, aBeamOnlyOneHypothesisCanPredictIsNotEvidence) {
    // MCL is 40 in behind the filter, far enough that the forward ray has no
    // wall inside maxRangeIn from there, so MCL cannot be asked what that beam
    // should read. Meanwhile something is standing 20 in in front of the robot,
    // which the filter CAN predict and gets a 75 sigma residual for. Charging
    // that to the filter alone hands MCL a 40 in relocation for free on a beam
    // it never answered. The three beams both poses can predict say the filter
    // is right.
    RelocRig r({-40.0f, 0.0f, 0.0f}, 0.0f, 0.0f);
    auto core = r.core();
    r.in.beams[0] = beamAt(20.0f);

    float self = 0.0f, other = 0.0f;
    CHECK_EQ(core->compareRangeFit(r.in.beams, r.in.mclPose, &self, &other), 3);
    CHECK_NEAR(self, 0.0f, 1e-3f);
    CHECK(other > 100.0f);
    std::printf("      one-sided beam: chi2 self %.3g mcl %.1f over %d beams\n",
                static_cast<double>(self), static_cast<double>(other), 3);

    for (int i = 0; i < 10 * r.cfg.mclResetCycles; i++) core->updateMcl(r.in);
    CHECK_EQ(core->relocateCount(), 0);
    CHECK_NEAR(core->state().x, 0.0f, 1e-6f);
}

CHAD_TEST(ekfMcl, withNoRangeEvidenceTheFilterHoldsItsGround) {
    // No distance sensors, so nothing can adjudicate. Holding is the honest
    // outcome: MCL without range readings is dead reckoning too, and there is
    // no reason to prefer its dead reckoning to the filter's.
    EkfCore::Config cfg = baseConfig();
    EkfCore core(cfg);
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);

    EkfCore::Inputs in;
    in.mclValid = true;
    in.mclPose = {20.0f, -15.0f, 0.0f};
    in.mclCovXX = in.mclCovYY = 1.0f;
    in.mclNeffFrac = 0.9f;

    for (int i = 0; i < 10 * cfg.mclResetCycles; i++) core.updateMcl(in);
    CHECK_EQ(core.relocateCount(), 0);
    CHECK(core.mclRejects() > 0u);
}

CHAD_TEST(ekfMcl, sigmaFloorStopsTheCloudClaimingCertainty) {
    EkfCore::Config cfg = baseConfig();
    EkfCore core(cfg);
    core.reset({0.0f, 0.0f, 0.0f}, 2.0f, 0.02f, 0);

    EkfCore::Inputs in;
    in.mclValid = true;
    in.mclPose = {1.0f, 0.0f, 0.0f};
    in.mclCovXX = in.mclCovYY = 0.0f;   // a cloud that has collapsed on itself
    in.mclNeffFrac = 0.9f;
    core.updateMcl(in);

    // The floor is the whole of R here, so the gain and the posterior are both
    // arithmetic: K = P/(P+floor^2) = 4/5.44 = 0.735, posterior = 4*1.44/5.44.
    // The old assertion was "covariance()[0] > 0.0f", which the 1e-9 diagonal
    // floor makes true for every reachable P -- deleting mclSigmaFloor entirely
    // would have passed it.
    const float floorVar = cfg.mclSigmaFloor * cfg.mclSigmaFloor;
    const float prior = 4.0f;
    const float gain = prior / (prior + floorVar);
    CHECK_NEAR(gain, 0.7353, 1e-3);
    CHECK_NEAR(core.state().x, gain, 2e-3);
    CHECK_NEAR(core.covariance()[0], prior * floorVar / (prior + floorVar), 2e-3);
    CHECK(core.covariance()[0] > 0.9f);
    // A zero-variance fix would take the whole step and the EKF would just be
    // MCL with extra steps.
    CHECK(core.state().x < 0.9f);
}

// ============================================================================
// Obstacles
// ============================================================================

CHAD_TEST(ekfObstacles, dropABeamThatRangedAPostInsteadOfTheWall) {
    EkfCore core(baseConfig());
    FourBeams rig;
    core.setMounts(rig.mounts);
    core.reset({0.0f, 0.0f, 0.0f}, 0.5f, 0.01f, 0);

    const RangeObstacle post{24.0f, 0.0f, 2.0f};
    core.setObstacles(&post, 1);

    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = beamAt(22.0f);   // the post surface, not the wall at 70.2
    core.updateRanges(b, 10);
    CHECK(core.beamStates()[0] == EkfCore::BeamState::Deadzone);
    CHECK_EQ(core.beamsUsed(), 0);
    // A deadzoned beam is not a gate rejection; conflating them would hide real
    // rejections behind a field full of posts.
    CHECK_EQ(core.rangeRejects(), 0u);
    CHECK_EQ(core.rangeRejectStreak(), 0);
    // But it IS counted, and it counts as blindness. A filter permanently
    // deadzoned used to report rangeRejects 0, rangeRejectStreak 0 and
    // beamsUsed 0 -- identical, from the outside, to a healthy one.
    CHECK_EQ(core.rangeDeadzoned(), 1u);
    CHECK_EQ(core.rangeBlindStreak(), 1);
    for (int i = 0; i < 24; i++) core.updateRanges(b, 20 + 10u * static_cast<std::uint32_t>(i));
    CHECK_EQ(core.rangeDeadzoned(), 25u);
    CHECK_EQ(core.rangeBlindStreak(), 25);
    CHECK_EQ(core.rangeRejects(), 0u);
    CHECK_EQ(core.rangeRejectStreak(), 0);

    core.setObstacles(nullptr, 0);
    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> clean{};
    clean[0] = beamAt(wallRangeForward(0.0f));
    core.updateRanges(clean, 400);
    CHECK(core.beamStates()[0] == EkfCore::BeamState::Used);
    // One good beam clears the streak, and the cumulative count survives it.
    CHECK_EQ(core.rangeBlindStreak(), 0);
    CHECK_EQ(core.rangeDeadzoned(), 25u);
}

CHAD_TEST(ekfObstacles, capacityIsBoundedNotWrapped) {
    EkfCore core(baseConfig());
    std::array<RangeObstacle, EkfCore::MAX_OBSTACLES + 8> many{};
    for (std::size_t i = 0; i < many.size(); i++) {
        many[i] = {200.0f + static_cast<float>(i), 0.0f, 1.0f};
    }
    core.setObstacles(many.data(), static_cast<int>(many.size()));
    // Off the field, so nothing should be deadzoned and nothing should crash.
    FourBeams rig;
    core.setMounts(rig.mounts);
    core.reset({0.0f, 0.0f, 0.0f}, 0.5f, 0.01f, 0);
    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = beamAt(wallRangeForward(0.0f));
    core.updateRanges(b, 10);
    CHECK(core.beamStates()[0] == EkfCore::BeamState::Used);
}

// ============================================================================
// Beam validity and grazing
// ============================================================================

CHAD_TEST(ekfBeams, classifiesEveryRejectionReason) {
    EkfCore::Config cfg = baseConfig();
    EkfCore core(cfg);
    std::array<Pose, EkfCore::SENSOR_COUNT> mounts{};
    mounts[0] = {0.0f, 0.0f, 0.0f};
    mounts[1] = {0.0f, 0.0f, 0.0f};
    mounts[2] = {0.0f, 0.0f, 0.0f};
    mounts[3] = {0.0f, 0.0f, degToRad(80.0f)};
    core.setMounts(mounts);
    // Close to the +X wall and looking along it, so beam 3 hits at 80 degrees
    // of incidence and the 1/cos in its Jacobian is not to be trusted.
    core.reset({65.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);

    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = {true, cfg.sentinelMm, kConf};             // no return
    b[1] = {true, cfg.minRangeMm - 1.0f, kConf};      // too close to mean anything
    b[2] = {true, 1500.0f, cfg.minConfidence - 1};    // below confidence
    b[3] = beamAt(50.0f);                             // grazing
    core.updateRanges(b, 10);

    CHECK(core.beamStates()[0] == EkfCore::BeamState::NoReturn);
    CHECK(core.beamStates()[1] == EkfCore::BeamState::NoReturn);
    CHECK(core.beamStates()[2] == EkfCore::BeamState::NoReturn);
    CHECK(core.beamStates()[3] == EkfCore::BeamState::Grazing);
    CHECK(core.beamStates()[4] == EkfCore::BeamState::Absent);
    CHECK_EQ(core.beamsUsed(), 0);
}

CHAD_TEST(ekfBeams, nonFiniteReadingsAreIgnoredWithoutTouchingTheState) {
    EkfCore core(baseConfig());
    FourBeams rig;
    core.setMounts(rig.mounts);
    core.reset({3.0f, -2.0f, 0.2f}, 1.0f, 0.01f, 0);
    const Pose before = core.state();

    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = {true, kNan, kConf};
    b[1] = {true, kInf, kConf};
    b[2] = {true, -kInf, kConf};
    core.updateRanges(b, 10);

    CHECK(core.beamStates()[0] == EkfCore::BeamState::NoReturn);
    CHECK(core.beamStates()[1] == EkfCore::BeamState::NoReturn);
    CHECK(core.beamStates()[2] == EkfCore::BeamState::NoReturn);
    CHECK_NEAR(core.state().x, before.x, 1e-9);
    CHECK_NEAR(core.state().y, before.y, 1e-9);
    CHECK_NEAR(core.state().theta, before.theta, 1e-9);
    CHECK(positiveDefinite(core.covariance()));
}

// ============================================================================
// Degenerate inputs
// ============================================================================

CHAD_TEST(ekfDegenerate, zeroMotionLeavesTheMeanAlone) {
    EkfCore core(baseConfig());
    core.reset({7.0f, -3.0f, 1.1f}, 1.0f, 0.02f, 0);
    core.predict({0.0f, 0.0f, 0.0f}, false, 10);
    CHECK_NEAR(core.state().x, 7.0f, 1e-6);
    CHECK_NEAR(core.state().y, -3.0f, 1e-6);
    CHECK_NEAR(core.state().theta, 1.1f, 1e-6);
    CHECK(positiveDefinite(core.covariance()));
}

CHAD_TEST(ekfDegenerate, nonFiniteOdometryIsTreatedAsNoMotion) {
    EkfCore core(baseConfig());
    core.reset({1.0f, 2.0f, 0.3f}, 1.0f, 0.02f, 0);
    core.predict({kNan, 0.5f, kInf}, false, 10);
    CHECK_FINITE(core.state().x);
    CHECK_FINITE(core.state().y);
    CHECK_FINITE(core.state().theta);
    CHECK_NEAR(core.state().theta, 0.3f, 1e-6);
    CHECK(positiveDefinite(core.covariance()));
}

CHAD_TEST(ekfDegenerate, singularInnovationCovarianceIsRefusedNotDivided) {
    // A real covariance, not diag(1e-9): with the old reset(0, 0) every entry of
    // P was already at the floor, so the trailing positive-definite check could
    // not fail whatever the refused paths did to it. Snapshot P and require it
    // back bit for bit.
    EkfCore core(baseConfig());
    core.reset({3.0f, -2.0f, 0.4f}, 2.0f, 0.05f, 0);
    core.predict({1.5f, 0.0f, 0.08f}, false, 10);
    CHECK(positiveDefinite(core.covariance()));
    CHECK(core.covariance()[1] != 0.0f);   // real correlations to lose

    float snapshot[9];
    std::memcpy(snapshot, core.covariance(), sizeof snapshot);
    const Pose before = core.state();

    const float zeroH[3] = {0.0f, 0.0f, 0.0f};
    CHECK(!core.fuseScalar(zeroH, 1.0f, 0.0f, 0.0f));

    const float H[3] = {1.0f, 0.0f, 0.0f};
    CHECK(!core.fuseScalar(H, kNan, 1.0f, 0.0f));
    CHECK(!core.fuseScalar(H, 1.0f, kNan, 0.0f));
    CHECK(!core.fuseScalar(H, 1.0f, -1.0f, 0.0f));
    const float badH[3] = {kInf, 0.0f, 0.0f};
    CHECK(!core.fuseScalar(badH, 1.0f, 1.0f, 0.0f));
    const float nanH[3] = {0.0f, kNan, 0.0f};
    CHECK(!core.fuseScalar(nanH, 1.0f, 1.0f, 0.0f));

    CHECK_NEAR(core.state().x, before.x, 1e-9);
    CHECK_NEAR(core.state().y, before.y, 1e-9);
    CHECK_NEAR(core.state().theta, before.theta, 1e-9);
    // Untouched, not merely still positive-definite.
    for (int i = 0; i < 9; i++) CHECK_NEAR(core.covariance()[i], snapshot[i], 0.0);
    CHECK(positiveDefinite(core.covariance()));
    CHECK_EQ(core.covarianceRepairs(), 0u);
}

CHAD_TEST(ekfDegenerate, nonFiniteMclFixIsRefused) {
    EkfCore core(baseConfig());
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);
    const Pose before = core.state();

    EkfCore::Inputs in;
    in.mclValid = true;
    in.mclNeffFrac = 0.9f;
    in.mclPose = {kNan, 0.0f, 0.0f};
    in.mclCovXX = in.mclCovYY = 1.0f;
    core.updateMcl(in);
    CHECK_NEAR(core.state().x, before.x, 1e-9);

    in.mclPose = {1.0f, 0.0f, 0.0f};
    in.mclCovXX = kNan;
    core.updateMcl(in);
    CHECK_NEAR(core.state().x, before.x, 1e-9);
    CHECK(positiveDefinite(core.covariance()));
}

CHAD_TEST(ekfDegenerate, nonFiniteHeadingIsRefused) {
    EkfCore core(baseConfig());
    core.reset({0.0f, 0.0f, 0.5f}, 1.0f, 0.02f, 0);
    core.updateHeading(kNan, 10);
    CHECK_NEAR(core.state().theta, 0.5f, 1e-9);
    core.updateHeading(kInf, 20);
    CHECK_NEAR(core.state().theta, 0.5f, 1e-9);
}

CHAD_TEST(ekfDegenerate, resetClearsStateHistoryAndDiagnostics) {
    EkfCore core(baseConfig());
    FourBeams rig;
    core.setMounts(rig.mounts);
    core.reset({0.0f, 0.0f, 0.0f}, 0.05f, 0.001f, 0);
    core.updateRanges(rig.readings(30.0f, 25.0f), 10);
    CHECK(core.rangeRejects() > 0u);
    CHECK(core.rangeRejectStreak() > 0);

    core.reset({-10.0f, 5.0f, -0.4f}, 2.0f, 0.03f, 100);
    CHECK_NEAR(core.state().x, -10.0f, 1e-6);
    CHECK_NEAR(core.state().theta, -0.4f, 1e-6);
    CHECK_NEAR(core.covariance()[0], 4.0f, 1e-5);
    CHECK_NEAR(core.covariance()[1], 0.0f, 1e-9);
    CHECK_EQ(core.rangeRejectStreak(), 0);
    CHECK_EQ(core.beamsUsed(), 0);
    CHECK(core.beamStates()[0] == EkfCore::BeamState::Absent);
}

// ============================================================================
// Whole cycle
// ============================================================================

CHAD_TEST(ekfStep, runsEveryEnabledFamilyInOneCall) {
    EkfCore::Config cfg = baseConfig();
    EkfCore core(cfg);
    FourBeams rig;
    core.setMounts(rig.mounts);
    core.reset({0.0f, 0.0f, 0.0f}, 2.0f, 0.05f, 0);

    EkfCore::Inputs in;
    in.headingValid = true;
    in.headingRad = 0.0f;
    in.beams = rig.readings(0.0f, 0.0f);
    in.mclValid = true;
    in.mclPose = {0.2f, 0.1f, 0.0f};
    in.mclCovXX = in.mclCovYY = 1.0f;
    in.mclNeffFrac = 0.9f;

    std::uint32_t t = 0;
    for (int i = 0; i < 50; i++) {
        t += 10;
        core.step(t, in);
    }
    CHECK_NEAR(core.state().x, 0.1f, 0.3f);
    CHECK_EQ(core.beamsUsed(), 4);
    CHECK(positiveDefinite(core.covariance()));
    CHECK_EQ(core.covarianceRepairs(), 0u);
}

CHAD_TEST(ekfStep, disabledFamiliesAreNotConsulted) {
    EkfCore::Config cfg = baseConfig();
    cfg.useRangeUpdates = false;
    cfg.useImuHeading = false;
    cfg.useMclUpdates = false;
    EkfCore core(cfg);
    FourBeams rig;
    core.setMounts(rig.mounts);
    core.reset({5.0f, 5.0f, 0.0f}, 1.0f, 0.02f, 0);

    EkfCore::Inputs in;
    in.headingValid = true;
    in.headingRad = 1.5f;
    in.beams = rig.readings(0.0f, 0.0f);
    in.mclValid = true;
    in.mclPose = {0.0f, 0.0f, 0.0f};
    in.mclCovXX = in.mclCovYY = 1.0f;
    in.mclNeffFrac = 0.9f;

    core.step(10, in);
    CHECK_NEAR(core.state().x, 5.0f, 1e-6);
    CHECK_NEAR(core.state().y, 5.0f, 1e-6);
    CHECK_NEAR(core.state().theta, 0.0f, 1e-6);
    CHECK_EQ(core.beamsUsed(), 0);
}

CHAD_TEST(ekfStep, aHeldHeadingIsNotFusedAsAMeasurement) {
    EkfCore::Config cfg = baseConfig();
    EkfCore core(cfg);
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);

    EkfCore::Inputs in;
    in.headingValid = false;
    in.headingRad = 1.2f;   // the last good value, repeated by a dead IMU
    core.step(10, in);
    CHECK_NEAR(core.state().theta, 0.0f, 1e-6);

    // The old assertion here was "covariance()[8] > 0.02*0.02", satisfied by
    // 0.04% after one tick and satisfied for ever afterwards however wrong the
    // heading got. What has to grow is the UNMEASURED heading variance, at
    // thetaUnmeasuredVarPerSec.
    const float perTick = cfg.thetaUnmeasuredVarPerSec * 0.001f * cfg.updatePeriodMs;
    CHECK_NEAR(core.covariance()[8] - 0.02f * 0.02f, perTick, 1e-5);

    // Two seconds of outage while the robot turns 90 degrees: the sigma has to be
    // the same order as the error it is hiding, not 70x smaller.
    std::uint32_t t = 10;
    for (int i = 0; i < 199; i++) {
        t += 10;
        core.step(t, in);
    }
    const float sigmaTheta = core.uncertainty().sigmaTheta;
    CHECK_NEAR(sigmaTheta, std::sqrt(200.0f * perTick), 0.02f);
    CHECK(sigmaTheta * RAD_TO_DEG > 60.0f);
    CHECK(radToDeg(HALF_PI) / (sigmaTheta * RAD_TO_DEG) < 1.5f);

    // A live IMU is charged the motion terms and nothing more, so the growth term
    // cannot be hiding a filter that is simply loose.
    EkfCore live(cfg);
    live.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);
    EkfCore::Inputs good;
    good.headingValid = true;
    good.headingRad = 0.0f;
    t = 0;
    for (int i = 0; i < 200; i++) {
        t += 10;
        live.step(t, good);
    }
    CHECK(live.uncertainty().sigmaTheta * RAD_TO_DEG < 1.0f);
}

// ============================================================================
// The range model itself, against the field box
// ============================================================================

CHAD_TEST(ekfModel, predictedRangeMatchesTheFieldGeometry) {
    EkfCore core(baseConfig());

    struct Case { Pose pose; Pose mount; };
    const Case cases[] = {
        {{0.0f, 0.0f, 0.0f},      {0.0f, 0.0f, 0.0f}},
        {{-20.0f, 10.0f, 0.0f},   {5.0f, -2.0f, 0.0f}},
        {{12.0f, -5.0f, 0.3f},    {4.0f, 1.0f, -0.3f}},
    };
    for (const Case& c : cases) {
        // Sensor origin and ray angle worked out here, then the +X wall
        // intersection from plane geometry rather than from the model.
        const float ct = std::cos(c.pose.theta), st = std::sin(c.pose.theta);
        const float sx = c.pose.x + c.mount.x * ct - c.mount.y * st;
        const float sy = c.pose.y + c.mount.x * st + c.mount.y * ct;
        const float psi = c.pose.theta + c.mount.theta;
        const float expected = (kWall - sx) / std::cos(psi);
        const float yAt = sy + std::tan(psi) * (kWall - sx);
        CHECK(std::fabs(yAt) < kWall);

        const EkfCore::RangePrediction rp = core.predictRange(c.pose, c.mount);
        CHECK(rp.valid());
        CHECK_NEAR(rp.range, expected, 2e-3);
        CHECK_NEAR(rp.sx, sx, 1e-4);
        CHECK_NEAR(rp.sy, sy, 1e-4);
        CHECK_NEAR(rp.nx, 1.0f, 1e-6);
    }
}

CHAD_TEST(ekfGate, grazingBeamsDoNotLookLikeRejections) {
    EkfCore core(baseConfig());
    std::array<Pose, EkfCore::SENSOR_COUNT> mounts{};
    mounts[0] = {0.0f, 0.0f, degToRad(80.0f)};
    core.setMounts(mounts);
    core.reset({65.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);

    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = beamAt(30.0f);
    for (int i = 0; i < 10; i++) core.updateRanges(b, static_cast<std::uint32_t>(10 * i));

    CHECK(core.beamStates()[0] == EkfCore::BeamState::Grazing);
    // An uninformative beam is not a deaf filter, and conflating the two makes
    // the streak useless as a signal.
    CHECK_EQ(core.rangeRejects(), 0u);
    CHECK_EQ(core.rangeRejectStreak(), 0);
    // It is still counted, and a robot whose only sensor grazes every cycle is
    // blind whatever the reason.
    CHECK_EQ(core.rangeGrazed(), 10u);
    CHECK_EQ(core.rangeBlindStreak(), 10);
    CHECK_EQ(core.rangeNoReturns(), 0u);
    CHECK_EQ(core.rangeDeadzoned(), 0u);
}

CHAD_TEST(ekfDegenerate, resetKeepsTheCumulativeHealthCounters) {
    EkfCore core(baseConfig());
    FourBeams rig;
    core.setMounts(rig.mounts);
    core.reset({0.0f, 0.0f, 0.0f}, 0.05f, 0.001f, 0);
    core.updateRanges(rig.readings(30.0f, 25.0f), 10);
    const std::uint32_t rejects = core.rangeRejects();
    CHECK(rejects > 0u);

    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 100);
    // Cumulative counters describe the run, not the pose: a relocation must not
    // be able to hide the rejections that led to it.
    CHECK_EQ(core.rangeRejects(), rejects);
}

// ============================================================================
// Range measurement variance from the reported confidence
// ============================================================================

CHAD_TEST(ekfRangeSigma, confidenceInflatesTheMeasuredSigmaAndNeverShrinksIt) {
    // This line had no test at all: every other case in the suite plants
    // conf == 32, where the scale factor is exactly one, so deleting the line
    // would have failed nothing. Two things have to hold. The factor must never
    // take sigma below rangeSigmaBase, which is bench-measured [robot] -- the
    // old 32/conf form halved it at conf 63 and claimed 0.86 mm of V5
    // time-of-flight accuracy. And a faulted port read must not reach the
    // divisor at all: PROS_ERR gave a 2.2e-8 inch sigma, so anything outside
    // 0..CONFIDENCE_MAX is refused rather than believed.
    EkfCore::Config cfg = baseConfig();
    cfg.rangeSigmaProp = 0.0f;
    cfg.rangeSigmaBase = 0.5f;
    cfg.rangeGateChi2 = 0.0f;

    auto posterior = [&cfg](int conf) {
        EkfCore core(cfg);
        std::array<Pose, EkfCore::SENSOR_COUNT> mounts{};
        core.setMounts(mounts);
        core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);
        std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
        b[0] = {true, mmFor(wallRangeForward(4.0f)), conf};
        core.updateRanges(b, 10);
        return core;
    };

    const EkfCore nominal = posterior(EkfCore::CONFIDENCE_NOMINAL);
    // R = base^2 = 0.25 against a prior of 1, so K = 0.8 on a 4 inch innovation.
    CHECK_NEAR(nominal.state().x, 4.0f * 1.0f / 1.25f, 0.01f);
    CHECK_NEAR(nominal.covariance()[0], 0.25f / 1.25f, 0.01f);

    // Flat at and above nominal, across the whole legal range.
    for (int conf : {32, 40, 63}) {
        const EkfCore c = posterior(conf);
        CHECK(c.beamStates()[0] == EkfCore::BeamState::Used);
        CHECK_NEAR(c.covariance()[0], nominal.covariance()[0], 1e-6);
        CHECK_NEAR(c.state().x, nominal.state().x, 1e-5);
        CHECK_EQ(c.covarianceRepairs(), 0u);
    }
    // Outside it there is no confidence to scale by: a faulted port read is
    // refused and counted, not clamped to the top of the scale.
    for (int conf : {EkfCore::CONFIDENCE_MAX + 1, 200, -1, 2147483647}) {
        const EkfCore c = posterior(conf);
        CHECK(c.beamStates()[0] == EkfCore::BeamState::NoReturn);
        CHECK_NEAR(c.state().x, 0.0f, 1e-9);
        CHECK_EQ(c.rangeNoReturns(), 1u);
        CHECK_EQ(c.covarianceRepairs(), 0u);
    }
    // Monotone below it: sigma scales by CONFIDENCE_NOMINAL/conf.
    float prevSigma = 0.0f;
    for (int conf : {32, 20, 16, 10}) {
        const EkfCore c = posterior(conf);
        const float R = cfg.rangeSigmaBase * static_cast<float>(EkfCore::CONFIDENCE_NOMINAL)
                      / static_cast<float>(conf);
        // Posterior variance R*P/(R+P) with P == 1, solved back for R.
        const float got = c.covariance()[0];
        CHECK_NEAR(got, R * R / (R * R + 1.0f), 0.01f);
        CHECK(got > prevSigma);
        prevSigma = got;
    }

    // A short reading is exempt from the scaling by design (the 200 mm branch),
    // so a confidence report cannot inflate a close range out of the gate.
    EkfCore close(cfg);
    std::array<Pose, EkfCore::SENSOR_COUNT> mounts{};
    close.setMounts(mounts);
    close.reset({kWall - 5.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);
    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = {true, mmFor(4.0f), 1};   // 101 mm, below the 200 mm branch
    close.updateRanges(b, 10);
    CHECK(close.beamStates()[0] == EkfCore::BeamState::Used);
    CHECK_NEAR(close.covariance()[0], 0.25f / 1.25f, 0.01f);
}

CHAD_TEST(ekfBeams, everyDiscardReasonHasItsOwnCounter) {
    // The class promises "all of it is counted". Before, `returns++` sat after
    // the no-return, grazing and deadzone `continue`s, so none of the three
    // reached any counter and a filter blinded by them reported rangeRejects 0,
    // rangeRejectStreak 0 and beamsUsed 0 -- indistinguishable from healthy.
    EkfCore::Config cfg = baseConfig();
    EkfCore core(cfg);
    std::array<Pose, EkfCore::SENSOR_COUNT> mounts{};
    mounts[3] = {0.0f, 0.0f, degToRad(80.0f)};
    core.setMounts(mounts);
    core.reset({65.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);

    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = {true, cfg.sentinelMm, kConf};             // no return
    b[1] = {true, cfg.minRangeMm - 1.0f, kConf};      // too close to mean anything
    b[2] = {true, 1500.0f, cfg.minConfidence - 1};    // below confidence
    b[3] = beamAt(50.0f);                             // grazing
    core.updateRanges(b, 10);

    CHECK_EQ(core.rangeNoReturns(), 3u);
    CHECK_EQ(core.rangeGrazed(), 1u);
    CHECK_EQ(core.rangeDeadzoned(), 0u);
    CHECK_EQ(core.rangeRejects(), 0u);
    CHECK_EQ(core.beamsUsed(), 0);
    // Four sensors fitted, nothing folded in: blind, and it says so.
    CHECK_EQ(core.rangeBlindStreak(), 1);
    CHECK_EQ(core.rangeRejectStreak(), 0);

    // With no sensor fitted at all there is nothing to be blind about, so the
    // streak must not run away on a robot with no distance sensors.
    EkfCore bare(cfg);
    bare.setMounts(mounts);
    bare.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);
    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> none{};
    for (int i = 0; i < 20; i++) bare.updateRanges(none, static_cast<std::uint32_t>(10 * i));
    CHECK_EQ(bare.rangeBlindStreak(), 0);
    CHECK_EQ(bare.rangeNoReturns(), 0u);

    // Cumulative counters describe the run, so a reset must not hide them, and
    // the streaks must clear with the pose.
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 100);
    CHECK_EQ(core.rangeNoReturns(), 3u);
    CHECK_EQ(core.rangeGrazed(), 1u);
    CHECK_EQ(core.rangeBlindStreak(), 0);
}

// ===========================================================================
// The relocation evidence ratio, as a pair of rates
// ===========================================================================
//
// A ratio has to be judged on both rates at once: one so high the filter never
// relocates scores a perfect zero on false relocations, and 1.0 scores a perfect
// zero on missed ones. Both arms here use the SAME (stale pose, other pose,
// readings-from-truth) triple with the two poses' roles swapped, so the evidence
// is identical between them and only the right answer differs.

namespace {

/// Range to the VRC perimeter along a ray, in double, written from the geometry
/// rather than taken from EkfCore -- generating a trial's truth reading with the
/// filter's own cast would make every rate below a check of the code against
/// itself.
double refWallRange(double sx, double sy, double dx, double dy) {
    const double half = static_cast<double>(Field::FIELD_HALF_WALL);
    double best = 1.0e9;
    for (int s = -1; s <= 1; s += 2) {
        const double w = static_cast<double>(s) * half;
        if (std::fabs(dx) > 1e-12) {
            const double t = (w - sx) / dx;
            const double yy = sy + t * dy;
            if (t >= 0.0 && t < best && yy >= -half && yy <= half) best = t;
        }
        if (std::fabs(dy) > 1e-12) {
            const double t = (w - sy) / dy;
            const double xx = sx + t * dx;
            if (t >= 0.0 && t < best && xx >= -half && xx <= half) best = t;
        }
    }
    return best;
}

/// config.cpp's four mounts, not an idealised ring at the robot centre. Where a
/// sensor sits decides which beams can see a given displacement, and that is
/// what the ratio adjudicates.
std::array<Pose, EkfCore::SENSOR_COUNT> shippedMounts() {
    std::array<Pose, EkfCore::SENSOR_COUNT> m{};
    m[0] = { 1.00f, -3.75f, 0.0f};
    m[1] = { 0.50f,  6.25f, HALF_PI};
    m[2] = {-0.50f,  5.00f, PI};
    m[3] = {-0.75f, -6.75f, 3.0f * HALF_PI};
    return m;
}

/// Which noise the trial set runs at. `Model` draws at the sigma the filter
/// itself assumes, so chi-square per beam averages 1 and the absolute
/// `chi2Self > scored` precondition is genuinely marginal; `Bench` draws at
/// 5 mm, the figure the simulator uses, where a correct hypothesis spends almost
/// nothing. The two bracket the real sensor, so a ratio has to survive both.
enum class Noise { Model, Bench };

float trialSigma(Noise mode, double rangeIn) {
    if (mode == Noise::Bench) return 0.197f;   // 5 mm
    const EkfCore::Config c;
    return c.rangeSigmaBase + c.rangeSigmaProp * static_cast<float>(rangeIn);
}

/// Readings a robot really at `truth` produces, with the V5's ~2 m ceiling
/// reported as the no-return sentinel rather than clamped to it.
std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> senseFor(
        const Pose& truth, const std::array<Pose, EkfCore::SENSOR_COUNT>& mounts,
        Noise mode, std::mt19937& gen) {
    std::normal_distribution<float> unit(0.0f, 1.0f);
    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> out{};
    const double ct = std::cos(static_cast<double>(truth.theta));
    const double stt = std::sin(static_cast<double>(truth.theta));
    for (int i = 0; i < 4; i++) {
        const std::size_t idx = static_cast<std::size_t>(i);
        const Pose& m = mounts[idx];
        const double sx = static_cast<double>(truth.x) +
                          static_cast<double>(m.x) * ct - static_cast<double>(m.y) * stt;
        const double sy = static_cast<double>(truth.y) +
                          static_cast<double>(m.x) * stt + static_cast<double>(m.y) * ct;
        const double dx = std::cos(static_cast<double>(truth.theta + m.theta));
        const double dy = std::sin(static_cast<double>(truth.theta + m.theta));
        double r = refWallRange(sx, sy, dx, dy);
        r += static_cast<double>(unit(gen)) * static_cast<double>(trialSigma(mode, r));
        out[idx] = (r > 78.7 || r <= 0.5)
                       ? EkfCore::Beam{true, 9999.0f, kConf}
                       : EkfCore::Beam{true, static_cast<float>(r * INCH_TO_MM), kConf};
    }
    return out;
}

struct RelocRates {
    int kidnaps{0}, missed{0};
    int steady{0}, falsePos{0};
    double missPct() const {
        return kidnaps ? 100.0 * missed / kidnaps : 0.0;
    }
    double falsePct() const {
        return steady ? 100.0 * falsePos / steady : 0.0;
    }
};

/// How long the rule is given to fire, past the mclResetCycles the streak needs.
/// Fresh noise every cycle, because a real filter gets a new reading every tick
/// and retries the comparison on it -- so a marginal case is decided by the
/// luckiest of these draws, not by one.
constexpr int kRelocRetries = 20;

/// One arm. `stale` is where the filter sits, `other` is what MCL insists on,
/// and the readings always come from `truth`. True if it relocated.
bool relocatesFrom(const EkfCore::Config& cfg,
                   const std::array<Pose, EkfCore::SENSOR_COUNT>& mounts,
                   const Pose& stale, const Pose& other, const Pose& truth,
                   Noise mode, std::mt19937& gen) {
    EkfCore core(cfg);
    core.setMounts(mounts);
    core.reset(stale, 1.0f, 0.02f, 0);
    EkfCore::Inputs in;
    in.mclValid = true;
    in.mclPose = other;
    in.mclCovXX = in.mclCovYY = 1.0f;
    in.mclNeffFrac = 0.9f;
    for (int c = 0; c < cfg.mclResetCycles + kRelocRetries; c++) {
        in.beams = senseFor(truth, mounts, mode, gen);
        core.updateMcl(in);
        if (core.relocateCount() > 0) return true;
    }
    return false;
}

/// The whole trial set at one ratio: a kidnap of `mag` inches in each of eight
/// directions, from a grid of positions and headings.
RelocRates relocRates(float ratio, Noise mode, float mag, std::uint32_t seed) {
    EkfCore::Config cfg = baseConfig();
    cfg.mclRelocEvidenceRatio = ratio;
    const auto mounts = shippedMounts();
    const float xs[] = {-45.0f, -22.0f, 0.0f, 22.0f, 45.0f};
    const float ths[] = {0.0f, 0.5f, 1.0f, 1.6f, 2.4f, -0.8f};
    RelocRates out;
    std::mt19937 gen(seed);
    for (float x : xs) {
        for (float y : xs) {
            for (float th : ths) {
                for (int d = 0; d < 8; d++) {
                    const float a = static_cast<float>(d) * PI / 4.0f;
                    const Pose from{x, y, th};
                    const Pose to{x + mag * std::cos(a), y + mag * std::sin(a), th};
                    if (std::fabs(to.x) > 50.0f || std::fabs(to.y) > 50.0f) continue;
                    // Kidnapped: the robot is at `to`, MCL found it, the filter
                    // is still at `from`. Relocating is correct.
                    out.kidnaps++;
                    if (!relocatesFrom(cfg, mounts, from, to, to, mode, gen)) out.missed++;
                    // The same evidence with the roles swapped: the robot is at
                    // `to`, the filter has it right, MCL has gone to `from`.
                    // Relocating is wrong.
                    out.steady++;
                    if (relocatesFrom(cfg, mounts, to, from, to, mode, gen)) out.falsePos++;
                }
            }
        }
    }
    return out;
}

}  // namespace

CHAD_TEST(ekfReloc, theEvidenceRatioIsSweptAgainstBothRatesAtOnce) {
    // Ratio 1.0 accepts any margin at all, so the misses it still records are
    // the ones no ratio can fix: fewer than two beams both hypotheses can
    // answer, or a displacement the answering beams cannot see. Every higher row
    // is to be read against that floor, not against zero.
    const float ratios[] = {1.0f, 2.0f, 3.0f, 4.0f, 6.0f, 8.0f, 12.0f,
                            16.0f, 25.0f, 40.0f, 1.0e6f};
    for (float mag : {24.0f, 12.0f}) {
        for (Noise mode : {Noise::Bench, Noise::Model}) {
            std::printf("      %s noise, %.0f in displacement\n",
                        mode == Noise::Bench ? "5 mm bench" : "model-sigma",
                        static_cast<double>(mag));
            std::printf("        ratio        missed            false\n");
            for (float r : ratios) {
                const RelocRates s = relocRates(r, mode, mag, 0xBEEF1234u);
                std::printf("        %-9.4g %4d/%-4d %5.1f%%   %4d/%-4d %5.1f%%\n",
                            static_cast<double>(r), s.missed, s.kidnaps, s.missPct(),
                            s.falsePos, s.steady, s.falsePct());
            }
        }
    }

    // A second seed on the rows the choice is actually between: the difference
    // there is a handful of trials in 864 and one seed cannot carry it.
    std::printf("      model-sigma, 24 in, second seed\n");
    for (float r : {4.0f, 8.0f, 12.0f, 25.0f}) {
        const RelocRates s = relocRates(r, Noise::Model, 24.0f, 0x5EED0002u);
        std::printf("        ratio %-5.4g missed %4d/%-4d %5.1f%%   false %4d/%-4d %5.1f%%\n",
                    static_cast<double>(r), s.missed, s.kidnaps, s.missPct(),
                    s.falsePos, s.steady, s.falsePct());
    }

    // Both directions, so a rule that never fires cannot pass, and neither can
    // one that always does.
    const EkfCore::Config def;
    const RelocRates shipped = relocRates(def.mclRelocEvidenceRatio, Noise::Model,
                                          24.0f, 0xBEEF1234u);
    const RelocRates permissive = relocRates(1.0f, Noise::Model, 24.0f, 0xBEEF1234u);
    const RelocRates refusing = relocRates(1.0e6f, Noise::Model, 24.0f, 0xBEEF1234u);

    // Not every trial, and that is not a rounding slip. The test is a RATIO, so
    // an alternative that fits to float zero beats any finite multiplier, and
    // about one trial in 864 draws noise small enough for that. Relocating there
    // is the right answer anyway; the point is that no ratio, however large, is
    // the same thing as refusing to relocate.
    CHECK(refusing.missed > refusing.kidnaps - 5);
    CHECK_EQ(refusing.falsePos, 0);   // and a refusal scores perfectly here
    CHECK(permissive.falsePos > shipped.falsePos);
    CHECK(shipped.missed < refusing.missed);
    // The floor above, as a number: most of what the shipped ratio misses is
    // geometry rather than the threshold.
    CHECK(shipped.missed < permissive.missed + permissive.kidnaps / 40);
}
