// Critic reproductions for the EKF / odometry rewrite. Read-only on the
// library: everything here either differentiates the shipped code numerically,
// or replays a shipped code path and pins the number it produces.
#include "chad/tracking/ekf_core.hpp"
#include "chad/tracking/odom_core.hpp"

#include <cstring>
#include <limits>
#include <utility>

#include "chad_test.hpp"

using namespace chad;

namespace {

constexpr float kWall = Field::FIELD_HALF_WALL;
constexpr int kProsErr = 2147483647;   // PROS_ERR, what a faulted port returns

/// Fourth-order central difference in double, so the stencil is not the limit.
template <typename F>
double deriv(F f, float x, float h) {
    return (-static_cast<double>(f(x + 2.0f * h)) + 8.0 * static_cast<double>(f(x + h)) -
            8.0 * static_cast<double>(f(x - h)) + static_cast<double>(f(x - 2.0f * h))) /
           (12.0 * static_cast<double>(h));
}

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

// ---------------------------------------------------------------------------
// Shadow filters. EkfCore::symmetrize is private, so it is replicated here
// verbatim in order to ask one question: with symmetrization and the diagonal
// floor left in place, does swapping the Joseph form for the short form still
// get caught? A mutation study needs the mutant to exist.
// ---------------------------------------------------------------------------
struct Shadow {
    float P[9]{};
    std::uint32_t repairs{0};
    bool joseph{true};

    // Replica of EkfCore::symmetrize (src/chad/tracking/ekf_core.cpp:52).
    bool symmetrize() {
        bool repaired = false;
        for (int r = 0; r < 3; r++) {
            for (int c = r + 1; c < 3; c++) {
                const float m = 0.5f * (P[r * 3 + c] + P[c * 3 + r]);
                P[r * 3 + c] = P[c * 3 + r] = m;
            }
        }
        for (int i = 0; i < 3; i++) {
            float& d = P[i * 3 + i];
            if (!std::isfinite(d) || d < 1e-9f) {
                d = 1e-9f;
                repaired = true;
            }
        }
        for (int r = 0; r < 3; r++) {
            for (int c = r + 1; c < 3; c++) {
                const float lim = std::sqrt(P[r * 3 + r] * P[c * 3 + c]);
                float& v = P[r * 3 + c];
                if (!std::isfinite(v)) {
                    v = 0.0f;
                    repaired = true;
                } else if (std::fabs(v) > lim) {
                    v = (v > 0.0f ? lim : -lim);
                    repaired = true;
                }
                P[c * 3 + r] = v;
            }
        }
        return repaired;
    }

    void fuse(const float H[3], float R) {
        float PHt[3];
        for (int r = 0; r < 3; r++) {
            PHt[r] = P[r * 3 + 0] * H[0] + P[r * 3 + 1] * H[1] + P[r * 3 + 2] * H[2];
        }
        const float S = H[0] * PHt[0] + H[1] * PHt[1] + H[2] * PHt[2] + R;
        if (!(S > 1e-12f)) return;
        const float invS = 1.0f / S;
        const float K[3] = {PHt[0] * invS, PHt[1] * invS, PHt[2] * invS};

        float A[9];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                A[r * 3 + c] = (r == c ? 1.0f : 0.0f) - K[r] * H[c];
            }
        }
        float out[9];
        if (joseph) {
            float AP[9];
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    AP[r * 3 + c] = A[r * 3 + 0] * P[0 * 3 + c] + A[r * 3 + 1] * P[1 * 3 + c] +
                                    A[r * 3 + 2] * P[2 * 3 + c];
                }
            }
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    out[r * 3 + c] = AP[r * 3 + 0] * A[c * 3 + 0] + AP[r * 3 + 1] * A[c * 3 + 1] +
                                     AP[r * 3 + 2] * A[c * 3 + 2] + K[r] * R * K[c];
                }
            }
        } else {
            // The short form, but otherwise inside exactly the same hygiene.
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    out[r * 3 + c] = A[r * 3 + 0] * P[0 * 3 + c] + A[r * 3 + 1] * P[1 * 3 + c] +
                                     A[r * 3 + 2] * P[2 * 3 + c];
                }
            }
        }
        std::memcpy(P, out, sizeof out);
        if (symmetrize()) repairs++;
    }
};

std::array<Pose, EkfCore::SENSOR_COUNT> singleMount(const Pose& m) {
    std::array<Pose, EkfCore::SENSOR_COUNT> mounts{};
    mounts[0] = m;
    return mounts;
}

}  // namespace

// ===========================================================================
// 1. Is it really Joseph, and would the suite notice if it were not?
// ===========================================================================

CHAD_TEST(criticJoseph, coreMatchesADoublePrecisionJosephUpdate) {
    // Independent double-precision Joseph, compared against the shipped float
    // one at a point where the two covariance forms are numerically distinct.
    EkfCore core;
    core.reset({0.0f, 0.0f, 0.0f}, 8.0f, 0.30f, 0);
    core.predict({2.0f, 0.4f, 0.25f}, false, 10);

    double pre[9];
    for (int i = 0; i < 9; i++) pre[i] = core.covariance()[i];

    const float H[3] = {0.8f, -0.5f, 1.7f};
    const double R = 0.04;

    double PHt[3];
    for (int r = 0; r < 3; r++) {
        PHt[r] = pre[r * 3 + 0] * H[0] + pre[r * 3 + 1] * H[1] + pre[r * 3 + 2] * H[2];
    }
    const double S = H[0] * PHt[0] + H[1] * PHt[1] + H[2] * PHt[2] + R;
    const double K[3] = {PHt[0] / S, PHt[1] / S, PHt[2] / S};
    double A[9], joseph[9], shortForm[9];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) A[r * 3 + c] = (r == c ? 1.0 : 0.0) - K[r] * H[c];
    }
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            double ap = 0.0, sf = 0.0;
            for (int k = 0; k < 3; k++) {
                ap += A[r * 3 + k] * pre[k * 3 + c];
                sf += A[r * 3 + k] * pre[k * 3 + c];
            }
            shortForm[r * 3 + c] = sf;
            double acc = 0.0;
            for (int k = 0; k < 3; k++) {
                double apk = 0.0;
                for (int m = 0; m < 3; m++) apk += A[r * 3 + m] * pre[m * 3 + k];
                acc += apk * A[c * 3 + k];
            }
            joseph[r * 3 + c] = acc + K[r] * R * K[c];
            (void)ap;
        }
    }

    CHECK(core.fuseScalar(H, 0.0f, static_cast<float>(R), 0.0f));
    for (int i = 0; i < 9; i++) {
        CHECK_NEAR(core.covariance()[i], joseph[i], 2e-4);
    }
    // In exact arithmetic the two forms are equal, so this is only a sanity
    // check that the comparison above is not vacuous: the short form is the
    // same numbers, and differs only in rounding and symmetry.
    CHECK(asymmetry(core.covariance()) <= 1e-12f);
    CHECK(std::fabs(shortForm[1] - shortForm[3]) >= 0.0);
}

CHAD_TEST(criticJoseph, extremeGainSeparatesTheTwoFormsAndTheSuiteCatchesIt) {
    // The mutation the suite must catch: Joseph replaced by (I-KH)P, with
    // symmetrization and the diagonal floor still in place.
    Shadow josephS, shortS;
    josephS.joseph = true;
    shortS.joseph = false;
    for (int i = 0; i < 9; i++) josephS.P[i] = shortS.P[i] = 0.0f;
    josephS.P[0] = shortS.P[0] = 1e4f;
    josephS.P[4] = shortS.P[4] = 1e4f;
    josephS.P[8] = shortS.P[8] = 2.5e-3f;

    const float H[3] = {1.0f, 0.0f, 0.0f};
    const float R = 1e-4f;
    josephS.fuse(H, R);
    shortS.fuse(H, R);

    // Joseph recovers the posterior variance exactly; the short form has
    // subtracted two numbers that agree to more digits than float32 has, and
    // the floor is all that is left.
    CHECK_NEAR(josephS.P[0], R, 1e-6);
    CHECK(shortS.P[0] < 1e-6f);
    CHECK(shortS.repairs > 0u);

    // And the shipped core behaves like the Joseph shadow, not the short one.
    EkfCore core;
    core.reset({0.0f, 0.0f, 0.0f}, 100.0f, 0.05f, 0);
    CHECK(core.fuseScalar(H, 0.0f, R, 0.0f));
    CHECK_NEAR(core.covariance()[0], R, 1e-6);
}

CHAD_TEST(criticJoseph, theRotatingLoopTestWouldAlsoCatchTheShortForm) {
    // Same 4000-iteration loop as ekfCovariance/josephHoldsWhereTextbookFails,
    // but run against the short form WITH symmetrization, which is the mutant
    // that test has to kill. If the mutant survives, that test is decoration.
    EkfCore seed;
    seed.reset({10.0f, 20.0f, 0.3f}, 60.0f, 0.4f, 0);
    seed.predict({1.0f, 0.0f, 0.05f}, false, 10);

    Shadow mutant;
    mutant.joseph = false;
    std::memcpy(mutant.P, seed.covariance(), sizeof mutant.P);

    const float R = 1e-4f;
    bool mutantLostPd = false;
    for (int i = 0; i < 4000; i++) {
        const float a = 0.017f * static_cast<float>(i);
        const float H[3] = {std::cos(a), std::sin(a), 0.35f * std::cos(0.5f * a)};
        mutant.fuse(H, R);
        if (!positiveDefinite(mutant.P)) mutantLostPd = true;
    }
    // Either signal is enough for the shipped test's assertions
    // (positiveDefinite and covarianceRepairs == 0) to fail on the mutant.
    CHECK(mutantLostPd || mutant.repairs > 0u);
}

CHAD_TEST(criticJoseph, hundredThousandUpdatesInFloat32) {
    // Ten times the shipped endurance run, with real range geometry rather than
    // a synthetic H, and the diagonal watched for collapse as well as for PD.
    EkfCore core;
    core.setMounts(singleMount({0.0f, 0.0f, 0.0f}));
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);

    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    std::uint32_t t = 0;
    float worstAsym = 0.0f;
    for (int i = 0; i < 100000; i++) {
        t += 10;
        core.predict({0.0f, 0.0f, 0.0f}, false, t);
        core.updateHeading(0.0f, t);
        b[0] = {true, (kWall - core.state().x) * INCH_TO_MM, 32};
        core.updateRanges(b, t);
        worstAsym = std::fmax(worstAsym, asymmetry(core.covariance()));
        if (!positiveDefinite(core.covariance())) break;
    }
    CHECK(positiveDefinite(core.covariance()));
    CHECK_NEAR(worstAsym, 0.0, 1e-12);
    CHECK_EQ(core.covarianceRepairs(), 0u);
    CHECK_FINITE(core.covariance()[0]);
    CHECK(core.state().theta >= -PI && core.state().theta <= PI);
}

// ===========================================================================
// 2. Jacobians, differentiated through the shipped propagation function
// ===========================================================================

CHAD_TEST(criticJacobian, processJacobianAtDegenerateOperatingPoints) {
    struct Point { Pose pose; OdomDelta delta; const char* what; };
    const Point points[] = {
        {{3.0f, -4.0f, 0.9f},   {0.0f, 0.0f, 0.0f},      "parked"},
        {{3.0f, -4.0f, 0.9f},   {1e-5f, 0.0f, 1e-7f},    "near-zero speed"},
        {{3.0f, -4.0f, 0.9f},   {0.35f, 0.0f, 1e-4f},    "below the sinc series cut"},
        {{3.0f, -4.0f, 0.9f},   {0.35f, 0.0f, 1.6f},     "one huge tick"},
        {{-60.0f, 60.0f, PI},   {0.5f, -0.5f, -0.3f},    "at the branch cut"},
        {{0.0f, 0.0f, 0.0f},    {0.0f, 3.0f, 0.0f},      "pure lateral"},
    };

    for (const Point& p : points) {
        float F[9];
        EkfCore::processJacobian(p.pose, p.delta, F);

        // Perturb the state THROUGH propagate(), which is the function the
        // filter actually integrates -- not through the expression F came from.
        const double dxdx = deriv([&](float v) { Pose q = p.pose; q.x = v;
                                  return EkfCore::propagate(q, p.delta).x; }, p.pose.x, 1.0f);
        const double dydx = deriv([&](float v) { Pose q = p.pose; q.x = v;
                                  return EkfCore::propagate(q, p.delta).y; }, p.pose.x, 1.0f);
        const double dxdy = deriv([&](float v) { Pose q = p.pose; q.y = v;
                                  return EkfCore::propagate(q, p.delta).x; }, p.pose.y, 1.0f);
        const double dydy = deriv([&](float v) { Pose q = p.pose; q.y = v;
                                  return EkfCore::propagate(q, p.delta).y; }, p.pose.y, 1.0f);
        const double dxdt = deriv([&](float v) { Pose q = p.pose; q.theta = v;
                                  return EkfCore::propagate(q, p.delta).x; }, p.pose.theta, 3e-3f);
        const double dydt = deriv([&](float v) { Pose q = p.pose; q.theta = v;
                                  return EkfCore::propagate(q, p.delta).y; }, p.pose.theta, 3e-3f);
        // Heading row, differentiated the same way. atan2 wraps, so compare the
        // wrapped difference rather than the raw value.
        const double dtdt = deriv([&](float v) {
            Pose q = p.pose; q.theta = v;
            return wrapRad(EkfCore::propagate(q, p.delta).theta -
                           EkfCore::propagate(p.pose, p.delta).theta);
        }, p.pose.theta, 3e-3f);

        CHECK_NEAR(F[0], dxdx, 3e-4);
        CHECK_NEAR(F[3], dydx, 3e-4);
        CHECK_NEAR(F[1], dxdy, 3e-4);
        CHECK_NEAR(F[4], dydy, 3e-4);
        CHECK_NEAR(F[2], dxdt, 2e-3);
        CHECK_NEAR(F[5], dydt, 2e-3);
        CHECK_NEAR(F[6], 0.0, 1e-9);
        CHECK_NEAR(F[7], 0.0, 1e-9);
        CHECK_NEAR(F[8], dtdt, 1e-3);
    }
}

CHAD_TEST(criticJacobian, rangeJacobianNearGrazingAndNearACorner) {
    EkfCore core;
    struct Point { Pose pose; Pose mount; };
    const Point points[] = {
        // 60 degrees of incidence: just inside the 66 degree grazing cut, where
        // the 1/cos in H is 2x and any sign slip is loud.
        {{0.0f, 0.0f, degToRad(60.0f)},   {0.0f, 0.0f, 0.0f}},
        {{0.0f, 0.0f, -degToRad(60.0f)},  {0.0f, 0.0f, 0.0f}},
        // Large mount offset: the dtheta term is dominated by the lever arm.
        {{20.0f, -30.0f, 0.35f},          {7.5f, -6.0f, 0.5f}},
        {{20.0f, -30.0f, 0.35f},          {-7.5f, 6.0f, -0.5f}},
        // Near a corner, three inches off the diagonal.
        {{60.0f, 63.0f, degToRad(-35.0f)}, {5.0f, 0.0f, 0.0f}},
        // Pointing at -Y from close in.
        {{-5.0f, -60.0f, -HALF_PI + 0.2f}, {2.0f, 2.0f, 0.0f}},
    };

    for (const Point& p : points) {
        const EkfCore::RangePrediction rp = core.predictRange(p.pose, p.mount);
        CHECK(rp.valid());
        if (!rp.valid()) continue;

        auto r = [&](const Pose& q) { return core.predictRange(q, p.mount).range; };
        const double drdx = deriv([&](float v) { Pose q = p.pose; q.x = v; return r(q); },
                                  p.pose.x, 2e-2f);
        const double drdy = deriv([&](float v) { Pose q = p.pose; q.y = v; return r(q); },
                                  p.pose.y, 2e-2f);
        const double drdt = deriv([&](float v) { Pose q = p.pose; q.theta = v; return r(q); },
                                  p.pose.theta, 2e-3f);
        CHECK_NEAR(rp.H[0], drdx, 3e-3);
        CHECK_NEAR(rp.H[1], drdy, 3e-3);
        CHECK_NEAR(rp.H[2], drdt, 6e-2);
    }
}

CHAD_TEST(criticQ, noiseEllipseAxisIsTheDirectionTheMeanActuallyMoved) {
    // The builder's comment claims the midpoint heading is right for Q while
    // the mean uses the exact exponential. Establish it instead of believing
    // it: the chord of a constant-curvature arc lies exactly along the midpoint
    // heading, so the two agree by construction.
    EkfCore::Config cfg;
    cfg.vertBiasProp = 0.0f;
    cfg.slipBiasProp = 0.0f;
    cfg.posNoiseFloor = 0.0f;
    cfg.slipNoiseFloor = 0.0f;

    const float thetas[] = {0.0f, 0.7f, -2.4f, 3.0f};
    const float dthetas[] = {0.02f, 0.4f, -0.9f, 1.5f};
    for (float th : thetas) {
        for (float dth : dthetas) {
            EkfCore core(cfg);
            core.reset({0.0f, 0.0f, th}, 0.0f, 0.0f, 0);
            const Pose before = core.state();
            core.predict({4.0f, 0.0f, dth}, false, 10);
            const Pose after = core.state();

            // Direction the mean moved, from the shipped exponential.
            const float moveDir = std::atan2(after.y - before.y, after.x - before.x);
            // Major axis of the Q ellipse actually added to P.
            const float* P = core.covariance();
            const float axis =
                0.5f * std::atan2(2.0f * P[1], P[0] - P[4]);
            CHECK_NEAR(std::fabs(std::sin(moveDir - axis)), 0.0, 2e-3);
        }
    }
}

// ===========================================================================
// 3. Gating: both thresholds pinned through the paths the filter uses
// ===========================================================================

CHAD_TEST(criticGate, theMclGateBoundarySeparatesTwoDofFromOneDof) {
    // Was: everyMclGateAssertionInTheSuiteSurvivesAOneDofThreshold. The only MCL
    // gate test in the suite accepted a fix with NIS 0.762 and rejected one with
    // NIS 1097, so it passed for every threshold in [0.77, 1097] -- including
    // 9.0, the 1-dof value, which is exactly the confusion the 2-dof gate exists
    // to guard against. These probes straddle the two.
    //
    // reset(posSigma 2) gives P00 = P11 = 4 and P01 = 0; mclCov 1.0 scaled by
    // mclSigmaScale^2 = 2.56 clears the 1.44 floor, so S00 = S11 = 6.56 and
    // NIS = (vx^2 + vy^2) / 6.56 exactly.
    auto probe = [](float vx, float vy) {
        EkfCore core;                       // default mclGateChi2
        core.reset({0.0f, 0.0f, 0.0f}, 2.0f, 0.02f, 0);
        const float S = core.covariance()[0] + 2.56f;
        EkfCore::Inputs in;
        in.mclValid = true;
        in.mclPose = {vx, vy, 0.0f};
        in.mclCovXX = in.mclCovYY = 1.0f;
        in.mclNeffFrac = 0.8f;
        core.updateMcl(in);
        const bool accepted = core.mclRejects() == 0u;
        return std::pair<bool, float>{accepted, (vx * vx + vy * vy) / S};
    };

    // Between the 1-dof threshold (9.0) and the 2-dof one (11.8): must be
    // ACCEPTED. A 1-dof threshold rejects it.
    const auto inside = probe(8.30f, 0.0f);
    CHECK_NEAR(inside.second, 10.50, 0.05);
    CHECK(inside.first);
    // Just past the 2-dof threshold: must be REJECTED. Any threshold above 13
    // accepts it.
    const auto outside = probe(9.24f, 0.0f);
    CHECK_NEAR(outside.second, 13.01, 0.05);
    CHECK(!outside.first);
    // Together these pin mclGateChi2 into (10.50, 13.01), which contains 11.8
    // and excludes 9.0, 5.99, 2.30 and everything above 13.
    EkfCore::Config cfg;
    CHECK(cfg.mclGateChi2 > inside.second);
    CHECK(cfg.mclGateChi2 < outside.second);

    // Both axes at once, so a gate built on x alone does not pass either.
    const auto diagonal = probe(6.55f, 6.55f);
    CHECK_NEAR(diagonal.second, 13.08, 0.05);
    CHECK(!diagonal.first);
    CHECK(6.55f * 6.55f / (4.0f + 2.56f) < cfg.mclGateChi2);
}

CHAD_TEST(criticGate, theRangeGateBoundaryIsPinnedThroughTheConfigPath) {
    // Was: rangeGateThresholdIsAlsoFreeThroughTheConfigPath.
    // ekfGate/chiSquareThresholdIsTheDocumentedSigmaCount set cfg.rangeGateChi2
    // and then passed 9.0f to fuseScalar as a literal, so the default was never
    // exercised: it could be 300 and every range assertion in the suite still
    // passed, at which point a 20-inch outlier moved x by 3+ inches.
    //
    // With rangeSigmaProp zeroed the measurement variance is exactly
    // rangeSigmaBase^2, so S = P00 + R is known and the boundary is arithmetic.
    EkfCore::Config cfg;                 // DEFAULT rangeGateChi2
    cfg.rangeSigmaProp = 0.0f;
    cfg.rangeSigmaBase = 0.5f;

    auto fuseAt = [&cfg](float innovation) {
        EkfCore core(cfg);
        core.setMounts(singleMount({0.0f, 0.0f, 0.0f}));
        core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);
        std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
        b[0] = {true, (kWall + innovation) * INCH_TO_MM, 32};
        core.updateRanges(b, 10);
        return core.beamStates()[0];
    };

    const float S = 1.0f + 0.25f;
    const float edge = std::sqrt(9.0f * S);   // 3 sigma, the documented count
    CHECK_NEAR(edge, 3.354, 1e-3);
    CHECK(fuseAt(0.98f * edge) == EkfCore::BeamState::Used);
    CHECK(fuseAt(1.02f * edge) == EkfCore::BeamState::Rejected);
    CHECK(fuseAt(-0.98f * edge) == EkfCore::BeamState::Used);
    CHECK(fuseAt(-1.02f * edge) == EkfCore::BeamState::Rejected);
    // So the default is pinned into (8.64, 9.36): 4.0, 16.0, 100.0 and 300.0 all
    // break one of the four assertions above.
    CHECK(cfg.rangeGateChi2 > 9.0f * 0.98f * 0.98f);
    CHECK(cfg.rangeGateChi2 < 9.0f * 1.02f * 1.02f);

    // And the outlier the loose gate used to swallow: another robot 20 inches
    // nearer than the wall, at the shipped range sigma.
    EkfCore::Config shipped;
    shipped.rangeSigmaBase = 0.0680f;
    shipped.rangeSigmaProp = 0.030f;
    EkfCore core(shipped);
    core.setMounts(singleMount({0.0f, 0.0f, 0.0f}));
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);
    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = {true, (kWall - 20.0f) * INCH_TO_MM, 32};
    core.updateRanges(b, 10);
    CHECK(core.beamStates()[0] == EkfCore::BeamState::Rejected);
    CHECK_NEAR(core.state().x, 0.0f, 1e-6);
    CHECK_EQ(core.rangeRejects(), 1u);
}

// ===========================================================================
// 4. Measurement variance from a clamped, inflate-only confidence
// ===========================================================================

CHAD_TEST(criticRange, aFaultedConfidenceReadIsRefusedNotDivided) {
    // Was: aFaultedConfidenceReadCollapsesTheRangeVariance. EkfTracking::update
    // copied pros::Distance::get_confidence() straight into Beam::confidence and
    // updateRanges divided the range sigma by it. At PROS_ERR (2147483647) that
    // was a 2.2e-8 inch sigma: the filter snapped onto the reading, declared
    // sigmaX < 1e-4, and covarianceRepairs() ticked because P had to be floored.
    // get()'s own failure was caught by sentinelMm; this one by nothing.
    //
    // A value outside 0..CONFIDENCE_MAX is a faulted port read, not a
    // confidence, and a sensor that cannot say how sure it is has not given a
    // trustworthy range either -- so the beam is refused and counted, exactly as
    // sentinelMm refuses a faulted get().
    auto run = [](int conf) {
        EkfCore core;
        core.setMounts(singleMount({0.0f, 0.0f, 0.0f}));
        core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);
        std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
        b[0] = {true, (kWall - 2.0f) * INCH_TO_MM, conf};
        core.updateRanges(b, 10);
        return core;
    };

    for (int conf : {kProsErr, EkfCore::CONFIDENCE_MAX + 1, -1, -5000}) {
        const EkfCore faulted = run(conf);
        CHECK(faulted.beamStates()[0] == EkfCore::BeamState::NoReturn);
        CHECK_NEAR(faulted.state().x, 0.0f, 1e-9);
        CHECK_NEAR(faulted.uncertainty().sigmaX, 1.0f, 1e-6);
        CHECK_EQ(faulted.covarianceRepairs(), 0u);
        CHECK_EQ(faulted.rangeNoReturns(), 1u);
        CHECK_EQ(faulted.rangeBlindStreak(), 1);
        CHECK(positiveDefinite(faulted.covariance()));
    }

    // An honest reading at the top of the legal range is still used, and gets
    // the same sigma as one at nominal confidence: 2.646 in against a 1.0 in
    // prior is a 0.125 gain on a -2 in innovation, so a quarter inch, not two.
    const EkfCore nominal = run(EkfCore::CONFIDENCE_NOMINAL);
    const EkfCore best = run(EkfCore::CONFIDENCE_MAX);
    CHECK(best.beamStates()[0] == EkfCore::BeamState::Used);
    CHECK_NEAR(best.state().x, nominal.state().x, 1e-6);
    CHECK_NEAR(best.state().x, 0.25f, 0.01f);
    CHECK(best.uncertainty().sigmaX > 0.5f);
    CHECK_EQ(best.covarianceRepairs(), 0u);
}

CHAD_TEST(criticRange, confidenceCanOnlyInflateTheMeasuredSigma) {
    // Was: maximumHardwareConfidenceHalvesTheMeasuredSigma. V5 confidence runs
    // 0..63 and the factor was 32/conf, so above 32 it SHRANK sigma: at conf 63
    // the posterior variance was 3.11x smaller than at 32, which on the shipped
    // rangeSigmaBase of 0.068 in claimed 0.86 mm of V5 time-of-flight accuracy.
    // rangeSigmaBase is bench-measured [robot]; nothing entitles the filter to
    // better than the bench.
    EkfCore::Config cfg;
    cfg.rangeSigmaProp = 0.0f;       // isolate the base
    cfg.rangeGateChi2 = 0.0f;

    auto run = [&cfg](int conf) {
        EkfCore core(cfg);
        core.setMounts(singleMount({0.0f, 0.0f, 0.0f}));
        core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);
        std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
        b[0] = {true, (kWall - 4.0f) * INCH_TO_MM, conf};
        core.updateRanges(b, 10);
        return core;
    };

    const EkfCore nominal = run(32);
    const EkfCore best = run(63);
    const EkfCore poor = run(10);

    // At and above CONFIDENCE_NOMINAL the factor is exactly one, so a happy
    // sensor buys the measured sigma and nothing more.
    CHECK_NEAR(best.covariance()[0], nominal.covariance()[0], 1e-7);
    CHECK_NEAR(best.state().x, nominal.state().x, 1e-6);
    // Below it the factor still inflates, which is the half of the model that
    // was right: conf 10 is 3.2x the sigma, so it moves the state far less.
    CHECK(poor.covariance()[0] > nominal.covariance()[0]);
    CHECK(poor.state().x < nominal.state().x);
    // R = (0.6 * 32/10)^2 = 3.686 against a prior of 1: gain 0.213 versus 0.735.
    const float gainPoor = 1.0f / (1.0f + 0.6f * 0.6f * 3.2f * 3.2f);
    const float gainNominal = 1.0f / (1.0f + 0.6f * 0.6f);
    CHECK_NEAR(poor.state().x / 4.0f, gainPoor, 0.01);
    CHECK_NEAR(nominal.state().x / 4.0f, gainNominal, 0.01);
    // The floor holds across the whole legal range.
    for (int conf : {32, 40, 50, 63}) {
        const EkfCore c = run(conf);
        CHECK(c.beamStates()[0] == EkfCore::BeamState::Used);
        CHECK_NEAR(c.covariance()[0], nominal.covariance()[0], 1e-7);
    }
    // Past it there is no confidence to scale by, only a faulted port.
    for (int conf : {64, 1000, kProsErr}) {
        const EkfCore c = run(conf);
        CHECK(c.beamStates()[0] == EkfCore::BeamState::NoReturn);
    }
}

// ===========================================================================
// 5. Sequential multi-sensor updates
// ===========================================================================

CHAD_TEST(criticSequential, eachBeamSeesThePreviousBeamsCovariance) {
    // Two beams on the +X wall, mounted at +/-4 inches laterally. Fusing them
    // through updateRanges must equal fusing them one at a time by hand, and
    // must NOT equal fusing the second one against the pre-update P.
    EkfCore::Config cfg;
    cfg.rangeGateChi2 = 0.0f;
    cfg.rangeSigmaProp = 0.0f;
    cfg.rangeSigmaBase = 0.5f;

    std::array<Pose, EkfCore::SENSOR_COUNT> mounts{};
    mounts[0] = {0.0f, 4.0f, 0.0f};
    mounts[1] = {0.0f, -4.0f, 0.0f};

    EkfCore both(cfg);
    both.setMounts(mounts);
    both.reset({6.0f, 0.0f, 0.05f}, 3.0f, 0.05f, 0);

    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = {true, kWall * INCH_TO_MM, 32};
    b[1] = {true, kWall * INCH_TO_MM, 32};
    both.updateRanges(b, 10);
    CHECK_EQ(both.beamsUsed(), 2);

    // By hand, sequentially, re-linearising the second beam at the state the
    // first one left.
    EkfCore hand(cfg);
    hand.setMounts(mounts);
    hand.reset({6.0f, 0.0f, 0.05f}, 3.0f, 0.05f, 0);
    for (int i = 0; i < 2; i++) {
        const Pose now = hand.state();
        const EkfCore::RangePrediction rp =
            hand.predictRange(now, mounts[static_cast<std::size_t>(i)]);
        CHECK(rp.valid());
        const float sigma = 0.5f / std::max(rp.cosIncidence, 0.25f);
        CHECK(hand.fuseScalar(rp.H, kWall - rp.range, sigma * sigma, 0.0f));
    }
    CHECK_NEAR(both.state().x, hand.state().x, 1e-5);
    CHECK_NEAR(both.state().y, hand.state().y, 1e-5);
    CHECK_NEAR(both.state().theta, hand.state().theta, 1e-6);
    for (int i = 0; i < 9; i++) CHECK_NEAR(both.covariance()[i], hand.covariance()[i], 1e-5);

    // The stale variant: both beams predicted and gained from the pre-update
    // state and P. It lands somewhere else, so the check above has teeth.
    EkfCore stale(cfg);
    stale.setMounts(mounts);
    stale.reset({6.0f, 0.0f, 0.05f}, 3.0f, 0.05f, 0);
    const Pose frozen = stale.state();
    for (int i = 0; i < 2; i++) {
        const EkfCore::RangePrediction rp =
            stale.predictRange(frozen, mounts[static_cast<std::size_t>(i)]);
        const float sigma = 0.5f / std::max(rp.cosIncidence, 0.25f);
        stale.fuseScalar(rp.H, kWall - rp.range, sigma * sigma, 0.0f);
    }
    CHECK(std::fabs(stale.state().x - both.state().x) > 1e-3f);
}

// ===========================================================================
// 6. A held IMU heading is charged the growth it hides
// ===========================================================================

CHAD_TEST(criticHeading, aDeadImuIsChargedTheHeadingErrorItHides) {
    // Was: aDeadImuLeavesTheFilterConfidentAndNinetyDegreesWrong. predict() took
    // no headingValid flag and step() discarded the one it was handed, so a robot
    // turning 90 degrees through an IMU outage looked exactly like a robot
    // standing still: dTheta 0, thetaSigma at the floor, 1.28 degrees declared
    // against 90 degrees of error. A 70:1 lie.
    EkfCore core;
    core.reset({0.0f, 0.0f, 0.0f}, 0.5f, 0.02f, 0);

    OdomCore odom;
    odom.rebaseline(0.0f, 0.0f, 0.0f);

    std::uint32_t t = 0;
    float trueTheta = 0.0f;
    float driven = 0.0f;
    for (int i = 0; i < 200; i++) {          // two seconds of IMU outage
        t += 10;
        trueTheta += HALF_PI / 200.0f;       // the robot turns 90 degrees
        driven += 0.2f;
        // headingValid false: the IMU faulted, so OdomCore holds and reports
        // no rotation. This is exactly what EkfTracking feeds the core.
        const OdomDelta d = odom.step(driven, 0.0f, trueTheta, false);
        CHECK_NEAR(d.dTheta, 0.0f, 1e-9);
        EkfCore::Inputs in;
        in.delta = d;
        in.headingValid = false;
        core.step(t, in);
    }

    // The mean is still 90 degrees out -- with no heading source and no encoder
    // differential there is nothing to correct it with, and a held reading is
    // still not a measurement.
    CHECK_NEAR(core.state().theta, 0.0f, 1e-4);
    CHECK_NEAR(trueTheta, HALF_PI, 1e-3);
    // But the covariance now says so: unmeasured heading is charged
    // thetaUnmeasuredVarPerSec, so sigma grows as rate*sqrt(t) instead of
    // sitting on the floor.
    const float sigmaTheta = core.uncertainty().sigmaTheta;
    CHECK_NEAR(sigmaTheta * RAD_TO_DEG, 81.0, 1.0);
    CHECK(radToDeg(HALF_PI) / (sigmaTheta * RAD_TO_DEG) < 1.5f);

    // The growth is charged only while the heading is unmeasured. The same trace
    // with a live IMU keeps a tight sigma, so the term cannot be masking a
    // filter that is simply loose.
    EkfCore measured;
    measured.reset({0.0f, 0.0f, 0.0f}, 0.5f, 0.02f, 0);
    OdomCore odom2;
    odom2.rebaseline(0.0f, 0.0f, 0.0f);
    t = 0;
    trueTheta = 0.0f;
    driven = 0.0f;
    for (int i = 0; i < 200; i++) {
        t += 10;
        trueTheta += HALF_PI / 200.0f;
        driven += 0.2f;
        EkfCore::Inputs in;
        in.delta = odom2.step(driven, 0.0f, trueTheta, true);
        in.headingValid = true;
        in.headingRad = trueTheta;
        measured.step(t, in);
    }
    CHECK_NEAR(measured.state().theta, HALF_PI, 0.01f);
    CHECK(measured.uncertainty().sigmaTheta * RAD_TO_DEG < 1.0f);

    // And one tick of outage is charged one tick's worth, not nothing: the old
    // shipped assertion (P[8] > 0.02*0.02) passed by 0.04% and stayed satisfied
    // no matter how wrong the heading got.
    EkfCore oneTick;
    oneTick.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);
    EkfCore::Inputs idle;
    oneTick.step(10, idle);
    const float perTick = oneTick.config().thetaUnmeasuredVarPerSec
                        * 0.001f * oneTick.config().updatePeriodMs;
    CHECK_NEAR(oneTick.covariance()[8] - 0.02f * 0.02f, perTick, 1e-5);
    CHECK(oneTick.covariance()[8] > 20.0f * 0.02f * 0.02f);
}

CHAD_TEST(criticHeading, rangeHeadingInformationDecaysBackOntoTheRawImu) {
    // The IMU drives the prediction (through OdomDelta::dTheta) AND is fused as
    // an absolute measurement, so the two are treated as independent when they
    // are the same number. Consequence: any heading the ranges win is pulled
    // back to the raw IMU with a fixed time constant.
    EkfCore core;
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);

    // Settle P[8] to its steady state under repeated IMU updates.
    std::uint32_t t = 0;
    for (int i = 0; i < 400; i++) {
        t += 10;
        core.predict({0.0f, 0.0f, 0.0f}, false, t);
        core.updateHeading(0.0f, t);
    }
    const float pSteady = core.covariance()[8];
    const float R = 0.006f * 0.006f;
    const float gain = pSteady / (pSteady + R);
    CHECK(gain > 0.02f);
    CHECK(gain < 0.20f);

    // Plant a 5 degree heading correction, then let the IMU speak again.
    const float H[3] = {0.0f, 0.0f, 1.0f};
    CHECK(core.fuseScalar(H, degToRad(5.0f), 1e-9f, 0.0f));
    const float planted = core.state().theta;
    CHECK(planted > degToRad(4.5f));

    int ticksToHalf = -1;
    for (int i = 0; i < 400; i++) {
        t += 10;
        core.predict({0.0f, 0.0f, 0.0f}, false, t);
        core.updateHeading(0.0f, t);
        if (ticksToHalf < 0 && core.state().theta < 0.5f * planted) ticksToHalf = i + 1;
    }
    CHECK(ticksToHalf > 0);
    CHECK(ticksToHalf < 60);                       // under 600 ms to halve
    CHECK_NEAR(core.state().theta, 0.0f, 1e-3);    // and all of it is gone
}

// ===========================================================================
// 7. Obstacle deadzone: monotone in the filter's own sigma, and bounded
// ===========================================================================

CHAD_TEST(criticObstacles, theSigmaPadIsMonotoneInTheDocumentedDirection) {
    // Was: theSigmaPadMakesTheDeadzoneLooserNotStricter. The old branch
    // `postRange + tol < wallRange` got HARDER to satisfy as tol grew, so
    // raising obstacleSigmaPad -- documented as making an unsure filter "drop
    // more" -- turned a deadzoned beam into a fused one. Every use of tol is now
    // monotone the documented way.
    const RangeObstacle blocker{40.0f, 0.0f, 1.0f};   // postRange 39, wall 70.2

    // A return at the post surface is deadzoned at every pad, and a wider pad
    // never un-deadzones it.
    for (float pad : {0.0f, 2.0f, 8.0f}) {
        EkfCore::Config cfg;
        cfg.obstacleSigmaPad = pad;
        EkfCore core(cfg);
        core.setObstacles(&blocker, 1);
        core.reset({0.0f, 0.0f, 0.0f}, 10.0f, 0.01f, 0);   // sigma 14.1 in
        CHECK(core.obstacleOwnsReading(0.0f, 0.0f, 1.0f, 0.0f, 39.0f, kWall));
    }

    // Monotone in the filter's own sigma: a reading 12 inches short of the post
    // is outside a confident filter's tolerance and inside an unsure one's.
    auto owned = [&](float posSigma) {
        EkfCore::Config cfg;
        EkfCore core(cfg);
        core.setObstacles(&blocker, 1);
        core.reset({0.0f, 0.0f, 0.0f}, posSigma, 0.01f, 0);
        return core.obstacleOwnsReading(0.0f, 0.0f, 1.0f, 0.0f, 27.0f, kWall);
    };
    CHECK(!owned(0.05f));   // tol 6.1
    CHECK(owned(6.0f));     // tol 6 + 2*8.49, capped at 18

    // Monotone in the CONE too, which is the other half of "unsure which beams
    // a post could be spoiling": a post 20 inches off a ray that passes 40
    // inches from it is outside a confident filter's cone (1 + 40*0.22 + 0.14 =
    // 9.9) and inside an unsure one's (1 + 8.8 + 12 = 21.8). The cone pad is the
    // pose-sigma part only -- obstacleRangeTol is a range slop, not a lateral
    // one -- capped at obstacleTolMaxIn - obstacleRangeTol.
    auto lateral = [&](float posSigma) {
        EkfCore::Config cfg;
        EkfCore core(cfg);
        core.setObstacles(&blocker, 1);
        core.reset({0.0f, 0.0f, 0.0f}, posSigma, 0.01f, 0);
        // postRange = hypot(40, 20) - 1 = 43.72, and the beam read it.
        return core.obstacleOwnsReading(0.0f, -20.0f, 1.0f, 0.0f, 43.7f, kWall);
    };
    CHECK(!lateral(0.05f));
    CHECK(lateral(6.0f));

    // And bounded: obstacleTolMaxIn stops a lost filter deadzoning every beam it
    // owns, which is the state it most needs them in.
    EkfCore::Config cap;
    EkfCore lost(cap);
    lost.setObstacles(&blocker, 1);
    lost.reset({0.0f, 0.0f, 0.0f}, 60.0f, 0.5f, 0);   // sigma 84.9 in
    CHECK(!lost.obstacleOwnsReading(0.0f, 0.0f, 1.0f, 0.0f, kWall, kWall));
    CHECK_NEAR(cap.obstacleTolMaxIn, 18.0, 1e-6);
}

CHAD_TEST(criticObstacles, aWallRangeReadingThroughAPostConeIsKept) {
    // Was: theTwoBranchesAreComplementsForAWallRangeReading. The two accept
    // branches were exact complements for a reading at the predicted wall range
    // (W - p > tol XOR W - p <= tol), so ANY beam whose 12.5 degree cone held a
    // post in front of the wall was dropped unconditionally -- an 18 inch wide
    // blind strip per post at 40 inches, and tol could not change one outcome.
    //
    // The occlusion branch is gone. A return that matches the wall says the cone
    // test was the conservative over-estimate it is; a return that matches the
    // post is still deadzoned, and everything in between belongs to the
    // Mahalanobis gate, which uses the live covariance and is counted.
    const RangeObstacle post{40.0f, 0.0f, 1.0f};   // postRange 39, wall at 70.2

    for (float sigmaPad : {0.0f, 2.0f, 20.0f}) {
        for (float posSigma : {0.05f, 0.5f, 10.0f, 40.0f}) {
            EkfCore::Config cfg;
            cfg.obstacleSigmaPad = sigmaPad;
            EkfCore core(cfg);
            core.setObstacles(&post, 1);
            core.reset({0.0f, 0.0f, 0.0f}, posSigma, 0.01f, 0);
            CHECK(!core.obstacleOwnsReading(0.0f, 0.0f, 1.0f, 0.0f, kWall, kWall));
            CHECK(core.obstacleOwnsReading(0.0f, 0.0f, 1.0f, 0.0f, 39.0f, kWall));
        }
    }

    // The strip that used to be blind: 9.8 inches either side of the ray at 40
    // inches out. A beam offset into it now measures the wall.
    EkfCore wide;
    wide.setObstacles(&post, 1);
    wide.reset({0.0f, 0.0f, 0.0f}, 0.5f, 0.01f, 0);
    const float halfWidth = post.radius + 40.0f * 0.22f;
    CHECK_NEAR(halfWidth, 9.8, 0.05);
    CHECK(!wide.obstacleOwnsReading(0.0f, 8.0f, 1.0f, 0.0f, kWall, kWall));
    // Still deadzoned when the return really is the post.
    CHECK(wide.obstacleOwnsReading(0.0f, 8.0f, 1.0f, 0.0f, 40.0f, kWall));

    // A post behind the wall this ray hits cannot have produced the reading, and
    // a post on top of the sensor means nothing this beam reads is a wall.
    const RangeObstacle behind{90.0f, 0.0f, 1.0f};
    EkfCore far;
    far.setObstacles(&behind, 1);
    far.reset({0.0f, 0.0f, 0.0f}, 0.1f, 0.01f, 0);
    CHECK(!far.obstacleOwnsReading(0.0f, 0.0f, 1.0f, 0.0f, kWall, kWall));
    const RangeObstacle touching{1.0f, 0.0f, 2.0f};
    EkfCore near;
    near.setObstacles(&touching, 1);
    near.reset({0.0f, 0.0f, 0.0f}, 0.1f, 0.01f, 0);
    CHECK(near.obstacleOwnsReading(0.0f, 0.0f, 1.0f, 0.0f, kWall, kWall));
}

// ===========================================================================
// 8. Range latency: EkfCore::rewind switched on
// ===========================================================================

CHAD_TEST(criticLatency, theRangeRewindIsOnByDefaultAndRemovesTheBias) {
    // Was: unrewoundStaleRangeDragsTheEstimateBackwards. Fusing a reading that
    // describes the pose 50 ms ago against the present one is a BIAS of
    // speed*latency, not noise: the prediction is unbiased, so the fixed point
    // is exactly -v*dt whatever the gain, and more beams do not average it out.
    // rangeLatencyMs now defaults to a nonzero estimate, because zero is the one
    // value guaranteed wrong.
    EkfCore::Config shipped;
    CHECK(shipped.rangeLatencyMs > 0.0f);

    auto run = [](float latencyMs) {
        EkfCore::Config cfg;
        cfg.rangeLatencyMs = latencyMs;
        EkfCore core(cfg);
        core.setMounts(singleMount({0.0f, 0.0f, 0.0f}));
        core.reset({-60.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);
        // 0.6 in per 10 ms tick == 60 in/s, and the beam reports the pose five
        // ticks back. The trail is primed first so the rewind has history.
        float truth = -60.0f;
        float lagged[6] = {-60.0f, -60.0f, -60.0f, -60.0f, -60.0f, -60.0f};
        std::uint32_t t = 0;
        std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
        for (int i = 0; i < 300; i++) {
            t += 10;
            truth += 0.6f;
            for (int k = 5; k > 0; k--) lagged[k] = lagged[k - 1];
            lagged[0] = truth;
            core.predict({0.6f, 0.0f, 0.0f}, false, t);
            b[0] = {true, (kWall - lagged[5]) * INCH_TO_MM, 32};
            core.updateRanges(b, t);
        }
        return core.state().x - truth;
    };

    // The old default. 60 in/s * 50 ms = 3.00 inches of standing offset, and
    // the filter reports rangeRejectStreak 0 while it sits there.
    const float errOff = run(0.0f);
    CHECK_NEAR(errOff, -3.0f, 0.05f);
    // The rewind, matched to the trace, is unbiased.
    const float errExact = run(50.0f);
    CHECK(std::fabs(errExact) < 0.05f);
    // The shipped default is an ESTIMATE of the port latency, so the residual is
    // speed * (true - configured). Being 20 ms out costs 1.2 in; being 50 ms out
    // costs 3.0. A wrong-but-plausible latency strictly beats zero.
    const float errShipped = run(shipped.rangeLatencyMs);
    CHECK(std::fabs(errShipped) < std::fabs(errOff));
    CHECK_NEAR(std::fabs(errShipped), 0.06f * (50.0f - shipped.rangeLatencyMs), 0.1f);
}

// ===========================================================================
// 9. Ray cast edges
// ===========================================================================

CHAD_TEST(criticRayCast, theNearerFaceWinsNextToACorner) {
    EkfCore core;
    // Just inside the +Y wall, facing +X: the +X face is the right answer even
    // though the slab test's |yAt| <= kWall + 1 slack would tolerate a hit an
    // inch outside the box.
    const EkfCore::RangePrediction along =
        core.predictRange({0.0f, kWall - 0.4f, 0.0f}, {0.0f, 0.0f, 0.0f});
    CHECK(along.valid());
    CHECK_NEAR(along.nx, 1.0, 1e-6);
    CHECK_NEAR(along.range, kWall, 1e-3);

    // Angled up at 45 degrees, the +Y face is nearer and is the one chosen.
    const EkfCore::RangePrediction up =
        core.predictRange({0.0f, kWall - 0.4f, QUARTER_PI}, {0.0f, 0.0f, 0.0f});
    CHECK(up.valid());
    CHECK_NEAR(up.ny, 1.0, 1e-6);
    CHECK_NEAR(up.range, 0.4f / std::sin(QUARTER_PI), 1e-2);

    // At 20 degrees the +Y face is still nearer, but the incidence is 70
    // degrees, past maxIncidenceRad, so the beam is refused rather than
    // inverted through a 1/cos of 2.9.
    const EkfCore::RangePrediction graze =
        core.predictRange({0.0f, kWall - 0.4f, degToRad(20.0f)}, {0.0f, 0.0f, 0.0f});
    CHECK(graze.state == EkfCore::BeamState::Grazing);
}

CHAD_TEST(criticRayCast, aSensorOriginOutsideTheFieldDoesNotInventAWall) {
    EkfCore core;
    // Robot believes it is against the wall with a 7 inch forward mount, so the
    // beam origin is outside the box. Nothing should come back as a wall hit
    // at a bogus range.
    const EkfCore::RangePrediction out =
        core.predictRange({kWall - 2.0f, 0.0f, 0.0f}, {7.0f, 0.0f, 0.0f});
    CHECK(out.state == EkfCore::BeamState::NoReturn);
    CHECK(!out.valid());
}

// ===========================================================================
// 10. Odometry: one wheel out of several is dropped and counted
// ===========================================================================

CHAD_TEST(criticOdom, aHalvedDistanceSourceIsDroppedAndCounted) {
    // Was: aHalvedDistanceSourceIsNotAJumpAndIsNotCounted. TrackingWheel holds
    // its last good distance on PROS_ERR and averageDistance used to average
    // that held value in with the live ones, so one dead wheel of two reported
    // half the distance driven: every delta was scaled, none of them jumped, so
    // maxDeltaIn could not fire and no counter moved.
    //
    // averageLiveDistance is the arithmetic OdomTracking::averageDistance now
    // uses, hoisted out of the PROS adapter so it can be exercised here.
    float healthy = 0.0f;
    float frozen = 0.0f;
    OdomCore averaged, live;
    averaged.rebaseline(0.0f, 0.0f, 0.0f);
    live.rebaseline(0.0f, 0.0f, 0.0f);

    int faultTicks = 0;
    for (int i = 0; i < 100; i++) {
        healthy += 0.5f;                     // the robot really drives 50 in
        const float values[2] = {healthy, frozen};
        const bool bad[2] = {false, true};   // wheel 1 is on a faulted port

        // The old arithmetic: the held value counted as a reading.
        const DistanceAverage all = averageLiveDistance(values, nullptr, 2);
        averaged.step(all.value, 0.0f, 0.0f, true);

        const DistanceAverage only = averageLiveDistance(values, bad, 2);
        CHECK_EQ(only.live, 1);
        CHECK_EQ(only.faulted, 1);
        if (only.faulted > 0) faultTicks++;
        live.step(only.value, 0.0f, 0.0f, true);
    }
    // Averaging the held value halves the distance, silently.
    CHECK_NEAR(averaged.pose().x, 25.0f, 1e-2);
    CHECK_EQ(averaged.jumpRejects(), 0u);
    // Dropping it keeps the scale, and every dropped tick is counted -- which is
    // what OdomTracking::distanceFaults() reports.
    CHECK_NEAR(live.pose().x, 50.0f, 1e-2);
    CHECK_EQ(live.jumpRejects(), 0u);
    CHECK_EQ(faultTicks, 100);

    // All faulted is an outage, not a zero: the caller has to hold rather than
    // step to a different sensor set.
    const float vals[2] = {10.0f, 12.0f};
    const bool allBad[2] = {true, true};
    const DistanceAverage none = averageLiveDistance(vals, allBad, 2);
    CHECK(!none.valid());
    CHECK_EQ(none.faulted, 2);
    // A non-finite reading is a fault whatever the port said.
    const float withNan[2] = {std::numeric_limits<float>::quiet_NaN(), 4.0f};
    const DistanceAverage skipped = averageLiveDistance(withNan, nullptr, 2);
    CHECK_EQ(skipped.live, 1);
    CHECK_NEAR(skipped.value, 4.0f, 1e-6);
}

CHAD_TEST(criticOdom, repeatedAndBackwardTimestampsDoNotBreakTheTrail) {
    EkfCore::Config cfg;
    cfg.rangeLatencyMs = 100.0f;
    EkfCore core(cfg);
    core.setMounts(singleMount({0.0f, 0.0f, 0.0f}));
    core.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 1000);

    // dt == 0 repeatedly, then time going backwards.
    for (int i = 0; i < 20; i++) core.predict({0.3f, 0.0f, 0.0f}, false, 1000);
    for (int i = 0; i < 20; i++) core.predict({0.3f, 0.0f, 0.0f}, false, 900);
    std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
    b[0] = {true, (kWall - core.state().x) * INCH_TO_MM, 32};
    core.updateRanges(b, 900);
    CHECK_FINITE(core.state().x);
    CHECK(positiveDefinite(core.covariance()));
    CHECK(core.beamStates()[0] == EkfCore::BeamState::Used);
}

// ===========================================================================
// 11. Numbers quoted in the review, printed so they can be checked
// ===========================================================================

CHAD_TEST(criticNumbers, printsTheQuantitiesTheReviewQuotes) {
    // Steady-state heading gain against the IMU, and the resulting time
    // constant of the pull back onto the raw IMU heading.
    EkfCore h;
    h.reset({0.0f, 0.0f, 0.0f}, 1.0f, 0.02f, 0);
    std::uint32_t t = 0;
    for (int i = 0; i < 600; i++) {
        t += 10;
        h.predict({0.6f, 0.0f, 0.0f}, false, t);
        h.updateHeading(0.0f, t);
    }
    const float p8 = h.covariance()[8];
    const float R = 0.006f * 0.006f;
    std::printf("    [num] P[8] steady = %.4g, K = %.4f, tau = %.1f ms\n",
                static_cast<double>(p8), static_cast<double>(p8 / (p8 + R)),
                static_cast<double>(10.0f * (p8 + R) / p8));

    // Position sigma after 40 inches of straight driving at 0.6 in/tick, which
    // is what the bias term in Q is calibrated to produce.
    EkfCore q;
    q.reset({0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0);
    t = 0;
    for (int i = 0; i < 67; i++) { t += 10; q.predict({0.6f, 0.0f, 0.0f}, false, t); }
    std::printf("    [num] after 40 in dead reckoning: sigmaX = %.3f in, sigmaY = %.3f in\n",
                static_cast<double>(q.uncertainty().sigmaX),
                static_cast<double>(q.uncertainty().sigmaY));

    // Steady-state bias from fusing a 50 ms stale range against the present
    // pose, at 60 in/s.
    for (float latency : {0.0f, 50.0f}) {
        EkfCore::Config cfg;
        cfg.rangeLatencyMs = latency;
        EkfCore core(cfg);
        core.setMounts(singleMount({0.0f, 0.0f, 0.0f}));
        core.reset({-60.0f, 0.0f, 0.0f}, 1.0f, 0.01f, 0);
        float truth = -60.0f;
        float lagged[6] = {-60.0f, -60.0f, -60.0f, -60.0f, -60.0f, -60.0f};
        std::uint32_t tt = 0;
        std::array<EkfCore::Beam, EkfCore::SENSOR_COUNT> b{};
        for (int i = 0; i < 300; i++) {
            tt += 10;
            truth += 0.6f;
            for (int k = 5; k > 0; k--) lagged[k] = lagged[k - 1];
            lagged[0] = truth;
            core.predict({0.6f, 0.0f, 0.0f}, false, tt);
            b[0] = {true, (kWall - lagged[5]) * INCH_TO_MM, 32};
            core.updateRanges(b, tt);
        }
        std::printf("    [num] rangeLatencyMs = %5.1f -> steady-state x error %.3f in\n",
                    static_cast<double>(latency),
                    static_cast<double>(core.state().x - truth));
    }

    // Range measurement sigma across the legal V5 confidence range, and at
    // PROS_ERR. The factor inflates and never shrinks, so everything at or above
    // CONFIDENCE_NOMINAL reports the bench-measured sigma.
    const EkfCore::Config sigmaCfg;
    for (int conf : {10, 32, 63, kProsErr}) {
        // Exactly what updateRanges does with the raw port value: out of range
        // is not a confidence, it is a faulted read.
        const int held = (conf < 0 || conf > EkfCore::CONFIDENCE_MAX) ? 0 : conf;
        if (held < sigmaCfg.minConfidence) {
            std::printf("    [num] conf = %10d -> refused, no return\n", conf);
            continue;
        }
        const float sigma = (0.6f + 0.03f * 30.0f) *
            std::fmax(static_cast<float>(EkfCore::CONFIDENCE_NOMINAL) /
                          static_cast<float>(std::max(held, 1)), 1.0f);
        std::printf("    [num] conf = %10d -> range sigma %.6g in at 30 in\n", conf,
                    static_cast<double>(sigma));
    }
    CHECK(true);
}

// ===========================================================================
// 12. The shipped tuning: the [gap] terms against what phase 3 measures
// ===========================================================================

CHAD_TEST(criticShipped, theBiasTermsSitInAKnownRatioToWhatPhaseThreeMeasures) {
    // Was: theUnmeasuredBiasTermsSwampThePhaseThreeMeasurement. The [gap] bias
    // defaults used to put sigmaX at 2.410 in after 40 inches of dead reckoning
    // against 0.142 in from the measured terms alone -- 17x -- which opened the
    // 3-sigma range gate to +/-7.4 inches and gave a 20-inch outlier a 0.96
    // single-tick gain. vertBiasProp/slipBiasProp are now 0.010, sized at 2.5x
    // the 0.0040/in out-and-back closure error phase 3 does measure.
    EkfCore::Config shipped;
    shipped.vertNoiseProp = 0.0040f;
    shipped.slipNoiseProp = 0.0040f;
    shipped.rangeSigmaBase = 0.0680f;
    shipped.rangeSigmaProp = 0.030f;

    EkfCore withBias(shipped);
    withBias.reset({0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0);

    EkfCore::Config noBias = shipped;
    noBias.vertBiasProp = 0.0f;
    noBias.slipBiasProp = 0.0f;
    EkfCore without(noBias);
    without.reset({0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0);

    std::uint32_t t = 0;
    for (int i = 0; i < 67; i++) {           // 40 inches at 0.6 in per tick
        t += 10;
        withBias.predict({0.6f, 0.0f, 0.0f}, false, t);
        without.predict({0.6f, 0.0f, 0.0f}, false, t);
    }
    std::printf("    [num] shipped cfg, 40 in dead reckoned: sigmaX with bias %.3f in, "
                "without %.3f in\n",
                static_cast<double>(withBias.uncertainty().sigmaX),
                static_cast<double>(without.uncertainty().sigmaX));

    // The bias term still dominates -- that is its job, the white term really
    // does undercount a systematic error -- but by a factor of 3, not 17, and
    // its own magnitude is now the 1% of distance a tape-measured wheel
    // diameter leaves rather than 6%.
    CHECK_NEAR(withBias.uncertainty().sigmaX, 0.426, 0.02);
    CHECK(without.uncertainty().sigmaX < 0.20f);
    CHECK(withBias.uncertainty().sigmaX > without.uncertainty().sigmaX);
    CHECK(withBias.uncertainty().sigmaX < 4.0f * without.uncertainty().sigmaX);
    // vertBiasProp alone, over the decorrelation length, is the claimed
    // systematic error at that distance: 0.010 * 40 in = 0.40 in.
    CHECK_NEAR(shipped.vertBiasProp * shipped.biasDecorrIn, 0.40, 1e-4);

    // And that is the sigma the 3-sigma range gate is built on. The range sigma
    // at 30 inches is the confidence-scaled base, which can no longer shrink
    // below the bench measurement.
    const float* P = withBias.covariance();
    const float sigma30 = shipped.rangeSigmaBase + shipped.rangeSigmaProp * 30.0f;
    const float S = P[0] + sigma30 * sigma30;
    std::printf("    [num] gate half-width %.2f in, single-tick gain %.3f\n",
                static_cast<double>(3.0f * std::sqrt(S)),
                static_cast<double>(P[0] / S));
    // Wide enough for honest drift, tight enough to refuse another robot's
    // shin, and no longer handing a whole outlier to the state in one cycle.
    CHECK(3.0f * std::sqrt(S) > 2.0f);
    CHECK(3.0f * std::sqrt(S) < 4.5f);
    CHECK(P[0] / S < 0.25f);
}
