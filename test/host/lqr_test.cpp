#include "chad/movement_control/lqr.hpp"

#include <cmath>

#include "chad/movement_control/mpc_lqr.hpp"
#include "chad_test.hpp"

using namespace chad;

namespace {

/// Q and R as lqr.cpp's costs() builds them: the weights are squared, and R is
/// floored so S = R + B'PB cannot go singular.
void lqrCosts(const LqrWeights& w, float Q[3], float R[2]) {
    Q[0] = w.qx * w.qx;
    Q[1] = w.qy * w.qy;
    Q[2] = w.qtheta * w.qtheta;
    const float rv = std::fmax(w.rv, 1e-3f);
    const float rw = std::fmax(w.romega, 1e-3f);
    R[0] = rv * rv;
    R[1] = rw * rw;
}

/// max |K - K*|, where K* is solved directly from the Riccati stationarity
/// condition (R + B'PB) K* = B'PA. Comparing gains rather than a normalised
/// residual keeps the check equally sensitive in every entry -- normalising by
/// the largest entry of B'PA hides an error in a small one.
float gainMismatch(float v, float dt, const LqrWeights& w, const float K[6],
                   const float P[9]) {
    float A[9], B[6];
    DrivetrainErrorPlant::discretise(v, dt, A, B);
    float Q[3], R[2];
    lqrCosts(w, Q, R);

    float S[4] = {R[0], 0.0f, 0.0f, R[1]};
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++)
                    S[i * 2 + j] += B[a * 2 + i] * P[a * 3 + b] * B[b * 2 + j];

    float T[6] = {};
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++)
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++)
                    T[i * 3 + j] += B[a * 2 + i] * P[a * 3 + b] * A[b * 3 + j];

    const float det = S[0] * S[3] - S[1] * S[2];
    const float si[4] = {S[3] / det, -S[1] / det, -S[2] / det, S[0] / det};

    float worst = 0.0f;
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 3; j++) {
            const float star = si[i * 2] * T[j] + si[i * 2 + 1] * T[3 + j];
            worst = std::fmax(worst, std::fabs(K[i * 3 + j] - star));
        }
    return worst;
}

/// ||x_n|| / ||x_0|| under x <- (A - BK)x. Below 1 only if the closed loop is
/// actually stable, which no single-step check can establish.
float closedLoopDecay(float v, float dt, const float K[6], const float x0[3], int steps) {
    float A[9], B[6];
    DrivetrainErrorPlant::discretise(v, dt, A, B);

    float acl[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float bk = 0.0f;
            for (int m = 0; m < 2; m++) bk += B[i * 2 + m] * K[m * 3 + j];
            acl[i * 3 + j] = A[i * 3 + j] - bk;
        }

    float x[3] = {x0[0], x0[1], x0[2]};
    const float start = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    for (int s = 0; s < steps; s++) {
        float nx[3];
        for (int i = 0; i < 3; i++) {
            nx[i] = acl[i * 3] * x[0] + acl[i * 3 + 1] * x[1] + acl[i * 3 + 2] * x[2];
        }
        x[0] = nx[0];
        x[1] = nx[1];
        x[2] = nx[2];
    }
    return std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]) / start;
}

/// Independent DARE reference: plain value iteration in double precision, with
/// no shared code with riccati.hpp's doubling. Both solvers in the library now
/// route to solveDare, so a cross-check between THEM proves only that the
/// wrappers agree; the solver itself needs a reference that was written twice.
bool referenceDare(const float A[9], const float B[6], const float Q[3],
                   const float R[2], double Kout[6]) {
    double P[3][3] = {{Q[0], 0, 0}, {0, Q[1], 0}, {0, 0, Q[2]}};
    for (int iter = 0; iter < 200000; iter++) {
        double S[2][2] = {{R[0], 0}, {0, R[1]}};
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 3; b++) S[i][j] += B[a * 2 + i] * P[a][b] * B[b * 2 + j];
        double T[2][3] = {};
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 3; j++)
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 3; b++) T[i][j] += B[a * 2 + i] * P[a][b] * A[b * 3 + j];
        const double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        const double si[2][2] = {{S[1][1] / det, -S[0][1] / det},
                                 {-S[1][0] / det, S[0][0] / det}};
        double K[2][3];
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 3; j++) K[i][j] = si[i][0] * T[0][j] + si[i][1] * T[1][j];

        double Pn[3][3], worst = 0.0;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                double atpa = 0.0;
                for (int a = 0; a < 3; a++)
                    for (int b = 0; b < 3; b++) atpa += A[a * 3 + i] * P[a][b] * A[b * 3 + j];
                double corr = 0.0;
                for (int m = 0; m < 2; m++) corr += T[m][i] * K[m][j];
                Pn[i][j] = (i == j ? Q[i] : 0.0) + atpa - corr;
            }
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                const double sym = 0.5 * (Pn[i][j] + Pn[j][i]);
                worst = std::fmax(worst, std::fabs(sym - P[i][j]));
                Pn[i][j] = sym;
            }
        double scale = 1.0;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                P[i][j] = Pn[i][j];
                scale = std::fmax(scale, std::fabs(P[i][j]));
            }
        // Relative: P runs to ~1e4 here, so an absolute 1e-13 is below the
        // double ULP of its own entries and never arrives.
        if (worst < 1e-12 * scale) {
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 3; j++) Kout[i * 3 + j] = K[i][j];
            return true;
        }
    }
    return false;
}

}  // namespace

CHAD_TEST(lqr, gainSatisfiesTheStationarityCondition) {
    const LqrWeights w{};
    for (float v : {0.0f, 2.0f, 12.0f, 48.0f, 80.0f}) {
        float P[9];
        const LqrGain g = solveLqr(v, 0.01f, w, 600, 1e-6f, P);
        for (int i = 0; i < 6; i++) CHECK(std::isfinite(g.k[i]));
        CHECK(gainMismatch(v, 0.01f, w, g.k, P) < 2e-3f);
    }
}

CHAD_TEST(lqr, theStationarityCheckRejectsAWrongGain) {
    // Negative control: without this, a residual test that silently always
    // passes would look exactly like a correct solver.
    const LqrWeights w{};
    float P[9];
    const LqrGain g = solveLqr(12.0f, 0.01f, w, 600, 1e-6f, P);
    CHECK(gainMismatch(12.0f, 0.01f, w, g.k, P) < 2e-3f);

    for (int bad = 0; bad < 6; bad++) {
        LqrGain nudged = g;
        nudged.k[bad] += 0.05f;
        CHECK(gainMismatch(12.0f, 0.01f, w, nudged.k, P) > 1e-2f);
    }
}

CHAD_TEST(lqr, costToGoIsSymmetricAndPositiveDefinite) {
    const LqrWeights w{};
    for (float v : {2.0f, 12.0f, 80.0f}) {
        float P[9];
        solveLqr(v, 0.01f, w, 600, 1e-6f, P);
        for (int i = 0; i < 3; i++) {
            CHECK(P[i * 3 + i] > 0.0f);
            for (int j = 0; j < 3; j++) {
                const float scale = std::fmax(1.0f, std::fabs(P[i * 3 + j]));
                CHECK(std::fabs(P[i * 3 + j] - P[j * 3 + i]) < 1e-4f * scale);
            }
        }
        const float probes[4][3] = {
            {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, -2.0f, 0.5f}};
        for (const auto& x : probes) {
            float q = 0.0f;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) q += x[i] * P[i * 3 + j] * x[j];
            CHECK(q > 0.0f);
        }
    }
}

CHAD_TEST(lqr, closedLoopContractsEveryErrorAxis) {
    const LqrWeights w{};
    const float x0[3] = {6.0f, 4.0f, 0.4f};
    for (float v : {2.0f, 12.0f, 48.0f}) {
        const LqrGain g = solveLqr(v, 0.01f, w);
        CHECK(closedLoopDecay(v, 0.01f, g.k, x0, 2000) < 1e-3f);
    }
}

CHAD_TEST(lqr, crossTrackIsUnreachableAtRest) {
    // A(0) and B(0) both zero the e_y row, so no gain on it can do anything and
    // the optimal one is exactly zero. A solver that returns otherwise is
    // steering on an error it cannot affect.
    float P[9];
    const LqrGain g = solveLqr(0.0f, 0.01f, LqrWeights{}, 600, 1e-6f, P);
    CHECK_NEAR(g.k[1], 0.0f, 1e-6);
    CHECK_NEAR(g.k[4], 0.0f, 1e-6);
    for (int i = 0; i < 9; i++) CHECK(std::isfinite(P[i]));
}

CHAD_TEST(lqr, aLongSweepReachesTheInfiniteHorizonGain) {
    const LqrWeights w{};
    constexpr int kN = 200;
    float v[kN];
    for (int i = 0; i < kN; i++) v[i] = 24.0f;

    LqrGain sweep[kN];
    solveTvlqr(v, nullptr, kN, 0.01f, w, sweep);
    const LqrGain steady = solveLqr(24.0f, 0.01f, w);
    for (int i = 0; i < 6; i++) {
        CHECK_NEAR(sweep[0].k[i], steady.k[i], 0.02 * (std::fabs(steady.k[i]) + 1e-3));
    }
}

CHAD_TEST(lqr, scheduleLookupSurvivesItsEdges) {
    LqrGainSchedule s;
    CHECK(!s.ready());
    s.build(LqrWeights{}, 0.01f);
    CHECK(s.ready());

    const LqrGain top = s.at(80.0f);
    const LqrGain past = s.at(400.0f);
    const LqrGain low = s.at(0.0f);
    const LqrGain floorGain = s.at(2.0f);
    const LqrGain mirrored = s.at(-40.0f);
    const LqrGain forward = s.at(40.0f);
    const LqrGain nan = s.at(std::numeric_limits<float>::quiet_NaN());

    for (int i = 0; i < 6; i++) {
        CHECK_NEAR(past.k[i], top.k[i], 1e-6);          // clamped, not extrapolated
        CHECK_NEAR(low.k[i], floorGain.k[i], 1e-6);     // below the grid floor
        CHECK_NEAR(mirrored.k[i], forward.k[i], 1e-6);  // keyed on |v|
        CHECK(std::isfinite(nan.k[i]));
    }
}

CHAD_TEST(lqr, doublingAgreesWithAnIndependentValueIteration) {
    const LqrWeights w{};
    for (float v : {2.0f, 12.0f, 48.0f}) {
        float A[9], B[6];
        DrivetrainErrorPlant::discretise(v, 0.01f, A, B);
        float Q[3], R[2];
        lqrCosts(w, Q, R);

        double ref[6];
        CHECK(referenceDare(A, B, Q, R, ref));
        const LqrGain got = solveLqr(v, 0.01f, w, 40, 1e-7f);
        for (int i = 0; i < 6; i++) {
            CHECK_NEAR(got.k[i], static_cast<float>(ref[i]),
                       0.01 * (std::fabs(ref[i]) + 1e-3));
        }
    }
}

CHAD_TEST(lqr, theTwoPlantAndWeightConventionsAgree) {
    // lqr.cpp squares its weights and discretises through DrivetrainErrorPlant;
    // mpc_lqr.cpp does neither. Both now solve through riccati.hpp, so this no
    // longer cross-checks the solver -- it checks that the two spellings of the
    // same problem really are the same problem.
    const LqrWeights lw{};
    const MpcWeights mw{lw.qx * lw.qx, lw.qy * lw.qy, lw.qtheta * lw.qtheta,
                        lw.rv * lw.rv, lw.romega * lw.romega};

    for (float v : {2.0f, 12.0f, 48.0f}) {
        const LqrGain a = solveLqr(v, 0.01f, lw, 4000, 1e-7f);
        const LqrSolution b = solveDiscreteLqr(DrivetrainPlant::discretize(v, 0.01f), mw, 4000,
                                               1e-9f);
        CHECK(b.converged);
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 3; j++) {
                CHECK_NEAR(a.k[i * 3 + j], b.K[i][j],
                           0.01 * (std::fabs(b.K[i][j]) + 1e-3));
            }
    }
}
