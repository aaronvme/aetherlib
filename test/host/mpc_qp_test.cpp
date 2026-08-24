#include "chad/movement_control/qp.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

#include "chad_test.hpp"

using namespace chad;

namespace {

constexpr int QV = 6;
constexpr int QR = 12;
using Qp = AdmmQp<QV, QR>;

// ---------------------------------------------------------------------------
// Independent reference: the exact solution of a small inequality-constrained
// QP by active-set enumeration in double precision. Every row is tried
// inactive, at its lower bound and at its upper bound; each combination gives
// an equality-constrained KKT system, and the first candidate that is primal
// feasible with the right multiplier signs is the unique global optimum,
// because H is positive definite. Nothing here shares code with AdmmQp.
// ---------------------------------------------------------------------------

constexpr int RN = 4;
constexpr int RM = 6;

struct RefQp {
    int n{0};
    int m{0};
    double H[RN][RN]{};
    double q[RN]{};
    double C[RM][RN]{};
    double l[RM]{};
    double u[RM]{};
};

bool solveLinear(int n, double A[][RN + RM], double* b, double* x) {
    for (int c = 0; c < n; c++) {
        int p = c;
        for (int r = c + 1; r < n; r++) {
            if (std::fabs(A[r][c]) > std::fabs(A[p][c])) p = r;
        }
        if (std::fabs(A[p][c]) < 1e-12) return false;
        if (p != c) {
            for (int j = 0; j < n; j++) {
                const double t = A[p][j];
                A[p][j] = A[c][j];
                A[c][j] = t;
            }
            const double t = b[p];
            b[p] = b[c];
            b[c] = t;
        }
        for (int r = c + 1; r < n; r++) {
            const double f = A[r][c] / A[c][c];
            if (f == 0.0) continue;
            for (int j = c; j < n; j++) A[r][j] -= f * A[c][j];
            b[r] -= f * b[c];
        }
    }
    for (int r = n - 1; r >= 0; r--) {
        double s = b[r];
        for (int j = r + 1; j < n; j++) s -= A[r][j] * x[j];
        x[r] = s / A[r][r];
    }
    return true;
}

bool refSolve(const RefQp& p, double z[RN]) {
    int total = 1;
    for (int r = 0; r < p.m; r++) total *= 3;

    for (int code = 0; code < total; code++) {
        int act[RM]{}, side[RM]{};
        int na = 0;
        int c = code;
        for (int r = 0; r < p.m; r++) {
            const int st = c % 3;
            c /= 3;
            if (st != 0) {
                act[na] = r;
                side[na] = st;
                na++;
            }
        }
        if (na > p.n) continue;

        const int dim = p.n + na;
        double A[RN + RM][RN + RM]{};
        double b[RN + RM]{}, x[RN + RM]{};
        for (int i = 0; i < p.n; i++) {
            for (int j = 0; j < p.n; j++) A[i][j] = p.H[i][j];
            b[i] = -p.q[i];
        }
        for (int k = 0; k < na; k++) {
            for (int j = 0; j < p.n; j++) {
                A[j][p.n + k] = p.C[act[k]][j];
                A[p.n + k][j] = p.C[act[k]][j];
            }
            b[p.n + k] = (side[k] == 1) ? p.l[act[k]] : p.u[act[k]];
        }
        if (!solveLinear(dim, A, b, x)) continue;

        bool ok = true;
        for (int r = 0; r < p.m && ok; r++) {
            double cz = 0.0;
            for (int j = 0; j < p.n; j++) cz += p.C[r][j] * x[j];
            if (cz < p.l[r] - 1e-8 || cz > p.u[r] + 1e-8) ok = false;
        }
        // Multiplier convention: H z + q + sum lambda_r C_r = 0, so a row held
        // at its UPPER bound needs lambda >= 0 and one at its lower needs <= 0.
        for (int k = 0; k < na && ok; k++) {
            const double lam = x[p.n + k];
            if (side[k] == 2 && lam < -1e-8) ok = false;
            if (side[k] == 1 && lam > 1e-8) ok = false;
        }
        if (!ok) continue;

        for (int j = 0; j < p.n; j++) z[j] = x[j];
        return true;
    }
    return false;
}

void loadRef(Qp& qp, const RefQp& p, float H[QV][QV]) {
    for (int i = 0; i < QV; i++)
        for (int j = 0; j < QV; j++) H[i][j] = 0.0f;
    for (int i = 0; i < p.n; i++)
        for (int j = 0; j < p.n; j++) H[i][j] = static_cast<float>(p.H[i][j]);

    qp.begin(p.n);
    float row[QV]{};
    for (int r = 0; r < p.m; r++) {
        for (int j = 0; j < QV; j++) row[j] = 0.0f;
        for (int j = 0; j < p.n; j++) row[j] = static_cast<float>(p.C[r][j]);
        CHECK(qp.addRow(row) == r);
    }
    for (int r = 0; r < p.m; r++) {
        qp.lower()[r] = static_cast<float>(p.l[r]);
        qp.upper()[r] = static_cast<float>(p.u[r]);
    }
}

/// Verifies the KKT conditions of the convex QP directly, in the scaled row
/// units the solver reports. Primal feasibility + stationarity + the correct
/// multiplier signs with complementary slackness are together SUFFICIENT for
/// global optimality when H is positive semidefinite, so this proves the
/// returned iterate is optimal rather than merely self-consistent.
void checkKkt(Qp& qp, const float H[QV][QV], const float* q, float eps) {
    const int n = qp.vars();
    const int m = qp.rows();
    const float* z = qp.z();
    const float* y = qp.dual();

    float stat[QV]{};
    for (int i = 0; i < n; i++) {
        float s = q[i];
        for (int j = 0; j < n; j++) s += H[i][j] * z[j];
        stat[i] = s;
    }
    for (int r = 0; r < m; r++) {
        const float* row = qp.rowOf(r);
        for (int j = 0; j < n; j++) stat[j] += row[j] * y[r];
    }
    for (int i = 0; i < n; i++) CHECK_NEAR(stat[i], 0.0f, eps);

    for (int r = 0; r < m; r++) {
        const float* row = qp.rowOf(r);
        const float s = qp.rowScale(r);
        float cz = 0.0f;
        for (int j = 0; j < n; j++) cz += row[j] * z[j];
        const float lo = s * qp.lower()[r];
        const float hi = s * qp.upper()[r];
        CHECK(cz >= lo - eps);
        CHECK(cz <= hi + eps);
        if (y[r] > eps) CHECK_NEAR(cz, hi, 20.0f * eps);
        if (y[r] < -eps) CHECK_NEAR(cz, lo, 20.0f * eps);
        const bool interior = (cz > lo + 20.0f * eps) && (cz < hi - 20.0f * eps);
        if (interior) CHECK_NEAR(y[r], 0.0f, eps);
    }
}

}  // namespace

CHAD_TEST(mpcqp, projectsOntoAHalfSpaceExactly) {
    // min 1/2 ||z||^2 - c'z  s.t.  a'z <= b  has the closed form
    //   z* = c - max(0, (a'c - b) / ||a||^2) a
    // straight out of the geometry of a projection, with no QP theory in it.
    // `a` is deliberately far from unit norm so the row equilibration inside
    // the solver has something to get wrong.
    const float a[3] = {300.0f, -400.0f, 0.0f};
    const float b = -1000.0f;
    const float c[3] = {1.0f, 2.0f, -1.0f};

    float H[QV][QV]{};
    for (int i = 0; i < 3; i++) H[i][i] = 1.0f;
    const float q[QV] = {-c[0], -c[1], -c[2], 0.0f, 0.0f, 0.0f};

    Qp qp;
    qp.begin(3);
    float row[QV] = {a[0], a[1], a[2], 0.0f, 0.0f, 0.0f};
    CHECK_EQ(qp.addRow(row), 0);
    qp.lower()[0] = -1.0e9f;
    qp.upper()[0] = b;
    CHECK(qp.factorize(H, qp.suggestRho(H), 1e-6f));

    qp.resetState();
    const QpResult res = qp.solve(q, 2000, 1e-6f, 1e-6f, 1);
    CHECK(res.converged);

    float ac = 0.0f, aa = 0.0f;
    for (int i = 0; i < 3; i++) { ac += a[i] * c[i]; aa += a[i] * a[i]; }
    const float t = std::fmax(0.0f, (ac - b) / aa);
    CHECK(t > 0.0f);   // the constraint must actually bind or this proves nothing
    for (int i = 0; i < 3; i++) {
        CHECK_NEAR(qp.z()[i], c[i] - t * a[i], 2e-4);
    }
}

CHAD_TEST(mpcqp, matchesActiveSetEnumeration) {
    RefQp p;
    p.n = 3;
    p.m = 4;
    const double H[3][3] = {{4.0, 1.0, 0.5}, {1.0, 3.0, -0.7}, {0.5, -0.7, 2.5}};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) p.H[i][j] = H[i][j];
    const double q[3] = {-2.0, 5.0, 1.5};
    for (int i = 0; i < 3; i++) p.q[i] = q[i];

    // A box on two coordinates, a sum constraint and a difference constraint:
    // enough coupling that the wrong active set gives a visibly wrong answer.
    const double C[4][3] = {{1.0, 0.0, 0.0},
                            {0.0, 1.0, 0.0},
                            {1.0, 1.0, 1.0},
                            {1.0, -1.0, 0.0}};
    const double lo[4] = {-0.5, -2.0, -1.0, -0.3};
    const double hi[4] = {0.5, 2.0, 1.0, 0.3};
    for (int r = 0; r < 4; r++) {
        for (int j = 0; j < 3; j++) p.C[r][j] = C[r][j];
        p.l[r] = lo[r];
        p.u[r] = hi[r];
    }

    double ref[RN]{};
    CHECK(refSolve(p, ref));

    float Hf[QV][QV]{};
    Qp qp;
    loadRef(qp, p, Hf);
    CHECK(qp.factorize(Hf, qp.suggestRho(Hf), 1e-6f));

    float qf[QV]{};
    for (int i = 0; i < p.n; i++) qf[i] = static_cast<float>(p.q[i]);

    qp.resetState();
    const QpResult res = qp.solve(qf, 4000, 1e-6f, 1e-6f, 1);
    CHECK(res.converged);
    for (int i = 0; i < p.n; i++) CHECK_NEAR(qp.z()[i], ref[i], 3e-4);
    checkKkt(qp, Hf, qf, 3e-4f);
}

CHAD_TEST(mpcqp, matchesEnumerationOnAnEqualityConstraint) {
    RefQp p;
    p.n = 3;
    p.m = 3;
    for (int i = 0; i < 3; i++) p.H[i][i] = 1.0 + 0.5 * i;
    p.H[0][1] = p.H[1][0] = 0.3;
    const double q[3] = {1.0, -2.0, 0.5};
    for (int i = 0; i < 3; i++) p.q[i] = q[i];

    // Row 0 is an equality (l == u), which is the shape a terminal set takes.
    p.C[0][0] = 1.0; p.C[0][1] = 1.0; p.C[0][2] = 1.0;
    p.l[0] = p.u[0] = 0.75;
    p.C[1][2] = 1.0; p.l[1] = -0.2; p.u[1] = 0.2;
    p.C[2][0] = 1.0; p.l[2] = -3.0; p.u[2] = 3.0;

    double ref[RN]{};
    CHECK(refSolve(p, ref));

    float Hf[QV][QV]{};
    Qp qp;
    loadRef(qp, p, Hf);
    CHECK(qp.factorize(Hf, qp.suggestRho(Hf), 1e-6f));
    float qf[QV]{};
    for (int i = 0; i < p.n; i++) qf[i] = static_cast<float>(p.q[i]);

    qp.resetState();
    const QpResult res = qp.solve(qf, 8000, 1e-6f, 1e-6f, 1);
    CHECK(res.converged);
    for (int i = 0; i < p.n; i++) CHECK_NEAR(qp.z()[i], ref[i], 5e-4);

    float sum = 0.0f;
    for (int i = 0; i < p.n; i++) sum += qp.z()[i];
    CHECK_NEAR(sum, 0.75f, 5e-4);
}

CHAD_TEST(mpcqp, rowScalingDoesNotMoveTheSolution) {
    // The same feasible set written with a row a thousand times larger. If the
    // internal equilibration did not carry through to l, u, y and the residual,
    // these would disagree.
    float H[QV][QV]{};
    for (int i = 0; i < 2; i++) H[i][i] = 2.0f;
    H[0][1] = H[1][0] = 0.5f;
    const float q[QV] = {-1.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    float z0[2]{}, z1[2]{};
    for (int pass = 0; pass < 2; pass++) {
        const float k = (pass == 0) ? 1.0f : 1000.0f;
        Qp qp;
        qp.begin(2);
        float r0[QV] = {k, k * 0.5f, 0.0f, 0.0f, 0.0f, 0.0f};
        float r1[QV] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        CHECK_EQ(qp.addRow(r0), 0);
        CHECK_EQ(qp.addRow(r1), 1);
        qp.lower()[0] = -0.4f * k;
        qp.upper()[0] = 0.4f * k;
        qp.lower()[1] = -10.0f;
        qp.upper()[1] = 10.0f;
        CHECK(qp.factorize(H, qp.suggestRho(H), 1e-6f));
        qp.resetState();
        const QpResult res = qp.solve(q, 4000, 1e-6f, 1e-6f, 1);
        CHECK(res.converged);
        float* dst = (pass == 0) ? z0 : z1;
        dst[0] = qp.z()[0];
        dst[1] = qp.z()[1];
    }
    CHECK_NEAR(z0[0], z1[0], 1e-4);
    CHECK_NEAR(z0[1], z1[1], 1e-4);
}

CHAD_TEST(mpcqp, reportedResidualsMatchTheirDefinitions) {
    float H[QV][QV]{};
    for (int i = 0; i < 3; i++) H[i][i] = 1.0f + static_cast<float>(i);
    const float q[QV] = {2.0f, -1.0f, 0.5f, 0.0f, 0.0f, 0.0f};

    Qp qp;
    qp.begin(3);
    for (int i = 0; i < 3; i++) CHECK(qp.addUnitRow(i, 1.0f) == i);
    for (int r = 0; r < 3; r++) { qp.lower()[r] = -0.4f; qp.upper()[r] = 0.4f; }
    CHECK(qp.factorize(H, qp.suggestRho(H), 1e-6f));
    qp.resetState();

    // Two iterations is nowhere near optimal, which is the point: the reported
    // residuals must describe the iterate that was actually returned.
    const QpResult res = qp.solve(q, 2, 1e-9f, 1e-9f, 1);
    CHECK(!res.converged);
    CHECK(res.hitIterationCap);
    CHECK_EQ(res.iterations, 2);

    float pr = 0.0f;
    for (int r = 0; r < 3; r++) {
        float cz = 0.0f;
        for (int j = 0; j < 3; j++) cz += qp.rowOf(r)[j] * qp.z()[j];
        const float d = std::fabs(cz - qp.slack()[r]) / qp.rowScale(r);
        if (d > pr) pr = d;
    }
    float dr = 0.0f;
    for (int i = 0; i < 3; i++) {
        float s = q[i];
        for (int j = 0; j < 3; j++) s += H[i][j] * qp.z()[j];
        for (int r = 0; r < 3; r++) s += qp.rowOf(r)[i] * qp.dual()[r];
        const float d = std::fabs(s);
        if (d > dr) dr = d;
    }
    CHECK_NEAR(res.primalResidual, pr, 1e-6);
    CHECK_NEAR(res.dualResidual, dr, 1e-6);
    CHECK(res.primalResidual > 1e-9f || res.dualResidual > 1e-9f);
}

CHAD_TEST(mpcqp, capIsReportedAndNotMistakenForSuccess) {
    float H[QV][QV]{};
    for (int i = 0; i < 4; i++) H[i][i] = 1.0f;
    const float q[QV] = {50.0f, -60.0f, 30.0f, -20.0f, 0.0f, 0.0f};

    Qp qp;
    qp.begin(4);
    for (int i = 0; i < 4; i++) qp.addUnitRow(i, 1.0f);
    for (int r = 0; r < 4; r++) { qp.lower()[r] = -1.0f; qp.upper()[r] = 1.0f; }
    CHECK(qp.factorize(H, qp.suggestRho(H), 1e-6f));
    qp.resetState();

    const QpResult one = qp.solve(q, 1, 1e-9f, 1e-9f, 1);
    CHECK_EQ(one.iterations, 1);
    CHECK(!one.converged);
    CHECK(one.hitIterationCap);

    qp.resetState();
    const QpResult many = qp.solve(q, 5000, 1e-6f, 1e-6f, 1);
    CHECK(many.converged);
    CHECK(!many.hitIterationCap);
    for (int i = 0; i < 4; i++) {
        CHECK(qp.z()[i] >= -1.0f - 1e-4f);
        CHECK(qp.z()[i] <= 1.0f + 1e-4f);
    }
}

CHAD_TEST(mpcqp, warmStartCostsFewerIterations) {
    float H[QV][QV]{};
    for (int i = 0; i < 4; i++) H[i][i] = 2.0f + 0.3f * static_cast<float>(i);
    H[0][1] = H[1][0] = 0.6f;
    const float q[QV] = {3.0f, -4.0f, 1.0f, 2.0f, 0.0f, 0.0f};

    Qp qp;
    qp.begin(4);
    for (int i = 0; i < 4; i++) qp.addUnitRow(i, 1.0f);
    qp.addPairRow(1, 0);
    for (int r = 0; r < 4; r++) { qp.lower()[r] = -0.7f; qp.upper()[r] = 0.7f; }
    qp.lower()[4] = -0.1f;
    qp.upper()[4] = 0.1f;
    CHECK(qp.factorize(H, qp.suggestRho(H), 1e-6f));

    qp.resetState();
    const QpResult cold = qp.solve(q, 5000, 1e-6f, 1e-6f, 1);
    CHECK(cold.converged);
    const QpResult warm = qp.solve(q, 5000, 1e-6f, 1e-6f, 1);
    CHECK(warm.converged);
    CHECK(warm.iterations < cold.iterations);
    CHECK(cold.iterations > 1);
}

CHAD_TEST(mpcqp, shiftRowsMovesMultipliersAndRederivesTheSlackFromZ) {
    // Multipliers move with their rows; the slack does NOT move with them, it is
    // recomputed as C z. That is OSQP's warm start, and the difference is
    // visible on the rows `from` leaves behind: row 2 here has no counterpart,
    // so a shifted slack would keep a value belonging to the OLD primal.
    float H[QV][QV]{};
    for (int i = 0; i < 3; i++) H[i][i] = 1.0f;

    Qp qp;
    qp.begin(3);
    for (int i = 0; i < 3; i++) qp.addUnitRow(i, 2.0f);   // scaled rows, on purpose
    CHECK(qp.factorize(H, 1.0f, 1e-6f));
    qp.resetState();
    qp.z()[0] = 0.25f; qp.z()[1] = -0.50f; qp.z()[2] = 0.75f;
    qp.slack()[0] = 1.0f; qp.slack()[1] = 2.0f; qp.slack()[2] = 3.0f;
    qp.dual()[0] = -1.0f; qp.dual()[1] = -2.0f; qp.dual()[2] = -3.0f;

    const short from[3] = {1, 2, -1};
    qp.shiftRows(from);

    CHECK_EQ(qp.dual()[0], -2.0f);
    CHECK_EQ(qp.dual()[1], -3.0f);
    CHECK_EQ(qp.dual()[2], -3.0f);   // -1 keeps the tail

    // w == C z on EVERY row, including the one the map abandoned.
    for (int r = 0; r < 3; r++) {
        float cz = 0.0f;
        for (int j = 0; j < 3; j++) cz += qp.rowOf(r)[j] * qp.z()[j];
        CHECK_NEAR(qp.slack()[r], cz, 1e-7);
    }
    // And not the shifted values, which is what makes the check bite.
    CHECK(std::fabs(qp.slack()[0] - 2.0f) > 1e-3f);
    CHECK(std::fabs(qp.slack()[2] - 3.0f) > 1e-3f);
}

CHAD_TEST(mpcqp, detectsAnEmptyIntersectionOfTwoConsistentRows) {
    // Each row's own bounds are consistent; together they are empty. The
    // iterate must stay finite and something must say so -- ADMM on its own
    // just parks at a fixed point with a standing primal residual.
    float H[QV][QV]{};
    for (int i = 0; i < 2; i++) H[i][i] = 1.0f;
    const float q[QV] = {0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    Qp qp;
    qp.begin(2);
    CHECK_EQ(qp.addUnitRow(0, 1.0f), 0);
    CHECK_EQ(qp.addUnitRow(0, -3.0f), 1);   // same variable, opposite sign
    CHECK_EQ(qp.addUnitRow(1, 1.0f), 2);
    qp.lower()[0] = -3.6f; qp.upper()[0] = -0.4f;    // z0 in [-3.6, -0.4]
    qp.lower()[1] = -0.03f; qp.upper()[1] = 0.03f;   // -3 z0 in [-.03,.03]
    qp.lower()[2] = -1.0f; qp.upper()[2] = 1.0f;
    CHECK(qp.factorize(H, qp.suggestRho(H), 1e-6f));

    qp.resetState();
    const QpResult blind = qp.solve(q, 60, 1e-5f, 1e-5f, 0);   // no certification
    CHECK(blind.primalInfeasible);
    CHECK_EQ(blind.relaxedRows, 0);          // no single row's bounds crossed
    CHECK(blind.primalResidual > 0.0f);      // measured anyway, because it matters
    CHECK(!blind.converged);
    CHECK_FINITE(qp.z()[0]);

    // The same rows made compatible must NOT trip the detector.
    qp.lower()[1] = -12.0f; qp.upper()[1] = 12.0f;
    qp.resetState();
    const QpResult ok = qp.solve(q, 4000, 1e-5f, 1e-5f, 1);
    CHECK(!ok.primalInfeasible);
    CHECK(ok.converged);
}

CHAD_TEST(mpcqp, relativeDualToleranceScalesWithTheProblem) {
    // The dual residual is in gradient units. A fixed absolute tolerance is a
    // different demand on a problem scaled by 1000, and below the float32 noise
    // floor it is no demand that can be met at all.
    float H[QV][QV]{};
    for (int i = 0; i < 3; i++) H[i][i] = 1.0f + static_cast<float>(i);
    const float q[QV] = {2000.0f, -1000.0f, 500.0f, 0.0f, 0.0f, 0.0f};

    Qp qp;
    qp.begin(3);
    for (int i = 0; i < 3; i++) CHECK_EQ(qp.addUnitRow(i, 1.0f), i);
    for (int r = 0; r < 3; r++) { qp.lower()[r] = -400.0f; qp.upper()[r] = 400.0f; }
    CHECK(qp.factorize(H, qp.suggestRho(H), 1e-6f));

    // dualScale is max(||Hz||, ||C'y||, ||q||) at the reported iterate.
    qp.resetState();
    const QpResult two = qp.solve(q, 2, 1e-9f, 1e-9f, 1);
    float hz = 0.0f, cy = 0.0f, qn = 0.0f;
    for (int i = 0; i < 3; i++) {
        float s = 0.0f;
        for (int j = 0; j < 3; j++) s += H[i][j] * qp.z()[j];
        hz = std::fmax(hz, std::fabs(s));
        qn = std::fmax(qn, std::fabs(q[i]));
        float c = 0.0f;
        for (int r = 0; r < 3; r++) c += qp.rowOf(r)[i] * qp.dual()[r];
        cy = std::fmax(cy, std::fabs(c));
    }
    CHECK_NEAR(two.dualScale, std::fmax(hz, std::fmax(cy, qn)), 1e-2);
    CHECK(two.dualScale > 100.0f);

    // An absolute-only run cannot reach 1e-4 on a problem this size; the same
    // run with eps_rel = 1e-3 terminates on a criterion of the same shape.
    qp.resetState();
    const QpResult absolute = qp.solve(q, 3000, 1e-4f, 1e-4f, 1);
    qp.resetState();
    const QpResult relative = qp.solve(q, 3000, QpTolerance{.primal = 1e-4f, .dualAbs = 1e-4f, .dualRel = 1e-3f}, 1);
    CHECK(relative.converged);
    CHECK(relative.iterations < absolute.iterations);
    CHECK(relative.dualResidual <= 1e-4f + 1e-3f * relative.dualScale);
    // Loosening the DUAL must not have loosened the primal, which is the whole
    // reason the two are separate.
    CHECK(relative.primalResidual <= 1e-4f);
}

CHAD_TEST(mpcqp, rejectsAFactorizationItCannotTrust) {
    Qp qp;
    qp.begin(2);
    qp.addUnitRow(0, 1.0f);
    qp.addUnitRow(1, 1.0f);

    float indefinite[QV][QV]{};
    indefinite[0][0] = -5.0f;
    indefinite[1][1] = 1.0f;
    CHECK(!qp.factorize(indefinite, 1e-3f, 1e-6f));
    CHECK(!qp.factorized());

    float poisoned[QV][QV]{};
    poisoned[0][0] = std::nanf("");
    poisoned[1][1] = 1.0f;
    CHECK(!qp.factorize(poisoned, 1.0f, 1e-6f));

    float ok[QV][QV]{};
    ok[0][0] = 1.0f;
    ok[1][1] = 1.0f;
    CHECK(!qp.factorize(ok, 0.0f, 1e-6f));        // rho must be positive
    CHECK(!qp.factorize(nullptr, 1.0f, 1e-6f));
    CHECK(qp.factorize(ok, 1.0f, 1e-6f));

    // A solve without a factor reports the failure instead of emitting a guess.
    Qp bare;
    bare.begin(2);
    const float q[QV]{};
    const QpResult res = bare.solve(q, 10, 1e-6f, 1e-6f, 1);
    CHECK(res.factorizationFailed);
    CHECK(!res.converged);
    CHECK_EQ(res.iterations, 0);
}

CHAD_TEST(mpcqp, semidefiniteHessianStillFactorsThroughSigma) {
    // H = 0 is the degenerate end of the range: the objective is linear and the
    // constraints carry the whole problem. sigma + rho C'C has to keep the KKT
    // matrix definite.
    float H[QV][QV]{};
    const float q[QV] = {1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    Qp qp;
    qp.begin(2);
    qp.addUnitRow(0, 1.0f);
    qp.addUnitRow(1, 1.0f);
    qp.lower()[0] = -1.0f; qp.upper()[0] = 1.0f;
    qp.lower()[1] = -1.0f; qp.upper()[1] = 1.0f;
    CHECK(qp.factorize(H, 1.0f, 1e-6f));
    qp.resetState();
    const QpResult res = qp.solve(q, 4000, 1e-5f, 1e-5f, 1);
    CHECK(res.converged);
    // Linear objective on a box: the optimum is the opposite corner.
    CHECK_NEAR(qp.z()[0], -1.0f, 1e-3);
    CHECK_NEAR(qp.z()[1], 1.0f, 1e-3);
}

CHAD_TEST(mpcqp, crossedBoundsAreCollapsedNotDiverged) {
    float H[QV][QV]{};
    for (int i = 0; i < 2; i++) H[i][i] = 1.0f;
    const float q[QV] = {5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    Qp qp;
    qp.begin(2);
    qp.addUnitRow(0, 1.0f);
    qp.addUnitRow(1, 1.0f);
    qp.lower()[0] = 0.4f;      // deliberately above the upper bound
    qp.upper()[0] = -0.4f;
    qp.lower()[1] = -1.0f;
    qp.upper()[1] = 1.0f;
    CHECK(qp.factorize(H, qp.suggestRho(H), 1e-6f));
    qp.resetState();
    const QpResult res = qp.solve(q, 2000, 1e-5f, 1e-5f, 1);
    CHECK_EQ(res.relaxedRows, 1);
    CHECK(res.converged);
    CHECK_FINITE(qp.z()[0]);
    CHECK_NEAR(qp.z()[0], 0.0f, 1e-3);   // the midpoint of the crossed pair
}

CHAD_TEST(mpcqp, emptyAndOversizedRowsAreRefused) {
    Qp qp;
    qp.begin(3);
    float zeroRow[QV]{};
    CHECK_EQ(qp.addRow(zeroRow), -1);           // the empty-row branch
    // A NaN with a finite nonzero beside it is the case that reaches the
    // FINITENESS branch. A lone NaN does not: it leaves the peak at zero and is
    // refused as empty, so it tests nothing about the guard.
    float nanRow[QV] = {1.0f, std::nanf(""), 0.0f, 0.0f, 0.0f, 0.0f};
    CHECK_EQ(qp.addRow(nanRow), -1);
    float infRow[QV] = {0.5f, 0.0f, HUGE_VALF, 0.0f, 0.0f, 0.0f};
    CHECK_EQ(qp.addRow(infRow), -1);
    float loneNan[QV]{};
    loneNan[0] = std::nanf("");
    CHECK_EQ(qp.addRow(loneNan), -1);
    CHECK_EQ(qp.addUnitRow(3, 1.0f), -1);      // column out of range
    CHECK_EQ(qp.addUnitRow(0, 0.0f), -1);      // zero coefficient
    CHECK_EQ(qp.addPairRow(0, 0), -1);         // degenerate difference
    CHECK_EQ(qp.rows(), 0);

    for (int i = 0; i < QR; i++) CHECK(qp.addUnitRow(i % 3, 1.0f) >= 0);
    CHECK_EQ(qp.rows(), QR);
    CHECK_EQ(qp.addUnitRow(0, 1.0f), -1);      // budget spent, not overrun
}

CHAD_TEST(mpcqp, supportSpansCoverEveryNonzero) {
    // The matvecs only touch [begin, end), so a row whose span missed a nonzero
    // would silently drop that coupling. Compare a sparse pair row against the
    // same row entered densely.
    float H[QV][QV]{};
    for (int i = 0; i < 4; i++) H[i][i] = 1.0f;
    const float q[QV] = {2.0f, -3.0f, 1.0f, 0.5f, 0.0f, 0.0f};

    float sparseZ[4]{}, denseZ[4]{};
    for (int pass = 0; pass < 2; pass++) {
        Qp qp;
        qp.begin(4);
        if (pass == 0) {
            CHECK_EQ(qp.addPairRow(3, 0), 0);
        } else {
            float row[QV] = {-1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
            CHECK_EQ(qp.addRow(row), 0);
        }
        qp.lower()[0] = -0.5f;
        qp.upper()[0] = 0.5f;
        CHECK(qp.factorize(H, qp.suggestRho(H), 1e-6f));
        qp.resetState();
        CHECK(qp.solve(q, 4000, 1e-5f, 1e-5f, 1).converged);
        float* dst = (pass == 0) ? sparseZ : denseZ;
        for (int i = 0; i < 4; i++) dst[i] = qp.z()[i];
    }
    for (int i = 0; i < 4; i++) CHECK_NEAR(sparseZ[i], denseZ[i], 1e-5);
    // Unconstrained this difference would be -q_3 + q_0 = 1.5, so the upper
    // bound has to bind. A dropped nonzero would leave it at 1.5.
    CHECK_NEAR(sparseZ[3] - sparseZ[0], 0.5f, 1e-4);
}

CHAD_TEST(mpcqp, zeroPrimalToleranceDoesNotCertifyAFeasibleProblemInfeasible) {
    // tol.primal == 0 is what the RTI path uses to burn a fixed iteration
    // budget. The optimum here sits outside a bound, so that row stays active
    // and the primal residual floors at a small non-zero float32 value instead
    // of reaching exact zero -- so `converged` can never fire, the residual
    // stops decreasing, and that is indistinguishable from an empty feasible
    // set. The heuristic used to fire on it, and fired MORE often the longer the
    // solver ran: 9/400 ticks at 20 iterations, 296/400 at 160.
    // A tilted active row, not an axis-aligned one: C*z is then a float32 sum
    // that cannot land exactly on the bound, so the primal residual floors at a
    // small non-zero value and stops decreasing -- the condition the heuristic
    // misread. An axis-aligned box row reaches exact zero and converges, which
    // is why it does not reproduce this.
    float H[QV][QV]{};
    for (int i = 0; i < 3; i++) H[i][i] = 2.0f;
    H[0][1] = H[1][0] = 0.3f;
    const float q[QV] = {-9.7f, 2.1f, -3.3f, 0.0f, 0.0f, 0.0f};
    float tilted[QV]{};
    tilted[0] = 0.7f;
    tilted[1] = 1.3f;
    tilted[2] = -0.37f;

    for (int iters : {20, 80, 160, 400}) {
        Qp qp;
        qp.begin(3);
        for (int i = 0; i < 3; i++) {
            qp.addUnitRow(i, 1.0f);
            qp.lower()[i] = -1.0f;
            qp.upper()[i] = 1.0f;      // the optimum wants x0 = 5, so row 0 binds
        }
        const int rt = qp.addRow(tilted);
        CHECK(rt >= 0);
        qp.lower()[rt] = -0.31f;
        qp.upper()[rt] = 0.31f;        // binds too, and not on an axis
        CHECK(qp.factorize(H, qp.suggestRho(H), 1e-6f));
        qp.resetState();
        // tol.primal = 0 is what the RTI path passes to burn a fixed budget.
        const QpResult res = qp.solve(
            q, iters, QpTolerance{.primal = 0.0f, .dualAbs = 1e-4f, .dualRel = 1e-3f}, 4);
        CHECK(!res.primalInfeasible);
        CHECK(res.primalResidual < 1e-3f);
        CHECK(res.primalScale > 0.0f);
        CHECK_NEAR(qp.z()[0], 1.0f, 1e-3);     // clamped at the bound
        std::printf("      iters %3d: primal %.3e dual %.3e conv %d infeas %d\n",
                    iters, static_cast<double>(res.primalResidual),
                    static_cast<double>(res.dualResidual),
                    res.converged ? 1 : 0, res.primalInfeasible ? 1 : 0);
    }

    // The whole band the absolute-only tolerance broke on: anything below the
    // achievable float32 residual floor (~2.4e-7 here) could neither converge
    // nor be told apart from a stall, so gating the heuristic on
    // `tol.primal > 0` fixed exactly one value and left these misfiring.
    for (float tp : {0.0f, 1e-9f, 1e-8f, 1e-7f, 1e-6f}) {
        Qp qp;
        qp.begin(3);
        for (int i = 0; i < 3; i++) {
            qp.addUnitRow(i, 1.0f);
            qp.lower()[i] = -1.0f;
            qp.upper()[i] = 1.0f;
        }
        const int rt = qp.addRow(tilted);
        CHECK(rt >= 0);
        qp.lower()[rt] = -0.31f;
        qp.upper()[rt] = 0.31f;
        CHECK(qp.factorize(H, qp.suggestRho(H), 1e-6f));
        qp.resetState();
        const QpResult r = qp.solve(
            q, 400, QpTolerance{.primal = tp, .dualAbs = 1e-4f, .dualRel = 1e-3f}, 4);
        CHECK(!r.primalInfeasible);
    }

    // The heuristic must still catch a real empty set when given a real
    // tolerance. Two rows over the same two variables, so the cheap
    // single-variable interval test cannot see it: x0 + x1 cannot be both >= 2
    // and <= 1.
    Qp bad;
    bad.begin(2);
    float row[QV]{};
    row[0] = 1.0f;
    row[1] = 1.0f;
    const int ra = bad.addRow(row);
    const int rb = bad.addRow(row);
    CHECK(ra >= 0);
    CHECK(rb >= 0);
    bad.lower()[ra] = 2.0f;
    bad.upper()[ra] = 10.0f;
    bad.lower()[rb] = -10.0f;
    bad.upper()[rb] = 1.0f;
    float H2[QV][QV]{};
    for (int i = 0; i < 2; i++) H2[i][i] = 1.0f;
    const float q2[QV]{};
    CHECK(bad.factorize(H2, bad.suggestRho(H2), 1e-6f));
    bad.resetState();
    const QpResult res2 = bad.solve(q2, 400, QpTolerance{.primal = 1e-6f, .dualAbs = 1e-4f, .dualRel = 1e-3f}, 4);
    CHECK(res2.primalInfeasible);
}

// ===========================================================================
// The stall guard's noise floor. It fires when the dual has converged while the
// primal residual will not move, and it needs a floor below which "will not
// move" means "has hit float32 cancellation" rather than "is stuck outside an
// empty set".
//
// That floor used to be scaled by ||Cz||, the magnitude of the row dot product
// after it has summed. The error in the sum is bounded by the magnitude of the
// TERMS, and a row pinned near zero by a tight bound while the iterate is large
// cancels by orders of magnitude between the two -- so ||Cz|| understated the
// floor by exactly that factor and certified feasible problems infeasible.
//
// Every infeasible case here is empty over TWO OR MORE variables. Rows on a
// single variable are caught by the interval scan in solve() before any
// iteration runs, so using one would test the wrong mechanism.
// ===========================================================================
namespace {

struct GuardProblem {
    const char* name;
    bool feasible;
    int n;
    int rows;
    float C[QR][QV];
    float lo[QR];
    float hi[QR];
    float H[QV][QV];
    float q[QV];
};

void guardRow(GuardProblem& g, const float* a, float lo, float hi) {
    for (int j = 0; j < g.n; j++) g.C[g.rows][j] = a[j];
    g.lo[g.rows] = lo;
    g.hi[g.rows] = hi;
    g.rows++;
}

/// H = hdiag * I over `n` variables; the rows decide what is reachable.
GuardProblem guardCase(const char* name, bool feasible, int n, float hdiag) {
    GuardProblem g{};
    g.name = name;
    g.feasible = feasible;
    g.n = n;
    for (int i = 0; i < n; i++) g.H[i][i] = hdiag;
    return g;
}

QpResult runGuard(const GuardProblem& g, float eps, int checkEvery) {
    Qp qp;
    qp.begin(g.n);
    for (int r = 0; r < g.rows; r++) {
        if (qp.addRow(g.C[r]) != r) return QpResult{};
    }
    if (!qp.factorize(g.H, qp.suggestRho(g.H), 1e-6f)) return QpResult{};
    for (int r = 0; r < g.rows; r++) {
        qp.lower()[r] = g.lo[r];
        qp.upper()[r] = g.hi[r];
    }
    qp.resetState();
    return qp.solve(g.q, 400,
                    QpTolerance{.primal = eps, .dualAbs = eps, .dualRel = 1e-3f},
                    checkEvery);
}

}  // namespace

CHAD_TEST(mpcqp, stallGuardFloorFollowsTermMagnitudeNotResultMagnitude) {
    // A tilted active row: no axis reaches a bound, so the residual floors at
    // cancellation noise instead of the exact zero an axis-aligned box gives.
    // At 1e3 scale the iterate is large while the row is held near its bound,
    // which is the cancellation the floor has to survive.
    const float tilt[3] = {317.3f, 721.1f, 615.1f};
    const float tilt2[3] = {811.2f, -231.7f, 537.1f};
    GuardProblem big = guardCase("tilted x1e3", true, 3, 2.0f);
    for (int i = 0; i < 3; i++) big.q[i] = -40000.0f;
    guardRow(big, tilt, -1000.0f, 1000.0f);
    guardRow(big, tilt2, -1000.0f, 1000.0f);

    // z = 0 satisfies both rows, so this is feasible whatever the solver says.
    CHECK(0.0f >= big.lo[0] && 0.0f <= big.hi[0]);
    CHECK(0.0f >= big.lo[1] && 0.0f <= big.hi[1]);

    const QpResult r = runGuard(big, 1e-6f, 4);
    const float fEps = std::numeric_limits<float>::epsilon();
    std::printf("[guard] tilted x1e3: res=%.3e resultScale=%.3e termScale=%.3e "
                "oldFloor=%.3e newFloor=%.3e infeas=%d\n",
                static_cast<double>(r.primalResidual),
                static_cast<double>(r.primalScale),
                static_cast<double>(r.primalTermScale),
                static_cast<double>(8.0f * fEps * r.primalScale),
                static_cast<double>(8.0f * fEps * r.primalTermScale),
                r.primalInfeasible ? 1 : 0);

    CHECK(!r.primalInfeasible);
    // The two scales are not interchangeable, and this is why. If a change ever
    // makes them comparable on this problem the assertion above stops proving
    // anything, so the gap is asserted too.
    CHECK(r.primalTermScale > 100.0f * r.primalScale);
    // The old formula would have fired; the new one does not. Both directions,
    // so reverting the floor to primalScale reopens a named failure.
    CHECK(r.primalResidual > 8.0f * fEps * r.primalScale);
    CHECK(r.primalResidual <= 8.0f * fEps * r.primalTermScale);
}

CHAD_TEST(mpcqp, stallGuardVerdictHoldsAcrossToleranceAndCheckInterval) {
    const float tilt[3] = {317.3f, 721.1f, 615.1f};
    const float tilt2[3] = {811.2f, -231.7f, 537.1f};
    const float e0[3] = {1.0f, 0.0f, 0.0f};
    const float e1[3] = {0.0f, 1.0f, 0.0f};
    const float sum[3] = {1.0f, 1.0f, 0.0f};
    const float diff[3] = {1.0f, -1.0f, 0.0f};
    const float back[3] = {-1.0f, 1.0f, 0.0f};

    GuardProblem cases[8];
    int nc = 0;
    {   // Feasible, tilted vertex: two rows active at once, the worst float case.
        GuardProblem g = guardCase("tilted vertex", true, 3, 2.0f);
        for (int i = 0; i < 3; i++) g.q[i] = -40.0f;
        guardRow(g, tilt, -1.0f, 1.0f);
        guardRow(g, tilt2, -1.0f, 1.0f);
        cases[nc++] = g;
    }
    {   // The same shape at 1e3, where the cancellation is three decades deep.
        GuardProblem g = guardCase("tilted x1e3", true, 3, 2.0f);
        for (int i = 0; i < 3; i++) g.q[i] = -40000.0f;
        guardRow(g, tilt, -1000.0f, 1000.0f);
        guardRow(g, tilt2, -1000.0f, 1000.0f);
        cases[nc++] = g;
    }
    {   // Ill-conditioned H under a tilted active row: suggestRho averages the
        // diagonal, so a 1e7 spread in it puts rho far from anything suitable
        // and the residual floors high.
        GuardProblem g = guardCase("illcond H", true, 3, 1.0f);
        g.H[0][0] = 1e4f; g.H[1][1] = 1.0f; g.H[2][2] = 1e-3f;
        g.q[0] = -1e4f; g.q[1] = -30.0f; g.q[2] = -0.03f;
        guardRow(g, tilt, -1.0f, 1.0f);
        cases[nc++] = g;
    }
    {   // FEASIBLE, and the shape that reads as if it were not: x0 in [2, 4]
        // with x0 + x1 in [-1, -0.5] is solved by x0 = 2, x1 = -3.
        GuardProblem g = guardCase("near miss", true, 2, 2.0f);
        guardRow(g, e0, 2.0f, 4.0f);
        guardRow(g, sum, -1.0f, -0.5f);
        cases[nc++] = g;
    }
    {   // Each variable's own box is satisfiable; their sum is not.
        GuardProblem g = guardCase("sum vs box", false, 2, 2.0f);
        guardRow(g, e0, 0.0f, 1.0f);
        guardRow(g, e1, 0.0f, 1.0f);
        guardRow(g, sum, 3.0f, 4.0f);
        cases[nc++] = g;
    }
    {   // Three rows whose combination forces x0 >= 1 against x0 <= 0.1:
        // x0+x1 >= 1 and x0-x1 >= 1 give 2*x0 >= 2. No single row is empty.
        GuardProblem g = guardCase("combination", false, 2, 2.0f);
        guardRow(g, sum, 1.0f, 5.0f);
        guardRow(g, diff, 1.0f, 5.0f);
        guardRow(g, e0, -0.1f, 0.1f);
        cases[nc++] = g;
    }
    {   // The shape a rate limit off an unreachable anchor makes: both inputs
        // inside their box, but the step between them wider than the box spans.
        GuardProblem g = guardCase("rate vs box", false, 2, 2.0f);
        guardRow(g, e0, -1.0f, 1.0f);
        guardRow(g, e1, -1.0f, 1.0f);
        guardRow(g, back, 5.0f, 6.0f);
        cases[nc++] = g;
    }
    {   // Tilted and empty at 1e3 scale: a large term magnitude raises the floor,
        // so this is where a genuine violation is most at risk of hiding under it.
        GuardProblem g = guardCase("tilted x1e3 empty", false, 3, 2.0f);
        guardRow(g, tilt, 1000.0f, 2000.0f);
        guardRow(g, tilt, -2000.0f, -1000.0f);
        cases[nc++] = g;
    }

    for (int i = 0; i < nc; i++) {
        const GuardProblem& g = cases[i];
        for (float eps : {1e-9f, 1e-8f, 1e-7f, 1e-6f, 1e-5f, 1e-4f}) {
            const QpResult r = runGuard(g, eps, 4);
            if (r.primalInfeasible == g.feasible) {
                std::printf("[guard] %s eps=%.0e flagged=%d expected=%d "
                            "res=%.3e termScale=%.3e\n",
                            g.name, static_cast<double>(eps),
                            r.primalInfeasible ? 1 : 0, g.feasible ? 0 : 1,
                            static_cast<double>(r.primalResidual),
                            static_cast<double>(r.primalTermScale));
            }
            CHECK(r.primalInfeasible == !g.feasible);
        }
        // The guard needs four consecutive stalled checks, so the check interval
        // changes when it can fire at all. It must not change the verdict.
        for (int ce : {1, 2, 4, 8, 16, 32}) {
            const QpResult r = runGuard(g, 1e-6f, ce);
            CHECK(r.primalInfeasible == !g.feasible);
        }
    }
    std::printf("[guard] %d shapes x 6 tolerances x 6 check intervals agreed\n", nc);
}

// A tolerance is a tolerance: two intervals that miss by less than eps describe
// a set the solver is entitled to call satisfied, and it does. Recorded so the
// boundary is a decision rather than a surprise.
CHAD_TEST(mpcqp, stallGuardStopsCallingItInfeasibleOnceEpsExceedsTheGap) {
    const float sum[3] = {1.0f, 1.0f, 0.0f};
    GuardProblem g = guardCase("gap 1e-3", false, 2, 2.0f);
    guardRow(g, sum, 0.0f, 1.0f);
    guardRow(g, sum, 1.001f, 2.0f);

    const QpResult tight = runGuard(g, 1e-6f, 4);
    CHECK(tight.primalInfeasible);
    CHECK(!tight.converged);

    const QpResult loose = runGuard(g, 1e-3f, 4);
    std::printf("[guard] gap 1e-3: eps 1e-6 -> infeas %d, eps 1e-3 -> infeas %d "
                "conv %d res %.3e\n",
                tight.primalInfeasible ? 1 : 0, loose.primalInfeasible ? 1 : 0,
                loose.converged ? 1 : 0, static_cast<double>(loose.primalResidual));
    CHECK(!loose.primalInfeasible);
    CHECK(loose.converged);
    // Because the residual it settled at is inside the tolerance it was given,
    // which is the whole content of "solved to 1e-3".
    CHECK(loose.primalResidual <= 1e-3f);
    CHECK(loose.primalResidual > 1e-4f);
}

// Per-row rho: the weight has to reach the row it names, and must not survive a
// rebuild that changes what the rows mean.
CHAD_TEST(mpcqp, rowRhoWeightAppliesPerRowAndIsClearedByBegin) {
    Qp qp;
    qp.begin(3);
    const float e0[3] = {1.0f, 0.0f, 0.0f};
    const float e1[3] = {0.0f, 1.0f, 0.0f};
    const float e2[3] = {0.0f, 0.0f, 1.0f};
    CHECK_EQ(qp.addRow(e0), 0);
    CHECK_EQ(qp.addRow(e1), 1);
    CHECK_EQ(qp.addRow(e2), 2);

    float H[QV][QV]{};
    for (int i = 0; i < 3; i++) H[i][i] = 2.0f;

    // A short count, a non-positive entry and a non-finite one all mean 1, so a
    // partly filled array cannot silently zero a row's penalty.
    const float w[3] = {0.25f, -1.0f, std::numeric_limits<float>::quiet_NaN()};
    qp.setRowRhoWeight(w, 3);
    CHECK(qp.factorize(H, 4.0f, 1e-6f));
    CHECK_NEAR(qp.rhoOfRow(0), 1.0f, 1e-6);
    CHECK_NEAR(qp.rhoOfRow(1), 4.0f, 1e-6);
    CHECK_NEAR(qp.rhoOfRow(2), 4.0f, 1e-6);
    CHECK_NEAR(qp.rho(), 4.0f, 1e-6);

    // Row 0 is penalised at a quarter of rho, so the solve it produces differs
    // from the unweighted one -- otherwise the weight would be decorative.
    const float q[QV] = {-8.0f, -8.0f, -8.0f, 0.0f, 0.0f, 0.0f};
    for (int r = 0; r < 3; r++) { qp.lower()[r] = -1.0f; qp.upper()[r] = 1.0f; }
    qp.resetState();
    const QpResult weighted = qp.solve(q, 3, QpTolerance{0.0f, 0.0f, 0.0f}, 0);
    const float z0 = qp.z()[0];

    qp.setRowRhoWeight(nullptr, 0);
    CHECK(qp.factorize(H, 4.0f, 1e-6f));
    CHECK_NEAR(qp.rhoOfRow(0), 4.0f, 1e-6);
    qp.resetState();
    qp.solve(q, 3, QpTolerance{0.0f, 0.0f, 0.0f}, 0);
    std::printf("[rowrho] weighted z0=%.6f unweighted z0=%.6f (iters %d)\n",
                static_cast<double>(z0), static_cast<double>(qp.z()[0]),
                weighted.iterations);
    CHECK(std::fabs(z0 - qp.z()[0]) > 1e-4f);

    // begin() drops it: after a rebuild, row 0 is a different constraint.
    const float w2[1] = {0.25f};
    qp.setRowRhoWeight(w2, 1);
    qp.begin(3);
    CHECK_EQ(qp.addRow(e0), 0);
    CHECK(qp.factorize(H, 4.0f, 1e-6f));
    CHECK_NEAR(qp.rhoOfRow(0), 4.0f, 1e-6);
}
