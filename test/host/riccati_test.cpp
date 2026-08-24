#include "chad/movement_control/riccati.hpp"

#include "chad/movement_control/mpc_plant.hpp"
#include "chad_test.hpp"

using namespace chad;

namespace {

/// Value iteration, the slow-but-obvious reference the doubling must agree with.
template <int N, int M>
void valueIterate(const float A[N][N], const float B[N][M], const float Q[N][N],
                  const float R[M][M], int sweeps, float P[N][N]) {
    linalg::copy<N, N>(Q, P);
    for (int it = 0; it < sweeps; it++) {
        float K[M][N];
        dareGain<N, M>(A, B, R, P, K);

        float pA[N][N];
        linalg::multiply<N, N, N>(P, A, pA);
        float atPA[N][N];
        linalg::multiplyTransposedA<N, N, N>(A, pA, atPA);

        float btPA[M][N];
        {
            float btP[M][N];
            linalg::multiplyTransposedA<M, N, N>(B, P, btP);
            linalg::multiply<M, N, N>(btP, A, btPA);
        }
        float corr[N][N];
        linalg::multiplyTransposedA<N, N, M>(btPA, K, corr);

        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++) P[i][j] = atPA[i][j] - corr[i][j] + Q[i][j];
        linalg::symmetrise<N>(P);
    }
}

}  // namespace

CHAD_TEST(linalg, inverseRoundTrips) {
    const float m[3][3] = {{4.0f, 7.0f, 2.0f}, {3.0f, 6.0f, 1.0f}, {2.0f, 5.0f, 3.0f}};
    float inv[3][3];
    CHECK((linalg::inverse<3>(m, inv)));

    float product[3][3];
    linalg::multiply<3, 3, 3>(m, inv, product);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) CHECK_NEAR(product[i][j], i == j ? 1.0f : 0.0f, 1e-4);
}

CHAD_TEST(linalg, inverseNeedsPivoting) {
    const float m[3][3] = {{0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 2.0f}};
    float inv[3][3];
    CHECK((linalg::inverse<3>(m, inv)));
    CHECK_NEAR(inv[0][1], 1.0f, 1e-6);
    CHECK_NEAR(inv[1][0], 1.0f, 1e-6);
    CHECK_NEAR(inv[2][2], 0.5f, 1e-6);
}

CHAD_TEST(linalg, singularInverseIsReported) {
    const float m[2][2] = {{1.0f, 2.0f}, {2.0f, 4.0f}};
    float inv[2][2];
    CHECK(!linalg::inverse<2>(m, inv));
    CHECK_EQ(inv[0][0], 1.0f);
    CHECK_EQ(inv[0][1], 0.0f);
}

CHAD_TEST(linalg, transposedProductsMatchManualForm) {
    const float a[3][2] = {{1.0f, 2.0f}, {3.0f, 4.0f}, {5.0f, 6.0f}};
    const float b[3][2] = {{7.0f, 8.0f}, {9.0f, 10.0f}, {11.0f, 12.0f}};
    float atb[2][2];
    linalg::multiplyTransposedA<2, 2, 3>(a, b, atb);
    CHECK_NEAR(atb[0][0], 1 * 7 + 3 * 9 + 5 * 11, 1e-4);
    CHECK_NEAR(atb[1][1], 2 * 8 + 4 * 10 + 6 * 12, 1e-4);

    const float c[2][3] = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
    float abt[2][2];
    linalg::multiplyTransposedB<2, 2, 3>(c, c, abt);
    CHECK_NEAR(abt[0][0], 14.0f, 1e-4);
    CHECK_NEAR(abt[0][1], 32.0f, 1e-4);
}

CHAD_TEST(linalg, spectralRadiusOfDiagonal) {
    const float m[3][3] = {{0.5f, 0.0f, 0.0f}, {0.0f, -0.9f, 0.0f}, {0.0f, 0.0f, 0.2f}};
    CHECK_NEAR((linalg::spectralRadius<3>(m)), 0.9f, 1e-3);
}

CHAD_TEST(dare, scalarCaseMatchesGoldenRatio) {
    const float A[1][1] = {{1.0f}};
    const float B[1][1] = {{1.0f}};
    const float Q[1][1] = {{1.0f}};
    const float R[1][1] = {{1.0f}};
    const DareSolution<1, 1> sol = solveDare<1, 1>(A, B, Q, R);
    CHECK(sol.converged);
    CHECK_NEAR(sol.P[0][0], 1.6180339887f, 1e-4);
    CHECK_NEAR(sol.K[0][0], 1.6180339887f / 2.6180339887f, 1e-4);
    CHECK(sol.residual < 1e-3f);
}

CHAD_TEST(dare, convergesQuadratically) {
    const float A[1][1] = {{1.0f}};
    const float B[1][1] = {{1.0f}};
    const float Q[1][1] = {{1.0f}};
    const float R[1][1] = {{1.0f}};
    const DareSolution<1, 1> sol = solveDare<1, 1>(A, B, Q, R);
    CHECK(sol.iterations <= 12);
}

CHAD_TEST(dare, zeroCostGivesZeroSolution) {
    const float A[2][2] = {{0.9f, 0.1f}, {0.0f, 0.8f}};
    const float B[2][1] = {{0.0f}, {1.0f}};
    const float Q[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
    const float R[1][1] = {{1.0f}};
    const DareSolution<2, 1> sol = solveDare<2, 1>(A, B, Q, R);
    CHECK(sol.converged);
    CHECK_NEAR((linalg::maxAbs<2, 2>(sol.P)), 0.0f, 1e-6);
}

CHAD_TEST(dare, agreesWithValueIteration) {
    const float A[3][3] = {{1.0f, 0.02f, 0.0f}, {0.0f, 0.95f, 0.05f}, {0.0f, 0.0f, 0.9f}};
    const float B[3][2] = {{0.01f, 0.0f}, {0.0f, 0.02f}, {0.0f, 0.05f}};
    const float Q[3][3] = {{2.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.5f}};
    const float R[2][2] = {{0.4f, 0.0f}, {0.0f, 0.7f}};

    const DareSolution<3, 2> sol = solveDare<3, 2>(A, B, Q, R, 60, 1e-9f);
    CHECK(sol.converged);

    float reference[3][3];
    valueIterate<3, 2>(A, B, Q, R, 6000, reference);

    const float scale = linalg::maxAbs<3, 3>(reference);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) CHECK_NEAR(sol.P[i][j], reference[i][j], 2e-3 * scale);
}

CHAD_TEST(dare, solutionIsSymmetricAndStabilising) {
    const float A[3][3] = {{1.05f, 0.02f, 0.0f}, {0.0f, 1.0f, 0.04f}, {0.0f, 0.0f, 1.0f}};
    const float B[3][2] = {{0.01f, 0.0f}, {0.0f, 0.01f}, {0.0f, 0.03f}};
    const float Q[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    const float R[2][2] = {{0.5f, 0.0f}, {0.0f, 0.5f}};

    const DareSolution<3, 2> sol = solveDare<3, 2>(A, B, Q, R);
    CHECK(sol.converged);
    CHECK(sol.residual < 1e-2f);

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) CHECK_NEAR(sol.P[i][j], sol.P[j][i], 1e-5);

    float cl[3][3];
    closedLoop<3, 2>(A, B, sol.K, cl);
    CHECK((linalg::spectralRadius<3>(cl)) < 0.9999f);
}

CHAD_TEST(dare, nonDiagonalWeightsAreHonoured) {
    const float A[2][2] = {{1.0f, 0.05f}, {0.0f, 1.0f}};
    const float B[2][2] = {{0.05f, 0.01f}, {0.01f, 0.05f}};
    const float Q[2][2] = {{2.0f, 0.5f}, {0.5f, 1.0f}};
    const float R[2][2] = {{0.3f, 0.1f}, {0.1f, 0.4f}};

    const DareSolution<2, 2> sol = solveDare<2, 2>(A, B, Q, R, 60, 1e-9f);
    CHECK(sol.converged);
    CHECK(sol.residual < 1e-2f);

    float reference[2][2];
    valueIterate<2, 2>(A, B, Q, R, 8000, reference);
    const float scale = linalg::maxAbs<2, 2>(reference);
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++) CHECK_NEAR(sol.P[i][j], reference[i][j], 3e-3 * scale);
}

CHAD_TEST(dare, uncontrollableStateIsReportedNotFaked) {
    // The pose-error plant at zero reference speed: cross-track cannot be
    // reached by any input, so no stabilising cost-to-go exists.
    const DrivetrainPlant plant = DrivetrainPlant::discretize(0.0f, 0.01f);
    const float Q[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    const float R[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};

    const DareSolution<3, 2> sol = solveDare<3, 2>(plant.Ad, plant.Bd, Q, R);
    CHECK(!sol.converged);
    CHECK_FINITE((linalg::maxAbs<3, 3>(sol.P)));
    CHECK_FINITE((linalg::maxAbs<2, 3>(sol.K)));
}

CHAD_TEST(dare, movingPlantIsControllableAndStable) {
    const DrivetrainPlant plant = DrivetrainPlant::discretize(1.5f, 0.01f);
    const float Q[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 2.0f}};
    const float R[2][2] = {{0.6f, 0.0f}, {0.0f, 0.4f}};

    const DareSolution<3, 2> sol = solveDare<3, 2>(plant.Ad, plant.Bd, Q, R);
    CHECK(sol.converged);
    CHECK(sol.residual < 1e-2f);

    float cl[3][3];
    closedLoop<3, 2>(plant.Ad, plant.Bd, sol.K, cl);
    CHECK((linalg::spectralRadius<3>(cl)) < 1.0f);
}

CHAD_TEST(dare, singularInputWeightDoesNotProduceGarbage) {
    const float A[2][2] = {{1.0f, 0.1f}, {0.0f, 1.0f}};
    const float B[2][1] = {{0.0f}, {0.1f}};
    const float Q[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
    const float R[1][1] = {{0.0f}};

    const DareSolution<2, 1> sol = solveDare<2, 1>(A, B, Q, R);
    CHECK_FINITE((linalg::maxAbs<2, 2>(sol.P)));
    CHECK_FINITE((linalg::maxAbs<1, 2>(sol.K)));
}

CHAD_TEST(dare, gainMatchesTheDefinitionGivenP) {
    const float A[2][2] = {{1.0f, 0.05f}, {0.0f, 0.98f}};
    const float B[2][1] = {{0.0f}, {0.05f}};
    const float Q[2][2] = {{1.0f, 0.0f}, {0.0f, 0.2f}};
    const float R[1][1] = {{0.5f}};

    const DareSolution<2, 1> sol = solveDare<2, 1>(A, B, Q, R);
    CHECK(sol.converged);

    float btP[1][2];
    linalg::multiplyTransposedA<1, 2, 2>(B, sol.P, btP);
    float m[1][1];
    linalg::multiply<1, 1, 2>(btP, B, m);
    m[0][0] += R[0][0];
    float btPA[1][2];
    linalg::multiply<1, 2, 2>(btP, A, btPA);
    CHECK_NEAR(sol.K[0][0], btPA[0][0] / m[0][0], 1e-4);
    CHECK_NEAR(sol.K[0][1], btPA[0][1] / m[0][0], 1e-4);
}
