#include "chad/tracking/mcl_core.hpp"

#include <array>
#include <atomic>
#include <cstdio>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "chad/tracking/mcl_adapter.hpp"
#include "chad_test.hpp"

using namespace chad;

namespace {

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

using Cfg = MclCore::Config;
using Line = MclCore::LineObstacle;
using Circle = MclCore::CircleObstacle;

/// ~70 KB of fixed particle buffers, so never on the stack.
std::unique_ptr<MclCore> makeCore(const Cfg& cfg, std::uint32_t seed = 0xC0FFEEu) {
    return std::make_unique<MclCore>(cfg, seed);
}

/// Every noise term off, so propagation and weighting are deterministic.
Cfg silentConfig(int particles = 1) {
    Cfg c;
    c.particleCount = particles;
    c.maxSensorRange = 140.0f;
    c.distResampleVariance = 0.0f;
    c.thetaResampleVariance = 0.0f;
    c.trackingWheelVariance = 0.0f;
    c.imeVariance = 0.0f;
    c.imuVariance = 0.0f;
    c.horizDependentVarianceProp = 0.0f;
    c.horizConstantNoise = 0.0f;
    c.vertConstantNoise = 0.0f;
    c.distSearchVariance = 0.0f;
    c.thetaSearchVariance = 0.0f;
    c.maxThetaDeviation = 100.0f;   // clamp inert
    return c;
}

/// sqrt(1/2), spelled once.
float d0() { return std::sqrt(0.5f); }

/// A square of side 2*half centred on the origin, wound counter-clockwise.
std::vector<Line> square(float half) {
    return {{{-half, -half}, {half, -half}},
            {{half, -half}, {half, half}},
            {{half, half}, {-half, half}},
            {{-half, half}, {-half, -half}}};
}

/// A live IMU. Pass hasImu = false for the faulted or absent case, where the
/// heading the adapter would hand over is a held value and not a measurement.
MclCore::MotionInput motion(float dVert, float dHoriz, float dTheta, float imu,
                            bool hasHoriz = true, bool hasImu = true) {
    MclCore::MotionInput in;
    in.delta = {dVert, dHoriz, dTheta};
    in.imuHeadingRad = imu;
    in.hasImu = hasImu;
    in.hasVertWheel = true;
    in.hasHorizWheel = hasHoriz;
    in.dtSeconds = 0.025f;
    return in;
}

/// Robot-frame mounts 7 in out on each face, each pointing outward.
std::array<Pose, MclCore::SENSOR_COUNT> fourFaceMounts() {
    std::array<Pose, MclCore::SENSOR_COUNT> m{};
    m[0] = {7.0f, 0.0f, 0.0f};
    m[1] = {0.0f, 7.0f, HALF_PI};
    m[2] = {-7.0f, 0.0f, PI};
    m[3] = {0.0f, -7.0f, -HALF_PI};
    return m;
}

/// Independent sensor model: the slab form of a ray against the VRC rectangle,
/// in double, written from the geometry and not from mcl_core.cpp. Generating
/// the synthetic readings with MclCore::expectedRange() -- which is what this
/// file used to do -- makes every whole-filter test below a check of the
/// implementation against itself, so a self-consistent ray-cast or mount
/// transform error would be invisible to all of them.
double refRangeAt(double px, double py, double th, double mx, double my, double myaw,
                  double half, double maxRange) {
    const double sx = px + mx * std::cos(th) - my * std::sin(th);
    const double sy = py + mx * std::sin(th) + my * std::cos(th);
    const double dx = std::cos(th + myaw), dy = std::sin(th + myaw);
    double best = maxRange;
    for (int s = -1; s <= 1; s += 2) {
        const double wall = static_cast<double>(s) * half;
        if (std::fabs(dx) > 1e-12) {
            const double t = (wall - sx) / dx;
            const double yy = sy + t * dy;
            if (t >= 0.0 && t < best && yy >= -half && yy <= half) best = t;
        }
        if (std::fabs(dy) > 1e-12) {
            const double t = (wall - sy) / dy;
            const double xx = sx + t * dx;
            if (t >= 0.0 && t < best && xx >= -half && xx <= half) best = t;
        }
    }
    return best;
}

float refRange(const Pose& pose, int sensor,
               const std::array<Pose, MclCore::SENSOR_COUNT>& mounts,
               float maxRange = 140.0f) {
    return static_cast<float>(refRangeAt(pose.x, pose.y, pose.theta, mounts[sensor].x,
                                         mounts[sensor].y, mounts[sensor].theta,
                                         Field::FIELD_HALF_WALL, maxRange));
}

float refRange(const Pose& pose, int sensor, float maxRange = 140.0f) {
    return refRange(pose, sensor, fourFaceMounts(), maxRange);
}

/// Synthetic readings from `truth`, in the RawSensor form the filter consumes,
/// off the independent model above. `noiseIn` is the standard deviation of the
/// Gaussian range noise; 0 gives the exact ranges.
void senseFrom(const Pose& truth, int sensors, MclCore::RawSensor* out,
               float maxRange = 140.0f, Pcg32* noise = nullptr, float noiseIn = 0.0f) {
    for (int i = 0; i < MclCore::SENSOR_COUNT; i++) out[i] = {};
    for (int i = 0; i < sensors; i++) {
        float range = refRange(truth, i, 1.0e6f);
        if (noise != nullptr && noiseIn > 0.0f) range += noise->normal() * noiseIn;
        // Past its range a V5 reports ~9999 mm, which the filter drops. Reporting
        // maxRange as though it were a reading -- which is what clamping the
        // model to maxRange does -- hands every particle a residual of zero on a
        // beam that measured nothing.
        out[i].mm = (range >= maxRange) ? 9999
                                        : static_cast<int>(range * INCH_TO_MM + 0.5f);
        out[i].confidence = 63;
        out[i].present = true;
    }
}

}  // namespace

// ===========================================================================
// RNG
// ===========================================================================

CHAD_TEST(mclRng, uniformIsInRangeAndReproducible) {
    Pcg32 a(7u);
    Pcg32 b(7u);
    Pcg32 c(8u);

    double sum = 0.0;
    int differsFromOtherSeed = 0;
    for (int i = 0; i < 100000; i++) {
        const float x = a.uniform();
        CHECK(x >= 0.0f);
        CHECK(x < 1.0f);
        CHECK_EQ(x, b.uniform());
        if (x != c.uniform()) differsFromOtherSeed++;
        sum += x;
    }
    CHECK_NEAR(sum / 100000.0, 0.5, 0.005);
    CHECK(differsFromOtherSeed > 99000);
}

CHAD_TEST(mclRng, normalHasUnitMomentsAndTails) {
    Pcg32 g(1234u);
    const int n = 400000;
    double sum = 0.0, sumSq = 0.0, sum4 = 0.0;
    int within1 = 0, within2 = 0, within3 = 0;
    for (int i = 0; i < n; i++) {
        const float z = g.normal();
        CHECK_FINITE(z);
        sum += z;
        sumSq += static_cast<double>(z) * z;
        sum4 += std::pow(static_cast<double>(z), 4.0);
        const float a = std::fabs(z);
        if (a < 1.0f) within1++;
        if (a < 2.0f) within2++;
        if (a < 3.0f) within3++;
    }
    CHECK_NEAR(sum / n, 0.0, 0.01);
    CHECK_NEAR(sumSq / n, 1.0, 0.01);
    // Kurtosis 3 is what separates a real normal from the sum-of-uniforms
    // shortcut, which comes out near 2.4 and has no tails to speak of.
    CHECK_NEAR(sum4 / n, 3.0, 0.08);
    CHECK_NEAR(static_cast<double>(within1) / n, 0.682689, 0.005);
    CHECK_NEAR(static_cast<double>(within2) / n, 0.954500, 0.003);
    CHECK_NEAR(static_cast<double>(within3) / n, 0.997300, 0.001);
}

CHAD_TEST(mclRng, seedChangesTheStream) {
    Pcg32 g(1u);
    const float first = g.normal();
    g.reseed(1u);
    CHECK_EQ(first, g.normal());
    g.reseed(2u);
    CHECK(g.normal() != first);
}

// ===========================================================================
// Ray casting
// ===========================================================================

CHAD_TEST(mclRaycast, wallsAndCornersOfTheVrcField) {
    Cfg c = silentConfig();
    auto core = makeCore(c);

    // Straight at each wall from the middle of the field.
    CHECK_NEAR(core->castRay(0.0f, 0.0f, 1.0f, 0.0f), 70.2f, 1e-3f);
    CHECK_NEAR(core->castRay(0.0f, 0.0f, 0.0f, 1.0f), 70.2f, 1e-3f);
    CHECK_NEAR(core->castRay(0.0f, 0.0f, -1.0f, 0.0f), 70.2f, 1e-3f);
    CHECK_NEAR(core->castRay(0.0f, 0.0f, 0.0f, -1.0f), 70.2f, 1e-3f);

    // Exactly at the corner: both adjacent walls are hit at the same distance
    // and neither may be discarded as an endpoint.
    const float d = std::sqrt(0.5f);
    CHECK_NEAR(core->castRay(0.0f, 0.0f, d, d), 99.277792f, 2e-3f);
    CHECK_NEAR(core->castRay(0.0f, 0.0f, -d, d), 99.277792f, 2e-3f);
    CHECK_NEAR(core->castRay(0.0f, 0.0f, -d, -d), 99.277792f, 2e-3f);
    CHECK_NEAR(core->castRay(0.0f, 0.0f, d, -d), 99.277792f, 2e-3f);
}

CHAD_TEST(mclRaycast, handComputedHitsOnASmallSquare) {
    Cfg c = silentConfig();
    c.maxSensorRange = 100.0f;
    auto core = makeCore(c);
    const std::vector<Line> sq = square(10.0f);
    core->setFieldMap({sq.data(), 4, nullptr, 0, 10.0f});

    CHECK_NEAR(core->castRay(5.0f, 0.0f, 1.0f, 0.0f), 5.0f, 1e-4f);
    CHECK_NEAR(core->castRay(5.0f, 0.0f, -1.0f, 0.0f), 15.0f, 1e-4f);
    CHECK_NEAR(core->castRay(0.0f, 0.0f, std::cos(degToRad(30.0f)),
                             std::sin(degToRad(30.0f))),
               11.547005f, 1e-3f);
    // Sliding along the top wall: that wall is parallel and must be a miss, so
    // the answer is the right wall's top endpoint.
    CHECK_NEAR(core->castRay(0.0f, 10.0f, 1.0f, 0.0f), 10.0f, 1e-4f);
    // Origin outside the square, aimed back through it. The cheap bounding
    // reject must not throw this away.
    CHECK_NEAR(core->castRay(20.0f, 0.0f, -1.0f, 0.0f), 10.0f, 1e-4f);
    CHECK_NEAR(core->castRay(-9.0f, -9.0f, d0(), d0()), 26.870058f, 2e-3f);
}

CHAD_TEST(mclRaycast, diagonalSegmentAndParallelMiss) {
    Cfg c = silentConfig();
    c.maxSensorRange = 100.0f;
    auto core = makeCore(c);
    const std::vector<Line> diag = {{{0.0f, 0.0f}, {10.0f, 10.0f}}};
    core->setFieldMap({diag.data(), 1, nullptr, 0, 100.0f});

    CHECK_NEAR(core->castRay(5.0f, 0.0f, 0.0f, 1.0f), 5.0f, 1e-4f);
    // A ray parallel to the segment never intersects it; the division by a zero
    // denominator must not leak an inf or a NaN out as a hit.
    const float miss = core->castRay(-1.0f, 1.0f, d0(), d0());
    CHECK_NEAR(miss, 100.0f, 1e-4f);
}

CHAD_TEST(mclRaycast, takesNearestHitNotFirstClassOfGeometry) {
    Cfg c = silentConfig();
    c.maxSensorRange = 100.0f;
    auto core = makeCore(c);

    // Far segment listed first. A cast that stops as soon as any hit is found
    // would report 30 and quietly ignore a wall 20 inches closer.
    const std::vector<Line> lines = {{{30.0f, -50.0f}, {30.0f, 50.0f}},
                                     {{10.0f, -50.0f}, {10.0f, 50.0f}}};
    core->setFieldMap({lines.data(), 2, nullptr, 0, 100.0f});
    CHECK_NEAR(core->castRay(0.0f, 0.0f, 1.0f, 0.0f), 10.0f, 1e-4f);

    // Same, with a circle nearer than the first-listed line.
    const std::vector<Line> beyond = {{{30.0f, -50.0f}, {30.0f, 50.0f}}};
    const std::vector<Circle> nearer = {{15.0f, 0.0f, 3.0f}};
    core->setFieldMap({beyond.data(), 1, nearer.data(), 1, 100.0f});
    CHECK_NEAR(core->castRay(0.0f, 0.0f, 1.0f, 0.0f), 12.0f, 1e-4f);
}

CHAD_TEST(mclRaycast, circles) {
    Cfg c = silentConfig();
    c.maxSensorRange = 100.0f;
    auto core = makeCore(c);

    const std::vector<Circle> one = {{10.0f, 0.0f, 2.0f}};
    core->setFieldMap({nullptr, 0, one.data(), 1, 100.0f});
    CHECK_NEAR(core->castRay(0.0f, 0.0f, 1.0f, 0.0f), 8.0f, 1e-4f);
    // Offset enough to pass by: 5 inches off the axis of a 2 inch circle.
    CHECK_NEAR(core->castRay(0.0f, 5.0f, 1.0f, 0.0f), 100.0f, 1e-4f);
    // Behind the sensor.
    CHECK_NEAR(core->castRay(0.0f, 0.0f, -1.0f, 0.0f), 100.0f, 1e-4f);

    // Starting inside: the exit point is the only non-negative root.
    const std::vector<Circle> around = {{0.0f, 0.0f, 3.0f}};
    core->setFieldMap({nullptr, 0, around.data(), 1, 100.0f});
    CHECK_NEAR(core->castRay(0.0f, 0.0f, 1.0f, 0.0f), 3.0f, 1e-4f);
}

CHAD_TEST(mclRaycast, emptyMapReturnsMaxRange) {
    Cfg c = silentConfig();
    c.maxSensorRange = 42.0f;
    auto core = makeCore(c);
    core->setFieldMap({nullptr, 0, nullptr, 0, 70.2f});
    CHECK_NEAR(core->castRay(0.0f, 0.0f, 1.0f, 0.0f), 42.0f, 1e-5f);
}

CHAD_TEST(mclRaycast, mountOffsetAndYaw) {
    Cfg c = silentConfig();
    c.maxSensorRange = 140.0f;
    auto core = makeCore(c);

    std::array<Pose, MclCore::SENSOR_COUNT> mounts{};
    mounts[3] = {2.0f, 3.0f, HALF_PI};   // 2 in forward, 3 in left, facing left
    core->setSensorMounts(mounts);

    // Hand-computed: from (10, 20, 30 deg) the sensor sits at
    // (10.2320508, 23.5980762) pointing 120 deg, which reaches the y = +70.2
    // wall at 53.8112665 in -- well short of the x = -70.2 wall at 160.86.
    const Pose robot{10.0f, 20.0f, degToRad(30.0f)};
    CHECK_NEAR(core->expectedRange(robot, 3), 53.811266f, 3e-3f);

    // Ignoring the mount entirely, or the mount's yaw, gives a different
    // answer, so the test would catch either omission.
    CHECK(std::fabs(core->expectedRange(robot, 0) - 53.811266f) > 1.0f);
}

// ===========================================================================
// Motion model
// ===========================================================================

CHAD_TEST(mclMotion, agreesWithPoseExp) {
    const Pose starts[] = {{0.0f, 0.0f, 0.0f},
                           {12.0f, -30.0f, 1.2f},
                           {-40.0f, 55.0f, -2.9f},
                           {5.0f, 5.0f, 3.1f}};
    const Twist2d twists[] = {{0.0f, 0.0f, 0.0f},
                              {4.0f, 0.0f, 0.0f},
                              {0.0f, 0.0f, 0.35f},
                              {4.0f, 0.0f, 0.35f},
                              {4.0f, -1.5f, -0.6f},
                              {10.0f, 3.0f, HALF_PI},
                              {-6.0f, 0.0f, 0.9f}};

    Cfg c = silentConfig(1);
    auto core = makeCore(c);
    for (const Pose& start : starts) {
        for (const Twist2d& t : twists) {
            MclCore::Particle p{};
            p.pose = start;
            core->setParticles(&p, 1);
            // imuHeadingRad is the expected final heading with the clamp opened
            // wide, so the IMU term cannot mask a disagreement.
            core->predict(motion(t.dx, t.dy, t.dtheta, wrapRad(start.theta + t.dtheta)));

            const Pose want = start.exp(t);
            const Pose got = core->particles()[0].pose;
            CHECK_NEAR(got.x, want.x, 2e-4f);
            CHECK_NEAR(got.y, want.y, 2e-4f);
            CHECK_NEAR(std::fabs(angleDiffRad(got.theta, want.theta)), 0.0f, 2e-5f);
        }
    }
}

CHAD_TEST(mclMotion, integratesTheArcNotTheChord) {
    Cfg c = silentConfig(1);
    auto core = makeCore(c);
    MclCore::Particle p{};
    core->setParticles(&p, 1);

    // 10 inches of forward travel while turning a quarter turn. Along the arc
    // that lands at (20/pi, 20/pi); integrating the twist as a straight line
    // then rotating lands at (10, 0), 5.2 inches away.
    core->predict(motion(10.0f, 0.0f, HALF_PI, HALF_PI));
    const Pose got = core->particles()[0].pose;
    CHECK_NEAR(got.x, 20.0f / PI, 1e-3f);
    CHECK_NEAR(got.y, 20.0f / PI, 1e-3f);
    CHECK_NEAR(got.theta, HALF_PI, 1e-4f);
    CHECK(std::fabs(got.x - 10.0f) > 3.0f);
}

CHAD_TEST(mclMotion, noiseIsSampledInTheBodyFrame) {
    // Lateral slip noise only. At heading 0 the cloud must spread along world
    // y; at heading 90 degrees the same noise must spread it along world x.
    // World-frame sampling would give the same axis both times.
    Cfg c = silentConfig(512);
    c.horizConstantNoise = 1.0f;

    for (int trial = 0; trial < 2; trial++) {
        const float heading = (trial == 0) ? 0.0f : HALF_PI;
        auto core = makeCore(c, 99u);
        std::vector<MclCore::Particle> seed(512);
        for (auto& p : seed) p.pose = {0.0f, 0.0f, heading};
        core->setParticles(seed.data(), 512);

        core->predict(motion(0.0f, 0.0f, 0.0f, heading, false));
        core->updateWeights(nullptr, 0);
        const MclCore::Estimate est = core->estimate();

        if (trial == 0) {
            CHECK(est.covYY > 20.0f * est.covXX);
            CHECK_NEAR(est.covYY, 1.0f, 0.2f);
        } else {
            CHECK(est.covXX > 20.0f * est.covYY);
            CHECK_NEAR(est.covXX, 1.0f, 0.2f);
        }
    }
}

CHAD_TEST(mclMotion, noiseIsNotReplayedEveryTick) {
    // Regression on a fixed noise pool: 1024 pre-drawn normals walked with a
    // stride coprime to the pool size returns to its starting index after
    // exactly one tick's worth of draws, so every particle receives the same
    // jitter on every tick. That is a coherent bias, not diffusion.
    Cfg c = silentConfig(256);
    c.horizConstantNoise = 1.0f;
    auto core = makeCore(c, 5u);

    std::vector<MclCore::Particle> seed(256);
    core->setParticles(seed.data(), 256);

    core->predict(motion(0.0f, 0.0f, 0.0f, 0.0f, false));
    std::vector<float> first(256);
    for (int i = 0; i < 256; i++) first[i] = core->particles()[i].pose.y;

    core->predict(motion(0.0f, 0.0f, 0.0f, 0.0f, false));
    int repeated = 0;
    for (int i = 0; i < 256; i++) {
        const float step = core->particles()[i].pose.y - first[i];
        if (std::fabs(step - first[i]) < 1e-6f) repeated++;
    }
    CHECK(repeated < 4);

    // Two ticks of independent unit noise give variance 2, not 4 as a repeated
    // draw would.
    double sumSq = 0.0;
    for (int i = 0; i < 256; i++) {
        sumSq += static_cast<double>(core->particles()[i].pose.y) *
                 core->particles()[i].pose.y;
    }
    CHECK_NEAR(sumSq / 256.0, 2.0, 0.6);
}

CHAD_TEST(mclMotion, nonFiniteDeltaIsRefused) {
    Cfg c = silentConfig(4);
    c.horizConstantNoise = 0.5f;
    auto core = makeCore(c);
    std::vector<MclCore::Particle> seed(4);
    for (auto& p : seed) p.pose = {3.0f, -4.0f, 0.5f};
    core->setParticles(seed.data(), 4);

    core->predict(motion(kNan, 0.0f, 0.0f, 0.0f));
    core->predict(motion(0.0f, kInf, 0.0f, 0.0f));
    core->predict(motion(0.0f, 0.0f, kNan, 0.0f));
    for (int i = 0; i < 4; i++) {
        CHECK_EQ(core->particles()[i].pose.x, 3.0f);
        CHECK_EQ(core->particles()[i].pose.y, -4.0f);
        CHECK_EQ(core->particles()[i].pose.theta, 0.5f);
    }

    // A non-finite IMU heading must not drag the whole cloud to zero either.
    core->predict(motion(1.0f, 0.0f, 0.0f, kNan));
    for (int i = 0; i < 4; i++) CHECK_NEAR(core->particles()[i].pose.theta, 0.5f, 1e-5f);
}

CHAD_TEST(mclMotion, headingStaysWithinTheImuBand) {
    Cfg c = silentConfig(256);
    c.imuVariance = 0.5f;         // far wider than the band
    c.maxThetaDeviation = 0.08f;
    auto core = makeCore(c, 3u);

    std::vector<MclCore::Particle> seed(256);
    for (auto& p : seed) p.pose = {0.0f, 0.0f, 3.10f};
    core->setParticles(seed.data(), 256);

    // IMU just the other side of the +/-pi cut, so a linear clamp would slam
    // the whole cloud across the field's heading range.
    const float imu = -3.10f;
    core->predict(motion(0.0f, 0.0f, 0.06f, imu));
    for (int i = 0; i < 256; i++) {
        const float dev = std::fabs(angleDiffRad(imu, core->particles()[i].pose.theta));
        CHECK(dev <= 0.08f + 1e-5f);
    }
}

// ===========================================================================
// Resampling
// ===========================================================================

CHAD_TEST(mclResample, systematicDrawIsExact) {
    int out[16] = {};

    const float flat[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    MclCore::systematicResample(flat, 4, 0.5f, out, 4);
    const int wantFlat[4] = {0, 1, 2, 3};
    for (int i = 0; i < 4; i++) CHECK_EQ(out[i], wantFlat[i]);

    // Two zero weights must never be drawn, even with the uniform at exactly 0.
    const float gappy[4] = {0.0f, 3.0f, 0.0f, 1.0f};
    MclCore::systematicResample(gappy, 4, 0.0f, out, 4);
    const int wantGappy[4] = {1, 1, 1, 3};
    for (int i = 0; i < 4; i++) CHECK_EQ(out[i], wantGappy[i]);

    const float single[4] = {4.0f, 0.0f, 0.0f, 0.0f};
    MclCore::systematicResample(single, 4, 0.5f, out, 4);
    for (int i = 0; i < 4; i++) CHECK_EQ(out[i], 0);

    // 1:2:3:4 over ten draws is exactly one, two, three and four copies.
    const float ramp[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    MclCore::systematicResample(ramp, 4, 0.5f, out, 10);
    const int wantRamp[10] = {0, 1, 1, 2, 2, 2, 3, 3, 3, 3};
    for (int i = 0; i < 10; i++) CHECK_EQ(out[i], wantRamp[i]);

    // Drawing fewer than the cloud holds.
    MclCore::systematicResample(flat, 4, 0.5f, out, 2);
    CHECK_EQ(out[0], 1);
    CHECK_EQ(out[1], 3);

    // No weight anywhere: spread rather than 4 copies of particle 0.
    const float dead[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    MclCore::systematicResample(dead, 4, 0.5f, out, 4);
    for (int i = 0; i < 4; i++) CHECK_EQ(out[i], i);
}

CHAD_TEST(mclResample, countsAreFloorOrCeilOfTheExpectation) {
    // The defining property of low-variance resampling, and what puts its
    // variance O(N) below multinomial: particle i is drawn either floor or ceil
    // of N * w_i / W times, never more and never fewer.
    Pcg32 g(77u);
    int out[64] = {};
    int counts[16] = {};
    for (int trial = 0; trial < 400; trial++) {
        float w[16];
        float total = 0.0f;
        for (int i = 0; i < 16; i++) {
            w[i] = (g.uniform() < 0.25f) ? 0.0f : g.uniform();
            total += w[i];
        }
        if (total <= 0.0f) continue;

        const int n = 64;
        MclCore::systematicResample(w, 16, g.uniform(), out, n);
        for (int i = 0; i < 16; i++) counts[i] = 0;
        for (int m = 0; m < n; m++) {
            CHECK(out[m] >= 0 && out[m] < 16);
            counts[out[m]]++;
        }
        int drawn = 0;
        for (int i = 0; i < 16; i++) {
            const float expect = static_cast<float>(n) * w[i] / total;
            CHECK(counts[i] >= static_cast<int>(std::floor(expect - 1e-3f)));
            CHECK(counts[i] <= static_cast<int>(std::ceil(expect + 1e-3f)));
            drawn += counts[i];
        }
        CHECK_EQ(drawn, n);
    }
}

CHAD_TEST(mclResample, effectiveSampleSizeArithmetic) {
    const float flat[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    CHECK_NEAR(MclCore::effectiveSampleSize(flat, 4), 4.0f, 1e-5f);

    const float scaled[4] = {0.25f, 0.25f, 0.25f, 0.25f};
    CHECK_NEAR(MclCore::effectiveSampleSize(scaled, 4), 4.0f, 1e-5f);

    const float half[4] = {1.0f, 1.0f, 0.0f, 0.0f};
    CHECK_NEAR(MclCore::effectiveSampleSize(half, 4), 2.0f, 1e-5f);

    const float skew[2] = {3.0f, 1.0f};
    CHECK_NEAR(MclCore::effectiveSampleSize(skew, 2), 1.6f, 1e-5f);

    const float spike[4] = {1.0f, 1e-9f, 1e-9f, 1e-9f};
    CHECK_NEAR(MclCore::effectiveSampleSize(spike, 4), 1.0f, 1e-4f);

    const float dead[3] = {0.0f, 0.0f, 0.0f};
    CHECK_EQ(MclCore::effectiveSampleSize(dead, 3), 0.0f);

    const float bad[3] = {kNan, kInf, 1.0f};
    CHECK_NEAR(MclCore::effectiveSampleSize(bad, 3), 1.0f, 1e-5f);
    CHECK_EQ(MclCore::effectiveSampleSize(nullptr, 4), 0.0f);
}

CHAD_TEST(mclResample, selectsTheExpectedParticleMultiset) {
    Cfg c = silentConfig(4);   // resample jitter is zero here
    auto core = makeCore(c);

    std::vector<MclCore::Particle> seed(4);
    for (int i = 0; i < 4; i++) {
        seed[i].pose = {static_cast<float>(i), 0.0f, 0.0f};
        seed[i].weight = static_cast<float>(i);   // 0, 1, 2, 3
    }
    core->setParticles(seed.data(), 4);
    core->resample(0.0f);

    // Weights 0:1:2:3 over four draws: no copy of particle 0 and, since the
    // draw is systematic, exactly the sweep of the cumulative array.
    int seen[4] = {};
    for (int i = 0; i < 4; i++) {
        const int which = static_cast<int>(core->particles()[i].pose.x + 0.5f);
        CHECK(which >= 1 && which <= 3);
        seen[which]++;
        CHECK_EQ(core->particles()[i].weight, 1.0f);
        CHECK_EQ(core->particles()[i].logWeight, 0.0f);
    }
    CHECK_EQ(seen[0], 0);
    CHECK(seen[3] >= 1);
    CHECK(core->resampledLast());
}

CHAD_TEST(mclResample, allWeightsZeroKeepsTheCloudAndStaysFinite) {
    Cfg c = silentConfig(8);
    auto core = makeCore(c);

    std::vector<MclCore::Particle> seed(8);
    for (int i = 0; i < 8; i++) {
        seed[i].pose = {static_cast<float>(i) - 4.0f, 2.0f, 0.1f};
        seed[i].weight = 0.0f;
    }
    core->setParticles(seed.data(), 8);
    core->resample(0.0f);

    for (int i = 0; i < 8; i++) {
        CHECK_EQ(core->particles()[i].pose.x, static_cast<float>(i) - 4.0f);
        CHECK_EQ(core->particles()[i].weight, 1.0f);
    }
    const MclCore::Estimate est = core->estimate();
    CHECK_FINITE(est.pose.x);
    CHECK_FINITE(est.pose.y);
    CHECK_FINITE(est.pose.theta);
    CHECK_FINITE(est.nEff);
    CHECK_NEAR(est.nEff, 8.0f, 1e-4f);
    CHECK_NEAR(est.pose.x, -0.5f, 1e-4f);
}

CHAD_TEST(mclResample, adaptiveGateHoldsWhileEssIsHigh) {
    Cfg c = silentConfig(64);
    c.particleCount = 64;
    c.resampleThreshold = 32;
    auto core = makeCore(c);
    core->setSensorMounts(fourFaceMounts());

    // No measurements at all: every weight is equal, ESS == N, nothing to gain
    // from resampling. Doing it anyway is what depletes the cloud.
    for (int i = 0; i < 5; i++) {
        core->step(motion(0.0f, 0.0f, 0.0f, 0.0f), nullptr, 0);
        CHECK(!core->resampledLast());
    }

    // One particle carrying all the weight drives ESS to 1.
    std::vector<MclCore::Particle> seed(64);
    for (int i = 0; i < 64; i++) seed[i].pose = {static_cast<float>(i) - 32.0f, 0.0f, 0.0f};
    core->setParticles(seed.data(), 64);

    MclCore::Beam beam{};
    beam.sensor = 0;
    beam.rangeIn = core->expectedRange(seed[10].pose, 0);
    beam.invSigma = 1.0f / 0.2f;
    core->updateWeights(&beam, 1);
    CHECK(core->estimate().nEff < 16.0f);

    core->setParticles(seed.data(), 64);
    core->step(motion(0.0f, 0.0f, 0.0f, 0.0f), &beam, 1);
    CHECK(core->resampledLast());
}

// ===========================================================================
// Beam model
// ===========================================================================

CHAD_TEST(mclSensor, mixtureShape) {
    Cfg c = silentConfig();
    const float floorLog = std::log(c.randWeight);

    // The unexpected-obstacle term switches on abruptly at a zero residual, so
    // the mixture has a step there: a reading a hair short scores 0.405 nats
    // above one that matches exactly. The peak is still at zero and the step is
    // a jump rather than a slope, so the mode is unbiased -- but the short side
    // does carry 1.5x the mass of the long side. Pinned here so that a change
    // to the mixture is a deliberate one.
    const float justShort = MclCore::logBeamLikelihoodExact(c, -1e-6f);
    const float exactHit = MclCore::logBeamLikelihoodExact(c, 0.0f);
    CHECK_NEAR(justShort - exactHit,
               std::log((c.hitWeight + c.shortWeight + c.randWeight) /
                        (c.hitWeight + c.randWeight)),
               1e-4f);

    // Long side: strictly decreasing while the Gaussian is above the floor,
    // then flat on the floor, and never below it.
    float prev = exactHit;
    for (float u = 0.25f; u <= 4.0f; u += 0.25f) {
        const float v = MclCore::logBeamLikelihoodExact(c, u);
        CHECK(v < prev);
        prev = v;
    }
    for (float u = 4.0f; u <= 8.0f; u += 0.25f) {
        const float v = MclCore::logBeamLikelihoodExact(c, u);
        CHECK(v <= prev + 1e-6f);
        CHECK(v >= floorLog - 1e-4f);
        prev = v;
    }

    // Short side: monotone away from the peak, and never harsher than the same
    // error long, because a short reading has an excuse and a long one has not.
    prev = justShort;
    for (float u = -0.25f; u >= -8.0f; u -= 0.25f) {
        const float v = MclCore::logBeamLikelihoodExact(c, u);
        CHECK(v < prev);
        CHECK(v > MclCore::logBeamLikelihoodExact(c, -u));
        prev = v;
    }

    // The floor is what stops one beam from zeroing a particle outright.
    CHECK_NEAR(MclCore::logBeamLikelihoodExact(c, 8.0f), floorLog, 1e-3f);
    CHECK(MclCore::logBeamLikelihoodExact(c, 40.0f) > -8.0f);
}

CHAD_TEST(mclSensor, lutTracksTheExactMixture) {
    Cfg c = silentConfig();
    auto core = makeCore(c);

    // One bound over the whole span, with nothing skipped. This test used to
    // skip |u| <= nodeStep -- the only place the table was inaccurate, by 0.203
    // nats against 1e-4 everywhere else -- and replace the check there with
    // "does the interpolant lie between the exact values either side?". That
    // cannot fail: linear interpolation between two samples of f is always
    // within [min f, max f] over the enclosing window, and the measured slack
    // was 3.06e-5 against a 5e-4 tolerance.
    float worst = 0.0f, worstAt = 0.0f;
    for (int i = -800000; i <= 800000; i++) {
        const float u = static_cast<float>(i) * 1e-5f;
        const float err = std::fabs(core->logBeamLikelihood(u) -
                                    MclCore::logBeamLikelihoodExact(c, u));
        if (err > worst) {
            worst = err;
            worstAt = u;
        }
    }

    // The bar is interpolation theory, not a number fitted to what the table
    // happens to do today: a linear interpolant on a cell of width step is off
    // by at most max|f''| * step^2 / 8. f = log g, so f'' = g''/g - (g'/g)^2,
    // and each half table is sampled on one side of the step at a zero residual
    // so the curvature that bounds it is one-sided. A tolerance loose enough to
    // pass a coarser table -- 5e-4 admits half as many nodes, which quadruples
    // the error -- is not a bound on anything.
    const double step = static_cast<double>(MclCore::LUT_SIGMA_SPAN) / 511.0;
    const double hw = static_cast<double>(c.hitWeight);
    const double rw = static_cast<double>(c.randWeight);
    const double sw = static_cast<double>(c.shortWeight);
    const double lam = static_cast<double>(c.shortDecaySigmas);
    double maxCurv = 0.0;
    for (int i = -400000; i <= 400000; i++) {
        const double s = static_cast<double>(i) * 2e-5;
        const double e = hw * std::exp(-0.5 * s * s);
        const double t = (s < 0.0) ? sw * std::exp(s / lam) : 0.0;
        const double g = e + rw + t;
        const double g1 = -s * e + t / lam;
        const double g2 = (s * s - 1.0) * e + t / (lam * lam);
        maxCurv = std::fmax(maxCurv, std::fabs(g2 / g - (g1 / g) * (g1 / g)));
    }
    const double bound = maxCurv * step * step / 8.0;
    std::printf("      lut worst |table - exact| %.3e nats at s = %+.5f "
                "(max|f''| %.3f, theory bound %.3e)\n",
                static_cast<double>(worst), static_cast<double>(worstAt), maxCurv,
                bound);
    CHECK(static_cast<double>(worst) < 1.2 * bound);

    // The step at a zero residual is 0.405 nats, and both one-sided limits have
    // to be reachable: a single table straddling zero can only return something
    // in between, which is what the 0.203 nat error was.
    CHECK_NEAR(core->logBeamLikelihood(0.0f), MclCore::logBeamLikelihoodExact(c, 0.0f),
               1e-6f);
    CHECK_NEAR(core->logBeamLikelihood(-1e-7f),
               MclCore::logBeamLikelihoodExact(c, -1e-6f), 1e-5f);
    CHECK(core->logBeamLikelihood(-1e-7f) - core->logBeamLikelihood(0.0f) > 0.4f);

    // Clamped outside the tabulated span, and never UB on a bad residual.
    CHECK_NEAR(core->logBeamLikelihood(50.0f), core->logBeamLikelihood(8.0f), 1e-6f);
    CHECK_NEAR(core->logBeamLikelihood(-50.0f), core->logBeamLikelihood(-8.0f), 1e-6f);
    CHECK_FINITE(core->logBeamLikelihood(kNan));
    CHECK_FINITE(core->logBeamLikelihood(kInf));

    // setConfig has to rebuild the table, or a retuned mixture is ignored.
    Cfg wider = c;
    wider.randWeight = 0.1f;
    core->setConfig(wider);
    CHECK_NEAR(core->logBeamLikelihood(8.0f), std::log(0.1f), 1e-3f);
}

CHAD_TEST(mclSensor, normalisedMixtureIsAProbabilityDensity) {
    Cfg c = silentConfig();
    // exp(logBeamLikelihoodExact - logMixtureNorm) is a density in sigmas, so it
    // integrates to 1 over the tabulated span. That is the property that makes a
    // beam score comparable with deadzoneLikelihood, which is a real density per
    // inch: without it log(0.25) was being weighed against an unnormalised
    // mixture whose peak is 1.5, and the two were incommensurable.
    const double norm = static_cast<double>(MclCore::logMixtureNorm(c));
    const int m = 400000;
    const double h = 2.0 * static_cast<double>(MclCore::LUT_SIGMA_SPAN) / m;
    double sum = 0.0;
    for (int i = 0; i <= m; i++) {
        const double sig = -static_cast<double>(MclCore::LUT_SIGMA_SPAN) + h * i;
        const double w = (i == 0 || i == m) ? 0.5 : 1.0;
        sum += w * h *
               std::exp(static_cast<double>(MclCore::logBeamLikelihoodExact(
                            c, static_cast<float>(sig))) -
                        norm);
    }
    CHECK_NEAR(sum, 1.0, 2e-3);

    // And it scales with the mixture it was built from, rather than being a
    // constant someone typed.
    Cfg wide = c;
    wide.shortWeight = 2.0f * c.shortWeight;
    CHECK(MclCore::logMixtureNorm(wide) > MclCore::logMixtureNorm(c) + 0.1f);
}

CHAD_TEST(mclSensor, weightMatchesTheHandComputedLikelihood) {
    Cfg c = silentConfig(1);
    auto core = makeCore(c);
    core->setSensorMounts(fourFaceMounts());

    MclCore::Particle p{};
    p.pose = {12.0f, -8.0f, 0.4f};
    core->setParticles(&p, 1);

    const float expected = refRange(p.pose, 1);
    MclCore::Beam beam{};
    beam.sensor = 1;
    beam.rangeIn = expected + 1.3f;   // 1.3 in long
    beam.invSigma = 1.0f / 0.65f;     // so exactly 2 sigma
    core->updateWeights(&beam, 1);

    // A density per inch, not a bare mixture value: log(1/sigma) less the
    // mixture's own integral. Drop either term and the beam score is no longer
    // comparable with deadzoneLikelihood, which is a real density.
    const float want = MclCore::logBeamLikelihoodExact(c, 2.0f) +
                       std::log(beam.invSigma) - MclCore::logMixtureNorm(c);
    CHECK_NEAR(core->particles()[0].logWeight, want, 1e-3f);

    // The normalisation is the whole reason a blocked beam and a measured one
    // can be compared: halving sigma must move the beam score by log 2 while
    // leaving the deadzone score alone.
    MclCore::Beam tighter = beam;
    tighter.invSigma = 2.0f / 0.65f;
    tighter.rangeIn = expected + 0.65f;   // still exactly 2 sigma
    core->updateWeights(&tighter, 1);
    CHECK_NEAR(core->particles()[0].logWeight, want + std::log(2.0f), 1e-3f);
}

CHAD_TEST(mclSensor, oneWildBeamCannotAnnihilateTheCloud) {
    Cfg c = silentConfig(2);
    auto core = makeCore(c);
    core->setSensorMounts(fourFaceMounts());

    const Pose truth{20.0f, -15.0f, 0.3f};
    std::vector<MclCore::Particle> seed(2);
    seed[0].pose = truth;
    seed[1].pose = {truth.x + 20.0f, truth.y, truth.theta};
    core->setParticles(seed.data(), 2);

    MclCore::Beam beams[4];
    for (int i = 0; i < 4; i++) {
        beams[i].sensor = i;
        beams[i].rangeIn = core->expectedRange(truth, i);
        beams[i].invSigma = 1.0f / 0.5f;
    }
    // Sensor 2 reads a robot parked in front of it instead of the wall: 40 in
    // short, which a pure Gaussian scores at exp(-3200).
    beams[2].rangeIn -= 40.0f;

    core->updateWeights(beams, 4);
    const float wTruth = core->particles()[0].weight;
    const float wWrong = core->particles()[1].weight;
    CHECK(wTruth > 0.0f);
    CHECK(wTruth / (wTruth + wWrong) > 0.9f);
    // And the outlier's contribution stays bounded rather than -inf.
    CHECK(core->particles()[0].logWeight > -32.0f);
}

CHAD_TEST(mclSensor, outOfFieldParticlesAreRankedNotFlattened) {
    Cfg c = silentConfig(3);
    auto core = makeCore(c);
    core->setSensorMounts(fourFaceMounts());

    std::vector<MclCore::Particle> seed(3);
    seed[0].pose = {0.0f, 0.0f, 0.0f};             // inside
    seed[1].pose = {75.0f, 0.0f, 0.0f};            // 4.8 in out
    seed[2].pose = {110.0f, 0.0f, 0.0f};           // 39.8 in out
    core->setParticles(seed.data(), 3);

    MclCore::Beam beam{};
    beam.sensor = 0;
    beam.rangeIn = core->expectedRange(seed[0].pose, 0);
    beam.invSigma = 1.0f / 0.5f;
    core->updateWeights(&beam, 1);

    const MclCore::Particle* p = core->particles();
    CHECK(p[0].logWeight > p[1].logWeight);
    // A flat out-of-field floor leaves these two identical, N_eff pinned at N,
    // and an escaped cloud with no route back inside.
    CHECK(p[1].logWeight > p[2].logWeight);
    CHECK(core->estimate().nEff < 3.0f);
}

CHAD_TEST(mclSensor, buildBeamsValidity) {
    Cfg c = silentConfig(1);
    c.confidenceThreshold = 10;
    c.minSensorRange = 20.0f;   // mm
    auto core = makeCore(c);
    core->setSensorMounts(fourFaceMounts());

    MclCore::RawSensor raw[MclCore::SENSOR_COUNT] = {};
    raw[0] = {1000, 63, true, false};     // good
    raw[1] = {1000, 63, false, false};    // no sensor in the slot
    raw[2] = {1000, 63, true, true};      // suppressed
    raw[3] = {10, 63, true, false};       // below minSensorRange
    raw[4] = {1000, 2, true, false};      // far and unconfident
    raw[5] = {150, 2, true, false};       // close, so confidence is not used
    raw[6] = {2147483647, 0, true, false};   // PROS_ERR
    raw[7] = {9500, 0, true, false};      // nothing in range

    MclCore::Beam beams[MclCore::SENSOR_COUNT] = {};
    bool valid[MclCore::SENSOR_COUNT] = {};
    const int n = core->buildBeams(raw, MclCore::SENSOR_COUNT, Pose{}, beams, valid);

    CHECK_EQ(n, 2);
    CHECK(valid[0]);
    CHECK(valid[5]);
    for (int i : {1, 2, 3, 4, 6, 7}) CHECK(!valid[i]);

    for (int b = 0; b < n; b++) {
        CHECK_FINITE(beams[b].rangeIn);
        CHECK(beams[b].rangeIn > 0.0f);
        CHECK_FINITE(beams[b].invSigma);
        CHECK(beams[b].invSigma > 0.0f);
    }
    CHECK_NEAR(beams[0].rangeIn, 1000.0f * MM_TO_INCH, 1e-4f);

    // With the max-range term on, the no-return becomes a beam at maxSensorRange.
    Cfg withMax = c;
    withMax.useNoReturnBeams = true;
    core->setConfig(withMax);
    const int n2 = core->buildBeams(raw, MclCore::SENSOR_COUNT, Pose{}, beams, valid);
    CHECK_EQ(n2, 3);
    CHECK(valid[7]);
    CHECK_NEAR(beams[2].rangeIn, withMax.maxSensorRange, 1e-4f);
}

CHAD_TEST(mclSensor, noReturnBeamPenalisesANearPrediction) {
    Cfg c = silentConfig(2);
    c.maxSensorRange = 40.0f;   // deliberately shorter than the field
    c.useNoReturnBeams = true;
    auto core = makeCore(c);
    core->setSensorMounts(fourFaceMounts());

    std::vector<MclCore::Particle> seed(2);
    seed[0].pose = {0.0f, 0.0f, 0.0f};      // wall 63 in away: beyond max range
    seed[1].pose = {60.0f, 0.0f, 0.0f};     // wall 3 in away
    core->setParticles(seed.data(), 2);

    MclCore::Beam beam{};
    beam.sensor = 0;
    beam.rangeIn = c.maxSensorRange;
    beam.invSigma = 1.0f / 2.0f;
    core->updateWeights(&beam, 1);

    // "I see nothing out to 40 inches" is evidence against the particle that
    // predicted a wall at 3 inches. Discarding the reading throws that away.
    CHECK(core->particles()[0].logWeight > core->particles()[1].logWeight + 3.0f);
}

CHAD_TEST(mclSensor, deadzoneCircleSuppressesTheBeam) {
    Cfg c = silentConfig(1);
    auto core = makeCore(c);
    core->setSensorMounts(fourFaceMounts());

    const std::vector<Circle> post = {{30.0f, 0.0f, 2.0f}};
    core->setDeadzoneCircles(post.data(), 1);

    MclCore::Particle p{};
    core->setParticles(&p, 1);

    // The sensor sits 7 in forward, so the post surface is 21 in out. A reading
    // at that distance is of the post, not the wall.
    MclCore::Beam beam{};
    beam.sensor = 0;
    beam.rangeIn = 21.0f;
    beam.invSigma = 1.0f / 0.5f;
    core->updateWeights(&beam, 1);
    CHECK_NEAR(core->particles()[0].logWeight, std::log(c.deadzoneLikelihood), 1e-4f);
    CHECK_NEAR(core->deadzoneFraction()[0], 1.0f, 1e-6f);

    // A reading well away from the post surface is a wall measurement again.
    beam.rangeIn = core->expectedRange(p.pose, 0);
    core->updateWeights(&beam, 1);
    CHECK_NEAR(core->deadzoneFraction()[0], 0.0f, 1e-6f);
}

// ===========================================================================
// Estimate
// ===========================================================================

CHAD_TEST(mclEstimate, circularMeanAcrossTheBranchCut) {
    const float angles[4] = {3.10f, -3.10f, 3.12f, -3.12f};
    const float flat[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    const float mean = MclCore::circularMean(angles, flat, 4);
    // The arithmetic mean of these is 0, which points the robot the other way.
    CHECK(std::fabs(mean) > 3.13f);
    CHECK_NEAR(std::fabs(angleDiffRad(mean, PI)), 0.0f, 1e-4f);

    const float quarter[2] = {0.0f, HALF_PI};
    CHECK_NEAR(MclCore::circularMean(quarter, nullptr, 2), QUARTER_PI, 1e-5f);

    const float weighted[2] = {0.0f, HALF_PI};
    const float lopsided[2] = {3.0f, 1.0f};
    CHECK_NEAR(MclCore::circularMean(weighted, lopsided, 2), std::atan2(1.0f, 3.0f),
               1e-5f);

    // Perfectly antipodal: no mean direction exists, so the documented answer
    // is 0. CHECK_FINITE here, which is what this used to be, cannot fail --
    // atan2 is finite for every finite pair -- and it certified a guard that
    // never fired: sinf(PI) is -8.74e-8, not 0, so a |s| < 1e-20 test misses it
    // and the function answered -pi/2, a heading 90 degrees out.
    const float opposed[2] = {0.0f, PI};
    CHECK_NEAR(MclCore::circularMean(opposed, nullptr, 2), 0.0f, 1e-6f);
    const float opposedTriple[4] = {0.6f, 0.6f + PI, -2.2f, -2.2f + PI};
    CHECK_NEAR(MclCore::circularMean(opposedTriple, nullptr, 4), 0.0f, 1e-6f);

    // The guard must not swallow an ordinary heading whose sine or cosine
    // happens to be near zero.
    const float alongY[2] = {HALF_PI, HALF_PI};
    CHECK_NEAR(MclCore::circularMean(alongY, nullptr, 2), HALF_PI, 1e-6f);
    const float alongX[2] = {0.0f, 0.0f};
    CHECK_NEAR(MclCore::circularMean(alongX, nullptr, 2), 0.0f, 1e-6f);
    const float nearlyOpposed[2] = {0.0f, PI - 0.02f};
    CHECK_NEAR(MclCore::circularMean(nearlyOpposed, nullptr, 2), (PI - 0.02f) * 0.5f,
               1e-4f);
}

CHAD_TEST(mclEstimate, poseAndSpreadFromTheCloud) {
    Cfg c = silentConfig(4);
    auto core = makeCore(c);

    std::vector<MclCore::Particle> seed(4);
    seed[0].pose = {1.0f, 0.0f, 3.13f};
    seed[1].pose = {-1.0f, 0.0f, -3.13f};
    seed[2].pose = {1.0f, 0.0f, 3.13f};
    seed[3].pose = {-1.0f, 0.0f, -3.13f};
    for (auto& p : seed) p.weight = 1.0f;
    core->setParticles(seed.data(), 4);

    const MclCore::Estimate est = core->estimate();
    CHECK_NEAR(est.pose.x, 0.0f, 1e-5f);
    CHECK_NEAR(est.pose.y, 0.0f, 1e-5f);
    CHECK(std::fabs(est.pose.theta) > 3.13f);
    CHECK_NEAR(est.nEff, 4.0f, 1e-5f);
    CHECK_NEAR(est.covXX, 1.0f, 1e-5f);
    CHECK_NEAR(est.covYY, 0.0f, 1e-5f);
    CHECK_NEAR(est.covXY, 0.0f, 1e-5f);
    // Heading spread measured as an angular distance, not a subtraction.
    CHECK_NEAR(est.covTheta, std::pow(angleDiffRad(PI, 3.13f), 2.0f), 1e-5f);
}

CHAD_TEST(mclEstimate, nonFiniteParticlesDoNotPoisonThePose) {
    Cfg c = silentConfig(4);
    auto core = makeCore(c);

    std::vector<MclCore::Particle> seed(4);
    seed[0].pose = {2.0f, 2.0f, 0.0f};
    seed[1].pose = {kNan, 0.0f, 0.0f};
    seed[2].pose = {0.0f, kInf, 0.0f};
    seed[3].pose = {4.0f, 6.0f, 0.0f};
    for (auto& p : seed) p.weight = 1.0f;
    core->setParticles(seed.data(), 4);

    const MclCore::Estimate est = core->estimate();
    CHECK_FINITE(est.pose.x);
    CHECK_FINITE(est.pose.y);
    CHECK_NEAR(est.pose.x, 3.0f, 1e-5f);
    CHECK_NEAR(est.pose.y, 4.0f, 1e-5f);
}

// ===========================================================================
// Whole filter
// ===========================================================================

CHAD_TEST(mclFilter, convergesOnNoiseFreeMeasurements) {
    Cfg c;
    c.particleCount = 512;
    c.resampleThreshold = 256;
    c.maxSensorRange = 140.0f;
    auto core = makeCore(c, 20260822u);
    core->setSensorMounts(fourFaceMounts());

    const Pose truth{20.0f, -15.0f, 0.3f};
    core->setPose({truth.x - 4.0f, truth.y + 3.0f, truth.theta});
    CHECK(core->rawEstimate().distanceTo(truth) > 4.9f);

    MclCore::RawSensor raw[MclCore::SENSOR_COUNT];
    senseFrom(truth, 4, raw);

    MclCore::Beam beams[MclCore::SENSOR_COUNT];
    bool valid[MclCore::SENSOR_COUNT];
    for (int tick = 0; tick < 120; tick++) {
        const int n = core->buildBeams(raw, MclCore::SENSOR_COUNT,
                                       core->rawEstimate(), beams, valid);
        CHECK_EQ(n, 4);
        core->step(motion(0.0f, 0.0f, 0.0f, truth.theta), beams, n);
    }

    const MclCore::Estimate est = core->estimate();
    CHECK_FINITE(est.pose.x);
    std::printf("      noise-free: error %.3f in, covXX %.4f, covYY %.4f, nEff %.0f\n",
                static_cast<double>(est.pose.distanceTo(truth)),
                static_cast<double>(est.covXX), static_cast<double>(est.covYY),
                static_cast<double>(est.nEff));
    // Bounds a small multiple of the achieved value. The old 2.5 in and 9.0 in^2
    // were 30x and 120x the settled behaviour, which no regression could break.
    CHECK(est.pose.distanceTo(truth) < 0.10f);
    CHECK_NEAR(std::fabs(angleDiffRad(est.pose.theta, truth.theta)), 0.0f, 0.02f);
    CHECK(est.covXX < 0.30f);
    CHECK(est.covYY < 0.30f);
    // A cloud that has genuinely contracted, not one frozen at the roughening's
    // own width: 1.5 in of jitter every resample leaves covXX above 2.
    CHECK(est.covXX > 1e-5f);
}

CHAD_TEST(mclFilter, convergesOnNoisyMeasurements) {
    // The one thing a four-component mixture exists for. Every other filter test
    // here feeds exact ranges or NaN, so the mixture's behaviour under real range
    // noise was untested end to end.
    Cfg c;
    c.particleCount = 1024;
    c.resampleThreshold = 512;
    c.maxSensorRange = 140.0f;
    auto core = makeCore(c, 20260822u);
    core->setSensorMounts(fourFaceMounts());

    const Pose truth{20.0f, -15.0f, 0.3f};
    core->setPose({truth.x - 4.0f, truth.y + 3.0f, truth.theta});
    Pcg32 noise(999u);

    MclCore::RawSensor raw[MclCore::SENSOR_COUNT];
    MclCore::Beam beams[MclCore::SENSOR_COUNT];
    bool valid[MclCore::SENSOR_COUNT];
    double sumErr = 0.0, sumDx = 0.0, sumDy = 0.0, worst = 0.0;
    int scored = 0;
    for (int tick = 0; tick < 400; tick++) {
        senseFrom(truth, 4, raw, c.maxSensorRange, &noise, 0.5f);
        const int n = core->buildBeams(raw, MclCore::SENSOR_COUNT,
                                       core->rawEstimate(), beams, valid);
        CHECK_EQ(n, 4);
        core->step(motion(0.0f, 0.0f, 0.0f, truth.theta), beams, n);
        if (tick < 100) continue;
        const Pose est = core->estimate().pose;
        sumErr += static_cast<double>(est.distanceTo(truth));
        sumDx += static_cast<double>(est.x - truth.x);
        sumDy += static_cast<double>(est.y - truth.y);
        worst = std::fmax(worst, static_cast<double>(est.distanceTo(truth)));
        scored++;
    }
    const double mean = sumErr / scored;
    std::printf("      0.5 in range noise: mean %.3f in, worst %.3f in, "
                "bias (%+.3f, %+.3f)\n", mean, worst, sumDx / scored, sumDy / scored);
    CHECK(mean < 0.30);
    CHECK(worst < 0.80);
    // The short tail is one-sided, so a mis-weighted mixture stands the estimate
    // off every wall. On symmetric geometry opposing beams cancel most of it;
    // what is left must stay well inside the noise.
    CHECK(std::fabs(sumDx / scored) < 0.10);
    CHECK(std::fabs(sumDy / scored) < 0.10);
}

CHAD_TEST(mclFilter, tracksASmallDisplacementWithoutInjection) {
    Cfg c;
    c.particleCount = 512;
    c.resampleThreshold = 256;
    c.maxSensorRange = 140.0f;
    CHECK_EQ(c.maxRandomFrac, 0.0f);   // recovery here is ordinary tracking only

    auto core = makeCore(c, 4242u);
    core->setSensorMounts(fourFaceMounts());

    Pose truth{0.0f, 0.0f, 0.0f};
    core->setPose(truth);
    Pcg32 noise(777u);

    MclCore::RawSensor raw[MclCore::SENSOR_COUNT];
    MclCore::Beam beams[MclCore::SENSOR_COUNT];
    bool valid[MclCore::SENSOR_COUNT];
    // Ranges carry real noise, and the score is a mean over the tail of the run:
    // a single tick of a resampling filter under noise is a draw, and recovery
    // means settling there rather than passing through once.
    auto run = [&](int ticks, int scoreLast) {
        double sum = 0.0;
        int scored = 0;
        for (int t = 0; t < ticks; t++) {
            senseFrom(truth, 4, raw, c.maxSensorRange, &noise, 0.5f);
            const int n = core->buildBeams(raw, MclCore::SENSOR_COUNT,
                                           core->rawEstimate(), beams, valid);
            core->step(motion(0.0f, 0.0f, 0.0f, truth.theta), beams, n);
            if (t >= ticks - scoreLast) {
                sum += static_cast<double>(core->estimate().pose.distanceTo(truth));
                scored++;
            }
        }
        return static_cast<float>(sum / static_cast<double>(scored));
    };

    const float settled = run(60, 20);
    CHECK(settled < 0.30f);

    // Teleport the robot 6 inches without telling the filter, then 20, which the
    // fixed 1.5 in roughening never recovered from at all. Recovery here is the
    // mismatch-driven search roughening, with maxRandomFrac still 0.
    truth.x += 6.0f;
    const float after6 = run(200, 40);
    truth.y -= 20.0f;
    const float after20 = run(300, 40);
    std::printf("      settled %.3f in, after a 6 in jump %.3f in, after 20 in %.3f in\n",
                static_cast<double>(settled), static_cast<double>(after6),
                static_cast<double>(after20));
    // Against 6 and 20 in of displacement, so these are assertions of recovery
    // to a fraction of an inch. The 20 in case sits higher because alphaSlow
    // gives the mismatch ramp a 200 tick memory, so the cloud is still slightly
    // wide 300 ticks later.
    CHECK(after6 < 0.25f);
    CHECK(after20 < 0.35f);
    // And it has to stay there, not merely pass through.
    CHECK(run(60, 40) < 0.35f);
}

CHAD_TEST(mclFilter, roughenAndSearchWidthsAreSeparateKnobs) {
    // distResampleVariance used to set both the setPose spread and the
    // resampling roughening. 1.5 in of roughening holds nEff above both gates,
    // so the cloud froze at the roughening's own width and the filter could not
    // do better than 0.4 in however long it ran. Re-coupling them is the
    // regression this catches.
    const Pose truth{20.0f, -15.0f, 0.3f};
    float err[2] = {0.0f, 0.0f};
    float cov[2] = {0.0f, 0.0f};

    for (int mode = 0; mode < 2; mode++) {
        Cfg c;
        c.particleCount = 1024;
        c.maxSensorRange = 140.0f;
        if (mode == 1) {
            // The old coupling: one number for both roles.
            c.distResampleVariance = c.distSearchVariance;
            c.thetaResampleVariance = c.thetaSearchVariance;
        }
        auto core = makeCore(c, 4242u);
        core->setSensorMounts(fourFaceMounts());
        core->setPose({truth.x - 4.0f, truth.y + 3.0f, truth.theta});
        Pcg32 noise(999u);

        MclCore::RawSensor raw[MclCore::SENSOR_COUNT];
        MclCore::Beam beams[MclCore::SENSOR_COUNT];
        bool valid[MclCore::SENSOR_COUNT];
        double sum = 0.0, sumCov = 0.0;
        int scored = 0;
        for (int t = 0; t < 400; t++) {
            senseFrom(truth, 4, raw, c.maxSensorRange, &noise, 0.5f);
            const int n = core->buildBeams(raw, MclCore::SENSOR_COUNT,
                                           core->rawEstimate(), beams, valid);
            core->step(motion(0.0f, 0.0f, 0.0f, truth.theta), beams, n);
            if (t < 200) continue;
            // Averaged, not a single tick: with noise-free ranges both settings
            // centre on the truth and one draw of the wide cloud can land closer
            // than the tight one by luck.
            const MclCore::Estimate e = core->estimate();
            sum += static_cast<double>(e.pose.distanceTo(truth));
            sumCov += static_cast<double>(e.covXX);
            scored++;
        }
        err[mode] = static_cast<float>(sum / scored);
        cov[mode] = static_cast<float>(sumCov / scored);
    }
    std::printf("      decoupled: %.3f in (covXX %.4f); recoupled: %.3f in (covXX %.4f)\n",
                static_cast<double>(err[0]), static_cast<double>(cov[0]),
                static_cast<double>(err[1]), static_cast<double>(cov[1]));
    // Both the estimate and the cloud's own width have to improve, or the gain
    // is one lucky draw rather than a contracted posterior.
    CHECK(err[0] < err[1] * 0.5f);
    CHECK(cov[0] < cov[1] * 0.2f);
}

CHAD_TEST(mclFilter, recoversFromAKidnapWhenInjectionIsEnabled) {
    Cfg c;
    c.particleCount = 1024;
    c.resampleThreshold = 512;
    c.maxSensorRange = 140.0f;
    c.maxRandomFrac = 0.5f;
    c.injectResampleFloor = 0.02f;
    auto core = makeCore(c, 31337u);
    core->setSensorMounts(fourFaceMounts());

    Pose truth{-30.0f, 40.0f, 0.0f};
    core->setPose(truth);

    MclCore::RawSensor raw[MclCore::SENSOR_COUNT];
    MclCore::Beam beams[MclCore::SENSOR_COUNT];
    bool valid[MclCore::SENSOR_COUNT];
    auto run = [&](int ticks) {
        for (int t = 0; t < ticks; t++) {
            senseFrom(truth, 4, raw);
            const int n = core->buildBeams(raw, MclCore::SENSOR_COUNT,
                                           core->rawEstimate(), beams, valid);
            core->step(motion(0.0f, 0.0f, 0.0f, truth.theta), beams, n);
        }
    };

    run(80);
    CHECK(core->estimate().pose.distanceTo(truth) < 3.0f);

    // Kidnap: 80 inches across the field, heading unchanged.
    truth.x = 50.0f;
    truth.y = -40.0f;

    float peakInject = 0.0f;
    int arrived = -1;
    for (int t = 0; t < 400; t++) {
        run(1);
        peakInject = std::max(peakInject, core->injectFraction());
        if (arrived < 0 && core->estimate().pose.distanceTo(truth) < 6.0f) arrived = t;
    }
    // The augmented-MCL likelihood gap has to notice, or nothing is injected.
    CHECK(peakInject > 0.0f);
    CHECK(arrived >= 0);

    // Asserting min over the 400 ticks, which is what this used to do, certifies
    // that the estimate passed within 6 in at some instant on its way elsewhere.
    // Recovery means arriving and then holding.
    double sum = 0.0, worst = 0.0;
    for (int t = 0; t < 100; t++) {
        run(1);
        const double e = static_cast<double>(core->estimate().pose.distanceTo(truth));
        sum += e;
        worst = std::fmax(worst, e);
    }
    std::printf("      kidnap: injected up to %.2f, first inside 6 in at tick %d, "
                "then mean %.3f in / worst %.3f in\n",
                static_cast<double>(peakInject), arrived, sum / 100.0, worst);
    CHECK(sum / 100.0 < 1.0);
    CHECK(worst < 3.0);
}

CHAD_TEST(mclFilter, survivesGarbageInputForever) {
    Cfg c;
    c.particleCount = 128;
    c.resampleThreshold = 64;
    auto core = makeCore(c, 8u);
    core->setSensorMounts(fourFaceMounts());

    MclCore::Beam beams[3];
    for (int t = 0; t < 300; t++) {
        beams[0] = {0, kNan, 1.0f};
        beams[1] = {1, 30.0f, kInf};
        beams[2] = {-1, 30.0f, 1.0f};   // out-of-range sensor index
        MclCore::MotionInput in = motion((t % 3 == 0) ? kNan : 0.5f,
                                         (t % 5 == 0) ? kInf : 0.0f, 0.02f,
                                         (t % 7 == 0) ? kNan : 0.1f);
        in.dtSeconds = (t % 11 == 0) ? 0.0f : 0.025f;
        const Pose pose = core->step(in, beams, 3);
        CHECK_FINITE(pose.x);
        CHECK_FINITE(pose.y);
        CHECK_FINITE(pose.theta);
    }
    const MclCore::Estimate est = core->estimate();
    CHECK_FINITE(est.nEff);
    CHECK_FINITE(est.covXX);
    CHECK_FINITE(est.covTheta);
    CHECK(est.nEff > 0.0f);
    for (int i = 0; i < 128; i++) {
        CHECK_FINITE(core->particles()[i].pose.x);
        CHECK_FINITE(core->particles()[i].pose.y);
        CHECK_FINITE(core->particles()[i].pose.theta);
    }
}

CHAD_TEST(mclFilter, uniformResetSpreadsHeadingUniformly) {
    Cfg c;
    c.particleCount = 1024;
    auto core = makeCore(c, 55u);
    core->uniformReset();

    int quadrant[4] = {};
    for (int i = 0; i < 1024; i++) {
        const Pose& p = core->particles()[i].pose;
        CHECK(std::fabs(p.x) <= Field::FIELD_HALF_WALL);
        CHECK(std::fabs(p.y) <= Field::FIELD_HALF_WALL);
        CHECK(p.theta >= -PI && p.theta <= PI);
        quadrant[static_cast<int>((p.theta + PI) / HALF_PI) & 3]++;
    }
    // A Gaussian heading draw, which is what this used to be, puts over half
    // the cloud in the two middle quadrants and leaves the rest unwrapped.
    for (int q = 0; q < 4; q++) {
        CHECK(quadrant[q] > 180);
        CHECK(quadrant[q] < 330);
    }
}

CHAD_TEST(mclFilter, seedingMakesARunReproducible) {
    Cfg c;
    c.particleCount = 256;
    auto a = makeCore(c, 1u);
    auto b = makeCore(c, 1u);
    auto d = makeCore(c, 2u);
    for (auto* core : {a.get(), b.get(), d.get()}) {
        core->setSensorMounts(fourFaceMounts());
        core->setPose({10.0f, 10.0f, 0.2f});
        for (int t = 0; t < 20; t++) {
            core->step(motion(0.4f, 0.0f, 0.01f, 0.2f), nullptr, 0);
        }
    }
    CHECK_EQ(a->estimate().pose.x, b->estimate().pose.x);
    CHECK_EQ(a->estimate().pose.y, b->estimate().pose.y);
    CHECK(d->estimate().pose.x != a->estimate().pose.x);
}

// ===========================================================================
// The MclTracking seam
//
// MclTracking itself is 273 lines of PROS: the concurrency, the hardware reads
// and the odometry writeback, with no host coverage at all. mcl_adapter.hpp is
// what could be lifted out of it, and this is that part under test.
// ===========================================================================

CHAD_TEST(mclAdapter, publishedEstimateIsCoherentAndNeverStale) {
    mcladapter::Published<MclCore::Estimate> pub;

    // Nothing published yet: nEff 0, which is the "no estimate" signal every
    // consumer already gates on.
    CHECK_EQ(pub.generation(), 0u);
    CHECK_EQ(pub.load().nEff, 0.0f);

    MclCore::Estimate a{};
    a.pose = {1.0f, 2.0f, 0.3f};
    a.nEff = 900.0f;
    a.covXX = 0.4f;
    pub.store(a);
    CHECK_EQ(pub.generation(), 1u);
    CHECK_EQ(pub.load().pose.x, 1.0f);
    CHECK_EQ(pub.load().nEff, 900.0f);

    // A second store lands in the other slot and is what readers then see. The
    // old code dropped the whole tick whenever a 5 ms lock take failed, so the
    // published estimate went stale with nothing to say it had.
    MclCore::Estimate b = a;
    b.pose = {-4.0f, 9.0f, -1.2f};
    b.nEff = 100.0f;
    pub.store(b);
    CHECK_EQ(pub.generation(), 2u);
    CHECK_EQ(pub.load().pose.x, -4.0f);
    CHECK_EQ(pub.load().nEff, 100.0f);
}

CHAD_TEST(mclAdapter, concurrentReadsAreNeverTorn) {
    // The defect this replaces: reading the published struct without the lock,
    // which is what the old estimate() returned on a lock-take timeout -- x from
    // tick N+1 with y from tick N, or a pose carrying the wrong covariance. The
    // writer below only ever publishes structs whose fields are all equal, so
    // any field disagreeing with another is a torn read.
    mcladapter::Published<MclCore::Estimate> pub;
    std::atomic<bool> done{false};
    std::atomic<bool> started{false};
    std::atomic<long long> torn{0};
    std::atomic<long long> real{0};
    std::atomic<long long> reads{0};

    std::thread reader([&]() {
        started.store(true, std::memory_order_release);
        while (!done.load(std::memory_order_acquire)) {
            const MclCore::Estimate e = pub.load();
            const float v = e.pose.x;
            if (e.pose.y != v || e.pose.theta != v || e.nEff != v || e.covXX != v ||
                e.covXY != v || e.covYY != v || e.covTheta != v) {
                torn.fetch_add(1, std::memory_order_relaxed);
            }
            if (v != 0.0f) real.fetch_add(1, std::memory_order_relaxed);
            reads.fetch_add(1, std::memory_order_relaxed);
        }
    });

    // Wait for the reader to exist. A fixed write count races the thread start
    // and can finish first, which leaves the reader nothing to tear.
    for (int spin = 0; spin < 1000000 && !started.load(std::memory_order_acquire);
         spin++) {
        std::this_thread::yield();
    }
    long long writes = 0;
    while (reads.load(std::memory_order_relaxed) < 20000 && writes < 20000000) {
        const float v = static_cast<float>(++writes);
        MclCore::Estimate e;
        e.pose = {v, v, v};
        e.nEff = v;
        e.covXX = e.covXY = e.covYY = e.covTheta = v;
        pub.store(e);
    }
    done.store(true, std::memory_order_release);
    reader.join();

    std::printf("      %lld writes, %lld reads, %lld non-empty, %lld torn\n", writes,
                reads.load(), real.load(), torn.load());
    CHECK_EQ(torn.load(), 0LL);
    CHECK(reads.load() > 0);
    // A load() that always answered with a default-constructed struct would pass
    // the tearing check without having read anything.
    CHECK(real.load() > 0);
}

CHAD_TEST(mclAdapter, sensorGateTimersExpireAndPermanentOnesDoNot) {
    mcladapter::SensorGate<MclCore::SENSOR_COUNT> gate;
    for (int i = 0; i < MclCore::SENSOR_COUNT; i++) CHECK(!gate.disabled[i]);

    // A timed disable comes back on its own.
    gate.disableFor(2, 60.0f);
    CHECK(gate.disabled[2]);
    gate.tick(25.0f);
    CHECK(gate.disabled[2]);
    gate.tick(25.0f);
    CHECK(gate.disabled[2]);
    gate.tick(25.0f);
    CHECK(!gate.disabled[2]);
    CHECK_EQ(gate.timers[2], 0.0f);

    // A plain disable has no timer, so no number of ticks re-enables it. The
    // slot of a sensor that is not fitted is held off this way.
    gate.disable(3);
    for (int t = 0; t < 200; t++) gate.tick(25.0f);
    CHECK(gate.disabled[3]);
    gate.enable(3);
    CHECK(!gate.disabled[3]);

    // A non-positive duration is a permanent disable, not one that expires on
    // the next tick and so reads as doing nothing.
    gate.disableFor(4, 0.0f);
    gate.tick(25.0f);
    CHECK(gate.disabled[4]);

    // Out of range is a no-op rather than a write past the array.
    gate.disable(-1);
    gate.disable(MclCore::SENSOR_COUNT);
    gate.disableFor(99, 100.0f);
    gate.enable(-5);
    for (int i = 0; i < MclCore::SENSOR_COUNT; i++) {
        if (i == 4) continue;
        CHECK(!gate.disabled[i]);
    }
}

CHAD_TEST(mclAdapter, rawClassificationRejectsFaultCodes) {
    // PROS_ERR is INT32_MAX and would sail past any bound in millimetres.
    const MclCore::RawSensor err = mcladapter::classifyRaw(true, 2147483647, 0, false);
    CHECK(!err.present);
    const MclCore::RawSensor neg = mcladapter::classifyRaw(true, -1, 30, false);
    CHECK(!neg.present);
    const MclCore::RawSensor absent = mcladapter::classifyRaw(false, 1000, 63, false);
    CHECK(!absent.present);
    CHECK_EQ(absent.mm, 0);

    // A no-return is a real reading and must survive to reach the p_max term.
    const MclCore::RawSensor far = mcladapter::classifyRaw(true, 9500, 0, false);
    CHECK(far.present);
    CHECK_EQ(far.mm, 9500);

    const MclCore::RawSensor good = mcladapter::classifyRaw(true, 1234, 63, false);
    CHECK(good.present);
    CHECK_EQ(good.mm, 1234);
    CHECK_EQ(good.confidence, 63);
    CHECK(!good.suppressed);

    // Confidence outside the 0..63 the V5 reports is a fault too, and must not
    // end up scaling a sigma by a garbage number.
    CHECK_EQ(mcladapter::classifyRaw(true, 1000, -7, false).confidence, 0);
    CHECK_EQ(mcladapter::classifyRaw(true, 1000, 900, false).confidence, 0);

    CHECK(mcladapter::classifyRaw(true, 1000, 63, true).suppressed);
}

CHAD_TEST(mclAdapter, referencePoseIgnoresAHeldImu) {
    const Pose cloud{10.0f, -20.0f, 1.0f};

    // Live IMU: its heading wins, because it is current and free.
    const Pose live = mcladapter::referencePose(cloud, 0.4f, true);
    CHECK_NEAR(live.theta, 0.4f, 1e-6f);
    CHECK_EQ(live.x, 10.0f);

    // Faulted or absent IMU: the two-argument OdomTracking overload returns
    // false and leaves a HELD value in `out`. That value is finite and looks
    // exactly like a robot that is not turning, so taking it would pin the
    // reference heading at whatever it read when the sensor died.
    const Pose held = mcladapter::referencePose(cloud, 0.4f, false);
    CHECK_NEAR(held.theta, 1.0f, 1e-6f);

    // And a non-finite read is refused even when the flag says it is good.
    const Pose bad = mcladapter::referencePose(cloud, kNan, true);
    CHECK_NEAR(bad.theta, 1.0f, 1e-6f);
}

CHAD_TEST(mclAdapter, syncStepLerpsAndCrossesTheBranchCut) {
    const Pose odom{0.0f, 0.0f, 3.10f};
    const Pose est{10.0f, -4.0f, -3.10f};

    const Pose out = mcladapter::syncStep(odom, est, 0.25f, 0.5f);
    CHECK_NEAR(out.x, 2.5f, 1e-5f);
    CHECK_NEAR(out.y, -1.0f, 1e-5f);
    // Half of the 0.083 rad angular difference the short way round, not half of
    // the 6.2 rad subtraction.
    CHECK_NEAR(std::fabs(angleDiffRad(out.theta, 3.10f)),
               0.5f * std::fabs(angleDiffRad(3.10f, -3.10f)), 1e-4f);
    CHECK(std::fabs(out.theta) > 3.10f);

    // A zero proportion leaves odometry alone; one takes the estimate whole.
    const Pose none = mcladapter::syncStep(odom, est, 0.0f, 0.0f);
    CHECK_NEAR(none.x, 0.0f, 1e-6f);
    CHECK_NEAR(none.theta, 3.10f, 1e-6f);
    const Pose all = mcladapter::syncStep(odom, est, 1.0f, 1.0f);
    CHECK_NEAR(all.x, 10.0f, 1e-5f);
    CHECK_NEAR(std::fabs(angleDiffRad(all.theta, -3.10f)), 0.0f, 1e-4f);

    // Out-of-range proportions are clamped rather than overshooting the estimate.
    const Pose over = mcladapter::syncStep(odom, est, 4.0f, -1.0f);
    CHECK_NEAR(over.x, 10.0f, 1e-5f);
    CHECK_NEAR(over.theta, 3.10f, 1e-6f);
}

CHAD_TEST(mclFilter, stepEstimateIsWeightedButARecomputedOneIsNot) {
    // resample() flattens every weight to 1, so an estimate recomputed after
    // step() reports nEff == N and a spread equal to the roughening kernel no
    // matter what the measurements said. That is why the published estimate has
    // to be the one step() computed: nEff is the filter's health metric, and a
    // metric pinned at its maximum cannot warn about anything.
    Cfg c;
    c.particleCount = 256;
    c.resampleThreshold = 128;
    c.maxSensorRange = 140.0f;
    // Spread pinned, not inherited. The premise here is a CONVERGED cloud that
    // has been displaced, so the width is stated rather than tracking a knob
    // that exists for the opposite case.
    c.acquireVariance = 1.5f;
    c.acquireThetaVariance = 0.01f;
    auto core = makeCore(c, 20260823u);
    core->setSensorMounts(fourFaceMounts());

    const Pose truth{20.0f, -15.0f, 0.3f};
    core->setPose({truth.x - 6.0f, truth.y + 5.0f, truth.theta});

    MclCore::RawSensor raw[MclCore::SENSOR_COUNT];
    senseFrom(truth, 4, raw);
    MclCore::Beam beams[MclCore::SENSOR_COUNT];
    bool valid[MclCore::SENSOR_COUNT];

    const float n = static_cast<float>(c.particleCount);
    float worstStep = n;
    for (int tick = 0; tick < 40; tick++) {
        const int count = core->buildBeams(raw, MclCore::SENSOR_COUNT,
                                           core->rawEstimate(), beams, valid);
        core->step(motion(0.0f, 0.0f, 0.0f, truth.theta), beams, count);
        CHECK(core->resampledLast());
        worstStep = std::fmin(worstStep, core->stepEstimate().nEff);
        // The recomputed one is uniform every single tick, which is the bug.
        CHECK_NEAR(core->estimate().nEff, n, 1e-3f);
    }
    std::printf("      nEff: step %.1f vs recomputed %.1f of %d\n",
                static_cast<double>(worstStep),
                static_cast<double>(core->estimate().nEff), c.particleCount);
    CHECK(worstStep < 0.9f * n);
}

CHAD_TEST(mclFilter, effectiveSampleSizeIsBlindToATotalDisplacement) {
    // ESS measures weight *disparity*, so it collapses when a few particles fit
    // far better than the rest -- and says nothing when every particle is
    // equally wrong. A 38 in displacement is the second case: all 256 likelihoods
    // are near-identically tiny, log-sum-exp with max subtraction flattens them,
    // and nEff sits at N while the filter is completely lost. This is why ESS
    // alone cannot trigger recovery, and why likelihoodMismatch() exists.
    Cfg c;
    c.particleCount = 256;
    c.resampleThreshold = 128;
    c.maxSensorRange = 140.0f;
    // Spread pinned, not inherited. The premise here is a CONVERGED cloud that
    // has been displaced, so the width is stated rather than tracking a knob
    // that exists for the opposite case.
    c.acquireVariance = 1.5f;
    c.acquireThetaVariance = 0.01f;
    auto core = makeCore(c, 7u);
    core->setSensorMounts(fourFaceMounts());

    const Pose truth{0.0f, 0.0f, 0.0f};
    MclCore::RawSensor raw[MclCore::SENSOR_COUNT];
    senseFrom(truth, 4, raw);
    MclCore::Beam beams[MclCore::SENSOR_COUNT];
    bool valid[MclCore::SENSOR_COUNT];

    auto oneStepFrom = [&](float dx, float dy) {
        core->setPose({truth.x + dx, truth.y + dy, truth.theta});
        const int count = core->buildBeams(raw, MclCore::SENSOR_COUNT,
                                           core->rawEstimate(), beams, valid);
        core->step(motion(0.0f, 0.0f, 0.0f, truth.theta), beams, count);
        return core->stepEstimate().nEff;
    };

    const float n = static_cast<float>(c.particleCount);
    const float straddling = oneStepFrom(1.5f, -1.0f);
    const float lost = oneStepFrom(30.0f, -24.0f);
    std::printf("      nEff of %d: straddling %.1f, 38 in out %.1f\n",
                c.particleCount, static_cast<double>(straddling),
                static_cast<double>(lost));

    CHECK(straddling < 0.8f * n);   // partial fit is what ESS can see
    CHECK(lost > 0.9f * n);         // and a total loss is what it cannot
}



// ===========================================================================
// The search ramp's driver
// ===========================================================================

namespace {

/// Shipped-shaped filter: 1024 particles and config.cpp's sensor sigmas.
Cfg shippedish() {
    Cfg c;
    c.particleCount = 1024;
    c.resampleThreshold = 512;
    c.maxSensorRange = 140.0f;
    c.distResampleVariance = 1.5f;
    c.thetaResampleVariance = 0.01f;
    c.imeVariance = 0.004f;
    c.horizDependentVarianceProp = 0.004f;
    c.rangeSigmaCloseIn = 0.068f;
    return c;
}

/// A moving robot with honest odometry: ground truth advances and the filter is
/// handed the matching delta, so the motion model and the readings agree. The
/// earlier version of this fixture told the filter the robot had moved while
/// taking readings from a robot that had not, which measures the filter fighting
/// its own input rather than anything under test. A stationary robot is no good
/// either: it gets no new information, so it cannot relocate.
class Runner {
public:
    Runner(const Cfg& cfg, const Pose& start, std::uint32_t seed = 4242u)
        : mCfg(cfg), mTruth(start), mCore(makeCore(cfg, seed)) {
        mCore->setSensorMounts(fourFaceMounts());
        mCore->setPose(start);
    }

    void run(int ticks, float dvPerTick = 0.3f, float dThetaPerTick = 0.0f) {
        for (int t = 0; t < ticks; t++) {
            mTruth.theta = wrapRad(mTruth.theta + dThetaPerTick);
            const Rotation2d r = Rotation2d::fromRadians(mTruth.theta);
            mTruth.x += dvPerTick * r.cos;
            mTruth.y += dvPerTick * r.sin;
            senseFrom(mTruth, 4, mRaw, mCfg.maxSensorRange, &mNoise, 0.15f);
            for (int i = 0; i < MclCore::SENSOR_COUNT; i++) {
                if (mSpoof[i]) mRaw[i] = spoofed(i);
                if (mForced[i] > 0.0f) {
                    mRaw[i] = {static_cast<int>(mForced[i] * INCH_TO_MM), 63, true, false};
                }
                if (mBlind) mRaw[i].present = false;
            }
            const int n = mCore->buildBeams(mRaw, MclCore::SENSOR_COUNT,
                                            mCore->rawEstimate(), mBeams, mValid);
            mCore->step(motion(dvPerTick, 0.0f, dThetaPerTick, mTruth.theta, mHasHoriz),
                        mBeams, n);
        }
    }

    /// Make sensor `i` report `rangeIn` whatever the geometry says, until it is
    /// cleared with 0. One tick of this is the outlier the residual cap exists
    /// for -- a robot parked in a beam, or a post the map has in the wrong place.
    void forceReading(int i, float rangeIn) { mForced[i] = rangeIn; }
    /// Every sensor absent, so no beam scores at all. The only state in which
    /// resampleThreshold is reachable.
    void blind(bool on) { mBlind = on; }
    /// Default true, which is what the tests above were measured with. False is
    /// the shipped drivetrain: no dead wheels, so lateral travel is unmeasured
    /// and the cloud diffuses sideways at horizConstantNoise instead of not at
    /// all.
    void setHorizWheel(bool on) { mHasHoriz = on; }

    /// Ground truth moves; the filter is not told.
    void kidnap(float dx, float dy) {
        mTruth.x += dx;
        mTruth.y += dy;
    }

    /// Make sensor `i` always range a cylinder at `c` instead of the wall.
    void rangeObstacle(int i, const Circle& c) {
        mSpoof[i] = true;
        mSpoofCircle[i] = c;
    }
    void clearObstacles() {
        for (int i = 0; i < MclCore::SENSOR_COUNT; i++) mSpoof[i] = false;
    }

    MclCore& core() { return *mCore; }
    const Pose& truth() const { return mTruth; }
    float err() const { return mCore->stepEstimate().pose.distanceTo(mTruth); }

private:
    MclCore::RawSensor spoofed(int i) const {
        const std::array<Pose, MclCore::SENSOR_COUNT> mounts = fourFaceMounts();
        const Rotation2d r = Rotation2d::fromRadians(mTruth.theta);
        const float sx = mTruth.x + mounts[i].x * r.cos - mounts[i].y * r.sin;
        const float sy = mTruth.y + mounts[i].x * r.sin + mounts[i].y * r.cos;
        const Circle& c = mSpoofCircle[i];
        const float d = std::sqrt((c.x - sx) * (c.x - sx) + (c.y - sy) * (c.y - sy)) -
                        c.radius;
        MclCore::RawSensor out{};
        out.mm = static_cast<int>(d * INCH_TO_MM);
        out.confidence = 63;
        out.present = true;
        return out;
    }

    Cfg mCfg;
    Pose mTruth;
    std::unique_ptr<MclCore> mCore;
    Pcg32 mNoise{99u};
    MclCore::RawSensor mRaw[MclCore::SENSOR_COUNT]{};
    MclCore::Beam mBeams[MclCore::SENSOR_COUNT]{};
    bool mValid[MclCore::SENSOR_COUNT]{};
    bool mSpoof[MclCore::SENSOR_COUNT]{};
    Circle mSpoofCircle[MclCore::SENSOR_COUNT]{};
    float mForced[MclCore::SENSOR_COUNT]{};
    bool mBlind{false};
    bool mHasHoriz{true};
};

/// Standard deviation of the cloud's x, unweighted, which right after a resample
/// is the roughening kernel's own width.
float cloudSigmaX(const MclCore& core) {
    const int n = core.particleCount();
    double s = 0.0, s2 = 0.0;
    for (int i = 0; i < n; i++) {
        const double x = core.particles()[i].pose.x;
        s += x;
        s2 += x * x;
    }
    const double mean = s / n;
    return static_cast<float>(std::sqrt(std::fmax(s2 / n - mean * mean, 0.0)));
}

}  // namespace

CHAD_TEST(mclSearch, fitSigmasSeparatesTrackingFromLostByAnOrderOfMagnitude) {
    // The quantity the search ramp is driven by. A residual is in sigmas, so a
    // correctly placed estimate reads about 1 by construction and a displaced one
    // reads its displacement over the sensor sigma. The band can then come from
    // physics instead of from a fit, and that is the claim being pinned.
    //
    // The ramp is switched off and the cloud pinned narrow so this measures the
    // SIGNAL and not the recovery it drives: with the ramp live the cloud walks
    // to the truth and the residual being characterised disappears.
    Cfg c = shippedish();
    c.distResampleVariance = c.distSearchVariance = 0.3f;
    c.thetaResampleVariance = c.thetaSearchVariance = 0.003f;
    const Pose start{-20.0f, -15.0f, 0.4f};

    auto after = [&](float dx, float dy) {
        Runner r(c, start);
        r.run(60);
        r.kidnap(dx, dy);
        r.run(20);   // alphaFast is a ten-tick smoother; let it arrive
        return r.core().fitSigmas();
    };

    Runner base(c, start);
    base.run(60);
    const float tracking = base.core().fitSigmas();
    const float trackErr = base.err();
    const float off2 = after(2.0f, -1.5f);
    const float off10 = after(8.0f, -6.0f);
    const float kidnap = after(18.0f, 16.0f);
    std::printf("      fitSigmas: tracking %.2f (err %.2f in), 2.5 in %.2f, "
                "10 in %.2f, 24 in %.2f\n",
                static_cast<double>(tracking), static_cast<double>(trackErr),
                static_cast<double>(off2), static_cast<double>(off10),
                static_cast<double>(kidnap));

    CHECK(trackErr < 1.0f);              // it really is tracking
    CHECK(kidnap > 15.0f * tracking);    // an order of magnitude, not a threshold
    CHECK(off10 > off2);
    CHECK(off10 < kidnap);

    // Which is what puts the shipped band on the right side of both. These are
    // the assertions that matter: the ramp reads exactly this comparison.
    const Cfg d;
    CHECK(tracking / d.lostFitSigmas < d.searchMismatchFloor);
    CHECK(off2 / d.lostFitSigmas < d.searchMismatchFloor);
    CHECK(kidnap / d.lostFitSigmas > d.searchMismatchFloor);
}

CHAD_TEST(mclSearch, wideningTheCloudDoesNotMakeItLookMoreLost) {
    // The failure this design exists to avoid. The augmented-MCL fast/slow gap
    // is taken on the cloud's MEAN likelihood, which falls when the cloud merely
    // spreads out -- so a search width driven by it widens because it widened. In
    // the simulator that produced 60 in excursions from a filter that was never
    // lost. fitSigmas() scores the ESTIMATE, which a wider cloud does not move,
    // so the loop has no gain.
    const Pose start{-20.0f, -15.0f, 0.4f};
    const float widths[3] = {0.15f, 1.5f, 4.0f};
    float fit[3];
    for (int i = 0; i < 3; i++) {
        Cfg c = shippedish();
        c.distResampleVariance = widths[i];
        // Ramp disabled, so the width under test is the width the cloud has.
        // With the ramp live this would measure the loop instead of its gain.
        c.distSearchVariance = widths[i];
        Runner r(c, start);
        r.run(60);
        fit[i] = r.core().fitSigmas();
        std::printf("      roughening %5.2f in: err %.2f in, fitSigmas %.2f, "
                    "mean-based mismatch %.3f\n",
                    static_cast<double>(widths[i]), static_cast<double>(r.err()),
                    static_cast<double>(fit[i]),
                    static_cast<double>(r.core().likelihoodMismatch()));
    }
    // 27x the cloud width, and the lost-ness signal must stay below the ramp's
    // floor throughout. It may grow -- a wider cloud does estimate slightly
    // worse -- but not enough to call the filter lost.
    const Cfg d;
    for (int i = 0; i < 3; i++) {
        CHECK(fit[i] / d.lostFitSigmas < d.searchMismatchFloor);
    }
    CHECK(fit[2] < 4.0f * fit[0]);
}

CHAD_TEST(mclSearch, theRougheningReachesTheSearchWidthOnlyWhenLost) {
    // The ramp end to end, observed on the cloud rather than on the private
    // helper: right after a resample the spread IS the roughening kernel.
    Cfg c = shippedish();
    c.distResampleVariance = 0.15f;
    c.thetaResampleVariance = 0.001f;
    c.distSearchVariance = 6.0f;
    const Pose start{-20.0f, -15.0f, 0.4f};

    Runner keep(c, start);
    keep.run(60);
    const float tracking = cloudSigmaX(keep.core());

    Runner lostRun(c, start);
    lostRun.run(60);
    lostRun.kidnap(18.0f, 16.0f);
    // MAX over the window, not the width at tick 20. The ramp comes back down
    // as soon as the cloud arrives, so at a fixed tick a FASTER recovery reads
    // as a narrower cloud and looks exactly like a ramp that never fired.
    float lost = 0.0f;
    for (int t = 0; t < 20; t++) {
        lostRun.run(1);
        lost = std::fmax(lost, cloudSigmaX(lostRun.core()));
    }
    std::printf("      cloud sigma x: tracking %.2f in, peak over the 20 ticks "
                "after a 24 in kidnap %.2f in, ending %.2f in out\n",
                static_cast<double>(tracking), static_cast<double>(lost),
                static_cast<double>(lostRun.err()));

    // Tracking sits at the roughening, not somewhere between: a ramp that leaks
    // is what costs steady-state accuracy.
    CHECK(tracking < 3.0f * c.distResampleVariance);
    // And lost widens by several times that without reaching the full width:
    // four beams on a benign rectangle read about 6.5 sigmas for a 24 in jump,
    // against a band of 4 to 7.2. Widening is not the claim on its own -- the
    // cloud has to have arrived, or a ramp stuck open would pass.
    CHECK(lost > 3.0f * tracking);
    CHECK(lost < c.distSearchVariance);
    CHECK(lostRun.err() < 3.0f);
}

CHAD_TEST(mclSearch, mappedObstacleReadingsAreNotEvidenceOfBeingLost) {
    // Posts the map knows about, ranged instead of the wall. TWO of them, and
    // that is the point: the fit takes the median of the per-beam residuals, so
    // one unmapped structure is already free. Two is where the map has to earn
    // its keep, and where an unmapped pair costs 45 sigmas -- enough to declare a
    // perfectly healthy filter lost and scatter the cloud to the search width.
    const Cfg c = shippedish();
    const Pose start{-20.0f, -15.0f, 0.0f};
    const Circle posts[2] = {{start.x + 40.0f, start.y, 2.0f},
                             {start.x, start.y + 40.0f, 2.0f}};

    auto fitWith = [&](bool mapped) {
        Runner r(c, start);
        if (mapped) r.core().setDeadzoneCircles(posts, 2);
        r.rangeObstacle(0, posts[0]);
        r.rangeObstacle(1, posts[1]);
        r.run(40);
        return std::pair<float, float>{r.core().fitSigmas(), r.err()};
    };

    const auto mapped = fitWith(true);
    const auto unmapped = fitWith(false);
    std::printf("      two posts ranged: mapped %.2f sigmas (err %.2f in), "
                "unmapped %.2f sigmas (err %.2f in)\n",
                static_cast<double>(mapped.first), static_cast<double>(mapped.second),
                static_cast<double>(unmapped.first), static_cast<double>(unmapped.second));

    const Cfg d;
    CHECK(mapped.second < 1.5f);   // the filter is fine, so the fit must say so
    CHECK(mapped.first / d.lostFitSigmas < d.searchMismatchFloor);
    // The unmapped case is the honest cost of unmapped structures, and it has to
    // be the larger one or the assertion above proves nothing.
    CHECK(unmapped.first > 3.0f * mapped.first);
}

CHAD_TEST(mclSearch, oneUnmappedStructureIsAbsorbedByTheMedian) {
    // Why the fit is a median and not a mean. A single beam reading something
    // the map does not have -- a post moved, a robot parked in front of a sensor
    // -- is tens of sigmas on that beam. Averaged over four it is enough to cross
    // the ramp floor and scatter a healthy cloud; as a median of four it is
    // nothing. Disabling the deadzone model in the simulator costs 15 in of
    // tracking with the median and 30 in with the mean.
    const Cfg c = shippedish();
    const Pose start{-20.0f, -15.0f, 0.0f};
    const Circle post{start.x + 40.0f, start.y, 2.0f};

    Runner r(c, start);
    r.rangeObstacle(0, post);   // NOT mapped
    r.run(40);
    const float fit = r.core().fitSigmas();
    std::printf("      one unmapped post: fitSigmas %.2f, err %.2f in\n",
                static_cast<double>(fit), static_cast<double>(r.err()));

    const Cfg d;
    CHECK(r.err() < 1.5f);
    CHECK(fit / d.lostFitSigmas < d.searchMismatchFloor);
    // A mean of four would put that one beam's residual at a quarter of tens of
    // sigmas, which is over the floor. State the arithmetic so the claim is not
    // just an assertion about an implementation detail.
    CHECK(45.0f / 4.0f > d.lostFitSigmas * d.searchMismatchFloor);
}

CHAD_TEST(mclSearch, oneBadTickOnEveryBeamDoesNotScatterTheCloud) {
    // What the alphaFast smoothing is for. The median covers one bad BEAM; this
    // is one bad TICK on all of them -- a robot crossing every sensor at once, or
    // a single scheduling hiccup that ages the readings. Reacting to it inside
    // one tick widens the cloud to the search width and throws away a converged
    // filter, and in the simulator removing the smoothing costs a kidnap and
    // takes the worst-case tracking excursion from 7.5 in to 10.2.
    const Cfg c = shippedish();
    const Pose start{-20.0f, -15.0f, 0.0f};
    const Circle blocker{start.x + 12.0f, start.y, 8.0f};

    Runner r(c, start);
    r.run(60);
    const float before = r.core().fitSigmas();
    for (int i = 0; i < 4; i++) r.rangeObstacle(i, blocker);
    r.run(1);
    const float spike = r.core().fitSigmas();
    r.clearObstacles();
    r.run(10);
    const float after = r.core().fitSigmas();
    std::printf("      one blinded tick: fitSigmas %.2f -> %.2f -> %.2f\n",
                static_cast<double>(before), static_cast<double>(spike),
                static_cast<double>(after));

    const Cfg d;
    // The invariant the residual cap is derived from: one tick cannot cross the
    // ramp floor on its own, however large the residual behind it.
    CHECK(spike / d.lostFitSigmas < d.searchMismatchFloor);
    // And it decays back, rather than leaving the cloud searching for half a
    // second the way an uncapped residual did.
    CHECK(after < spike);
    CHECK(after / d.lostFitSigmas < d.searchMismatchFloor);
    CHECK(r.err() < 1.5f);
    // Not vacuous: that tick really did carry a large residual, so a filter with
    // no cap and no smoothing would have widened.
    CHECK(spike > 20.0f * before);
}

CHAD_TEST(mclSearch, resampleCountAccumulatesAndSetPoseClearsIt) {
    // resampledLast() answers only for the tick just gone, so a filter that has
    // stopped resampling intermittently reads healthy on any single sample of it.
    const Cfg c = shippedish();
    const Pose start{10.0f, 10.0f, 0.0f};
    Runner r(c, start);
    r.run(25);
    CHECK_EQ(r.core().resampleCount(), 25u);

    // No beams at all: step() resamples only if the cloud has degenerated, so
    // the count has to stop moving rather than accumulate a random walk.
    for (int i = 0; i < 10; i++) {
        r.core().step(motion(0.3f, 0.0f, 0.0f, start.theta), nullptr, 0);
    }
    const std::uint32_t blind = r.core().resampleCount();
    std::printf("      resamples: 25 driven, %u after 10 blind ticks\n", blind);
    CHECK(blind < 30u);

    r.core().setPose(start);
    CHECK_EQ(r.core().resampleCount(), 0u);
}

// ===========================================================================
// The search ramp, swept
// ===========================================================================
//
// Every knob here has a degenerate optimum on one metric alone, so all three are
// measured on the same trials: recovery inside a BUDGET (given 200 ticks the
// base roughening gets there from every setting, ramp included), tracking error,
// and what one outlier tick does. The outlier spoofs TWO beams of four: the
// level is a MEDIAN, so one wild beam cannot move it and the cap is irrelevant
// to it.

namespace {

/// Shipped sensor model, shipped search width.
Cfg rampBase() {
    Cfg c = shippedish();
    c.distSearchVariance = 12.0f;
    c.thetaSearchVariance = 0.01f;
    return c;
}

struct RampStats {
    int trials{0}, recovered{0}, tickSum{0};
    double steadySum{0.0};
    int steadyTicks{0}, spuriousTicks{0};
    float worstSteady{0.0f};
    int outlierTrials{0}, outlierFires{0};
    float outlierWorst{0.0f};

    double recPct() const { return trials ? 100.0 * recovered / trials : 0.0; }
    double meanTicks() const {
        return recovered ? static_cast<double>(tickSum) / recovered : 0.0;
    }
    double steadyMean() const { return steadyTicks ? steadySum / steadyTicks : 0.0; }
};

/// A 10 in circle. A straight leg supplies no parallax, so it cannot separate a
/// real fix from a wall-parallel alias.
constexpr float kRampDv = 0.3f;
constexpr float kRampDth = 0.03f;
constexpr int kRampBudget = 50;

const Pose kRampStarts[] = {{-20.0f, -15.0f, 0.4f},
                            {18.0f, 16.0f, -1.1f},
                            {0.0f, -22.0f, 2.2f},
                            {20.0f, -5.0f, 1.7f},
                            {-8.0f, 20.0f, -2.6f},
                            {12.0f, -18.0f, 0.9f}};

RampStats rampSweep(const Cfg& c) {
    RampStats out;
    std::uint32_t seed = 7001u;
    for (const Pose& s : kRampStarts) {
        // Tracking arm: no kidnap, so any widening here is a false alarm.
        {
            Runner r(c, s, seed++);
            r.setHorizWheel(false);
            r.run(60, kRampDv, kRampDth);
            for (int t = 0; t < 120; t++) {
                r.run(1, kRampDv, kRampDth);
                const float e = r.err();
                out.steadySum += static_cast<double>(e);
                out.steadyTicks++;
                if (e > out.worstSteady) out.worstSteady = e;
                if (r.core().searchMismatch() > c.searchMismatchFloor) out.spuriousTicks++;
            }
        }

        // Outlier arm. Two beams at 4 in for one tick: close readings take
        // rangeSigmaCloseIn, so 4 in against a 60 in wall is some 800 sigmas,
        // and half the beams is enough to carry the median.
        {
            Runner r(c, s, seed++);
            r.setHorizWheel(false);
            r.run(60, kRampDv, kRampDth);
            r.forceReading(0, 4.0f);
            r.forceReading(1, 4.0f);
            r.run(1, kRampDv, kRampDth);
            r.forceReading(0, 0.0f);
            r.forceReading(1, 0.0f);
            bool fired = false;
            float worst = 0.0f;
            for (int t = 0; t < 60; t++) {
                r.run(1, kRampDv, kRampDth);
                if (r.core().searchMismatch() > c.searchMismatchFloor) fired = true;
                worst = std::fmax(worst, r.err());
            }
            out.outlierTrials++;
            if (fired) out.outlierFires++;
            out.outlierWorst = std::fmax(out.outlierWorst, worst);
        }

        // Recovery arm.
        for (int d = 0; d < 8; d++) {
            const float a = static_cast<float>(d) * (2.0f * PI / 8.0f);
            Runner r(c, s, seed++);
            r.setHorizWheel(false);
            r.run(60, kRampDv, kRampDth);
            r.kidnap(36.0f * std::cos(a), 36.0f * std::sin(a));
            out.trials++;
            int hit = -1;
            for (int t = 1; t <= kRampBudget; t++) {
                r.run(1, kRampDv, kRampDth);
                if (hit < 0 && r.err() < 3.0f) hit = t;
            }
            // Inside 3 in at some instant is not recovery; it has to still be
            // there when the budget runs out.
            if (hit >= 0 && r.err() < 3.0f) {
                out.recovered++;
                out.tickSum += hit;
            }
        }
    }
    return out;
}

void rampRow(const char* label, const RampStats& s) {
    std::printf("        %-24s rec %2d/%-2d %5.1f%% in %5.1f ticks   "
                "steady %.2f (worst %.2f) spurious %d/%d   "
                "outlier fires %d/%d worst %.1f in\n",
                label, s.recovered, s.trials, s.recPct(), s.meanTicks(),
                s.steadyMean(), static_cast<double>(s.worstSteady),
                s.spuriousTicks, s.steadyTicks, s.outlierFires, s.outlierTrials,
                static_cast<double>(s.outlierWorst));
}

}  // namespace

CHAD_TEST(mclRamp, theResidualCapTradesOutlierImmunityAgainstRecovery) {
    // What the coefficient buys is stated as an inequality in updateFit: one
    // tick crosses the ramp floor only from a standing level above
    // (1 - lostFitCapProp) * lost * floor / (1 - alphaFast). `f0` below is that
    // bound, and the tracking column is the level it has to be compared with.
    const Cfg base = rampBase();
    const float band = base.lostFitSigmas * base.searchMismatchFloor;
    std::printf("      cap coefficient; ramp floor %.1f sigmas, alphaFast %.2f\n",
                static_cast<double>(band), static_cast<double>(base.alphaFast));
    for (float prop : {0.0f, 0.4f, 0.8f, 1.0f, 1.4f, 3.0f}) {
        Cfg c = base;
        c.lostFitCapProp = prop;
        const double cap = prop > 0.0f
            ? static_cast<double>(prop * band / base.alphaFast) : 0.0;
        const double reach = (prop > 0.0f && prop < 1.0f)
            ? static_cast<double>((1.0f - prop) * band / (1.0f - base.alphaFast))
            : 0.0;
        char label[64];
        std::snprintf(label, sizeof label, "prop %.1f cap %.0f f0>%.2f",
                      static_cast<double>(prop), cap, reach);
        rampRow(label, rampSweep(c));
    }

    // Uncapped, one outlier tick is hundreds of sigmas and reaches the floor
    // from a filter that is tracking; capped, it cannot. Both on the same trials.
    Cfg off = base;
    off.lostFitCapProp = 0.0f;
    const RampStats uncapped = rampSweep(off);
    const RampStats capped = rampSweep(base);
    CHECK(uncapped.outlierFires > capped.outlierFires);
    CHECK_EQ(capped.outlierFires, 0);
    // And it is not paid for with the recovery the ramp exists for.
    CHECK(capped.recovered >= uncapped.recovered - 1);
    // The excursion the false alarm causes, which is the reason to care.
    CHECK(uncapped.outlierWorst > capped.outlierWorst);
}

CHAD_TEST(mclRamp, theLostThresholdAndSmootherAreSweptAgainstBothSides) {
    const Cfg base = rampBase();
    std::printf("      lostFitSigmas x searchMismatchFloor (band width held at 0.4)\n");
    for (float lost : {4.0f, 8.0f, 12.0f, 18.0f, 24.0f}) {
        for (float floor : {0.3f, 0.5f, 0.7f}) {
            Cfg c = base;
            c.lostFitSigmas = lost;
            c.searchMismatchFloor = floor;
            c.searchMismatchFull = floor + 0.4f;
            char label[64];
            std::snprintf(label, sizeof label, "lost %4.1f floor %.1f",
                          static_cast<double>(lost), static_cast<double>(floor));
            rampRow(label, rampSweep(c));
        }
    }

    std::printf("      searchMismatchFull, floor held at 0.5\n");
    for (float full : {0.6f, 0.9f, 1.2f}) {
        Cfg c = base;
        c.searchMismatchFull = full;
        char label[64];
        std::snprintf(label, sizeof label, "full %.1f", static_cast<double>(full));
        rampRow(label, rampSweep(c));
    }

    std::printf("      alphaFast, the smoother the level rises on\n");
    for (float a : {0.05f, 0.10f, 0.20f, 0.35f}) {
        Cfg c = base;
        c.alphaFast = a;
        char label[64];
        std::snprintf(label, sizeof label, "alphaFast %.2f", static_cast<double>(a));
        rampRow(label, rampSweep(c));
    }

    // A ramp that never fires is the failure this design replaced; a ramp that
    // fires on noise is the failure that replaced it. Both, from one sweep.
    Cfg dead = base;
    dead.lostFitSigmas = 0.0f;   // documented as disabling the ramp
    const RampStats none = rampSweep(dead);
    const RampStats live = rampSweep(base);
    std::printf("      ramp off: rec %d/%d; ramp on: rec %d/%d\n",
                none.recovered, none.trials, live.recovered, live.trials);
    CHECK(live.recovered > none.recovered);
    CHECK_EQ(none.spuriousTicks, 0);   // it really is off
    CHECK_EQ(live.spuriousTicks, 0);   // and healthy tracking does not reach it
}

CHAD_TEST(mclRamp, essThresholdIsUnreachableAndThatIsTheRightSide) {
    // step() resamples unconditionally whenever a beam scored, so
    // resampleThreshold cannot be read on any tick that has a measurement.
    // Pinning that is the point: a number nothing reads cannot be tuned.
    const Cfg base = rampBase();
    {
        Runner r(base, kRampStarts[0], 91u);
        r.setHorizWheel(false);
        r.run(30, kRampDv, kRampDth);
        const std::uint32_t before = r.core().resampleCount();
        r.run(40, kRampDv, kRampDth);
        CHECK_EQ(r.core().resampleCount() - before, 40u);
    }

    // Blind for 60 ticks while the robot keeps driving. The gate is
    // nEff < threshold/2 and a cloud with no measurement has flat weights, so
    // nEff is exactly N: nothing below 2N fires, and what fires above it is a
    // roughening random walk on top of odometry with no correction.
    std::printf("      resampleThreshold over a 60-tick blind window, N = %d\n",
                base.particleCount);
    for (int thresh : {512, 2048, 2049, 3072, 4096}) {
        Cfg c = base;
        c.resampleThreshold = thresh;
        double errSum = 0.0;
        float worst = 0.0f;
        std::uint32_t resamples = 0;
        int ticks = 0;
        for (int si = 0; si < 3; si++) {
            Runner r(c, kRampStarts[si], 601u);
            r.setHorizWheel(false);
            r.run(60, kRampDv, kRampDth);
            const std::uint32_t r0 = r.core().resampleCount();
            r.blind(true);
            for (int t = 0; t < 60; t++) {
                r.run(1, kRampDv, kRampDth);
                errSum += static_cast<double>(r.err());
                worst = std::fmax(worst, r.err());
                ticks++;
            }
            resamples += r.core().resampleCount() - r0;
        }
        std::printf("        thresh %5d  blind resamples %3u  mean err %.3f  worst %.3f\n",
                    thresh, resamples, errSum / ticks, static_cast<double>(worst));
    }

    // The shipped 512 is on the side that does nothing during a dropout, and
    // that is the correct side: with no measurement, resampling only adds a
    // random walk, and holding the cloud costs nothing.
    Cfg forced = base;
    forced.resampleThreshold = 4096;
    double worst[2] = {0.0, 0.0};
    for (int arm = 0; arm < 2; arm++) {
        Runner r(arm == 0 ? base : forced, kRampStarts[0], 601u);
        r.setHorizWheel(false);
        r.run(60, kRampDv, kRampDth);
        r.blind(true);
        float w = 0.0f;
        for (int t = 0; t < 60; t++) {
            r.run(1, kRampDv, kRampDth);
            w = std::fmax(w, r.err());
        }
        worst[arm] = static_cast<double>(w);
    }
    std::printf("      blind worst error: shipped %.3f in, forced-resample %.3f in\n",
                worst[0], worst[1]);
    CHECK(worst[0] <= worst[1]);
}
