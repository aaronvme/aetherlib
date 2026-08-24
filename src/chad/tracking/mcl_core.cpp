#include "chad/tracking/mcl_core.hpp"

#include <algorithm>
#include <cmath>

namespace chad {

namespace {

/// V5 distance sensor reports this or more when nothing is in range.
constexpr int kNoReturnMm = 9000;
/// A disconnected sensor reports PROS_ERR == INT32_MAX, which is not a
/// no-return: it carries no information at all.
constexpr int kSensorErrorMm = 20000;
/// Below this the sensor is in its fixed-noise regime.
constexpr int kCloseRangeMm = 200;

/// Beam mixture at a residual in sigmas, with the unexpected-obstacle term
/// under caller control so a table node can hold either one-sided limit at a
/// zero residual.
float mixtureAt(const MclCore::Config& c, float sigmas, bool withShort) {
    float p = c.hitWeight * std::exp(-0.5f * sigmas * sigmas) + c.randWeight;
    if (withShort) {
        p += c.shortWeight * std::exp(sigmas / std::max(c.shortDecaySigmas, 0.05f));
    }
    return std::max(p, 1e-30f);
}

}  // namespace

float Pcg32::normal() {
    if (mHaveSpare) {
        mHaveSpare = false;
        return mSpare;
    }
    float u, v, s;
    do {
        u = 2.0f * uniform() - 1.0f;
        v = 2.0f * uniform() - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s <= 1e-20f);
    const float f = std::sqrt(-2.0f * std::log(s) / s);
    mSpare = v * f;
    mHaveSpare = true;
    return u * f;
}

MclCore::MclCore() : MclCore(Config{}) {}

MclCore::MclCore(const Config& config, std::uint32_t seed)
    : mConfig(config), mMap(mclfield::map()), mRng(seed) {
    mConfig.particleCount = clamp(mConfig.particleCount, 1, PARTICLE_MAX);
    buildLut();
    setPose(Pose{});
}

void MclCore::setConfig(const Config& config) {
    const int held = mConfig.particleCount;
    mConfig = config;
    mConfig.particleCount = held;
    buildLut();
}

void MclCore::setSeed(std::uint32_t seed) { mRng.reseed(seed); }

void MclCore::setBlockers(const LineObstacle* lines, int count) {
    mBlockers = lines;
    mBlockerCount = (lines != nullptr && count > 0) ? count : 0;
}

void MclCore::setDeadzoneCircles(const CircleObstacle* circles, int count) {
    mDeadzoneCircles = circles;
    mDeadzoneCircleCount = (circles != nullptr && count > 0) ? count : 0;
}

void MclCore::setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts) {
    mMounts = mounts;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        mMountRots[i] = Rotation2d::fromRadians(mounts[i].theta);
    }
}

// ---------------------------------------------------------------------------
// Beam mixture
// ---------------------------------------------------------------------------

float MclCore::logBeamLikelihoodExact(const Config& config, float sigmas) {
    // Short readings mean something unmodelled is in the way; long ones have no
    // such excuse, so the tail is one-sided.
    return std::log(mixtureAt(config, sigmas, sigmas < 0.0f));
}

float MclCore::logMixtureNorm(const Config& config) {
    // Integral of the sigma-space mixture over the tabulated span: the Gaussian
    // contributes sqrt(2 pi), the one-sided exponential its decay length, and
    // the floor the width of the span.
    const float decay = std::max(config.shortDecaySigmas, 0.05f);
    const float total =
        config.hitWeight * 2.5066283f +
        config.shortWeight * decay * (1.0f - std::exp(-LUT_SIGMA_SPAN / decay)) +
        config.randWeight * 2.0f * LUT_SIGMA_SPAN;
    return std::log(std::max(total, 1e-30f));
}

void MclCore::buildLut() {
    // Node spacing 8/511, and index LUT_HALF-1 / LUT_HALF are both a zero
    // residual: the first holds the limit from below (short term present), the
    // second the limit from above. A single table straddling zero smears the
    // 0.405 nat step over one cell and is wrong by 0.203 nats at its middle.
    const float step = LUT_SIGMA_SPAN / static_cast<float>(LUT_HALF - 1);
    for (int i = 0; i < LUT_HALF; i++) {
        const float s = -LUT_SIGMA_SPAN + static_cast<float>(i) * step;
        mLut[i] = std::log(mixtureAt(mConfig, s, true));
        mLut[LUT_HALF + i] = std::log(mixtureAt(mConfig, static_cast<float>(i) * step,
                                                false));
    }
    mLogMixtureNorm = logMixtureNorm(mConfig);
}

float MclCore::logBeamLikelihood(float sigmas) const {
    // A non-finite residual would index the table with UB. The long tail is the
    // least informative value in it, so that is the safe answer.
    if (!std::isfinite(sigmas)) return mLut[LUT_SIZE - 1];
    const float u = clamp(sigmas, -LUT_SIGMA_SPAN, LUT_SIGMA_SPAN);
    const float invStep = static_cast<float>(LUT_HALF - 1) / LUT_SIGMA_SPAN;
    const float* half = (u < 0.0f) ? mLut : mLut + LUT_HALF;
    const float t = (u < 0.0f) ? (u + LUT_SIGMA_SPAN) * invStep : u * invStep;
    const int i = clamp(static_cast<int>(t), 0, LUT_HALF - 2);
    const float f = t - static_cast<float>(i);
    return half[i] + (half[i + 1] - half[i]) * f;
}

// ---------------------------------------------------------------------------
// Ray casting
// ---------------------------------------------------------------------------

float MclCore::intersectLine(float ox, float oy, float dirCos, float dirSin,
                             const LineObstacle& line, float maxRange) {
    const float x1 = line.p1.x, y1 = line.p1.y;
    const float x2 = line.p2.x, y2 = line.p2.y;

    // A ray that starts beyond the segment's extent and travels further that
    // way can never come back. Written with >= / <= so a ray with a zero
    // component is rejected on the other axis rather than solved for.
    const float xMin = std::min(x1, x2), xMax = std::max(x1, x2);
    const float yMin = std::min(y1, y2), yMax = std::max(y1, y2);
    if ((dirCos >= 0.0f && ox > xMax) || (dirCos <= 0.0f && ox < xMin) ||
        (dirSin >= 0.0f && oy > yMax) || (dirSin <= 0.0f && oy < yMin)) {
        return maxRange;
    }

    const float ex = x2 - x1, ey = y2 - y1;
    const float den = dirCos * ey - dirSin * ex;   // dir x edge
    const float wx = x1 - ox, wy = y1 - oy;
    const float t = (wx * ey - wy * ex) / den;     // along the ray, inches
    const float u = (wx * dirSin - wy * dirCos) / den;   // along the segment
    // Tested positively so that a parallel ray, which divides by zero above and
    // yields inf or NaN, falls through to the miss rather than past the guard.
    if (t >= 0.0f && t <= maxRange && u >= 0.0f && u <= 1.0f) return t;
    return maxRange;
}

float MclCore::intersectCircle(float ox, float oy, float dirCos, float dirSin,
                               const CircleObstacle& circle, float maxRange) {
    const float fx = ox - circle.x, fy = oy - circle.y;
    const float reach = maxRange + circle.radius;
    const float d2 = fx * fx + fy * fy;
    if (d2 > reach * reach) return maxRange;

    const float b = fx * dirCos + fy * dirSin;
    const float c = d2 - circle.radius * circle.radius;
    const float disc = b * b - c;
    if (!(disc >= 0.0f)) return maxRange;
    const float root = std::sqrt(disc);

    const float t1 = -b - root;
    if (t1 >= 0.0f && t1 <= maxRange) return t1;
    const float t2 = -b + root;
    if (t2 >= 0.0f && t2 <= maxRange) return t2;
    return maxRange;
}

float MclCore::castRay(float x, float y, float dirCos, float dirSin) const {
    // Nearest hit over the whole map. Testing one class of geometry only when
    // another produced no hit lets a wall closer than a structure be ignored.
    float best = mConfig.maxSensorRange;
    for (int i = 0; i < mMap.lineCount; i++) {
        const float d = intersectLine(x, y, dirCos, dirSin, mMap.lines[i], best);
        if (d < best) best = d;
    }
    for (int i = 0; i < mMap.circleCount; i++) {
        const float d = intersectCircle(x, y, dirCos, dirSin, mMap.circles[i], best);
        if (d < best) best = d;
    }
    return best;
}

float MclCore::castRayIncidence(float x, float y, float dirCos, float dirSin,
                                float* incidenceRad) const {
    float best = mConfig.maxSensorRange;
    int bestLine = -1, bestCircle = -1;
    for (int i = 0; i < mMap.lineCount; i++) {
        const float d = intersectLine(x, y, dirCos, dirSin, mMap.lines[i], best);
        if (d < best) {
            best = d;
            bestLine = i;
            bestCircle = -1;
        }
    }
    for (int i = 0; i < mMap.circleCount; i++) {
        const float d = intersectCircle(x, y, dirCos, dirSin, mMap.circles[i], best);
        if (d < best) {
            best = d;
            bestCircle = i;
            bestLine = -1;
        }
    }

    // cos of the angle to the surface normal. A miss leaves it at 1, i.e. no
    // grazing penalty on a reading that reached nothing.
    float cosInc = 1.0f;
    if (bestLine >= 0) {
        const LineObstacle& l = mMap.lines[bestLine];
        const float ex = l.p2.x - l.p1.x, ey = l.p2.y - l.p1.y;
        const float len = std::sqrt(ex * ex + ey * ey);
        if (len > 1e-6f) cosInc = std::fabs((dirCos * ey - dirSin * ex) / len);
    } else if (bestCircle >= 0) {
        const CircleObstacle& c = mMap.circles[bestCircle];
        const float hx = x + dirCos * best - c.x, hy = y + dirSin * best - c.y;
        const float len = std::sqrt(hx * hx + hy * hy);
        if (len > 1e-6f) cosInc = std::fabs((dirCos * hx + dirSin * hy) / len);
    }
    if (incidenceRad != nullptr) {
        *incidenceRad = std::acos(clamp(cosInc, 0.0f, 1.0f));
    }
    return best;
}

float MclCore::expectedRange(const Pose& pose, int sensor) const {
    if (sensor < 0 || sensor >= SENSOR_COUNT) return mConfig.maxSensorRange;
    const Rotation2d r = Rotation2d::fromRadians(pose.theta);
    const Pose& m = mMounts[sensor];
    const Rotation2d& mr = mMountRots[sensor];
    return castRay(pose.x + m.x * r.cos - m.y * r.sin,
                   pose.y + m.x * r.sin + m.y * r.cos,
                   r.cos * mr.cos - r.sin * mr.sin,
                   r.sin * mr.cos + r.cos * mr.sin);
}

bool MclCore::beamDeadzoned(float ox, float oy, float dirCos, float dirSin,
                            float rangeIn) const {
    for (int i = 0; i < mDeadzoneCircleCount; i++) {
        const CircleObstacle& c = mDeadzoneCircles[i];
        const float dx = c.x - ox, dy = c.y - oy;
        const float along = dx * dirCos + dy * dirSin;
        if (along < 0.0f) continue;
        const float perp = std::fabs(-dx * dirSin + dy * dirCos);
        if (perp > c.radius + along * mConfig.obstacleConeFrac) continue;
        const float surface = std::sqrt(dx * dx + dy * dy) - c.radius;
        if (std::fabs(rangeIn - surface) <= mConfig.obstacleRangeTol) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Cloud initialisation
// ---------------------------------------------------------------------------

void MclCore::setPose(const Pose& pose) {
    const bool ok = std::isfinite(pose.x) && std::isfinite(pose.y) &&
                    std::isfinite(pose.theta);
    const Pose seed = ok ? pose : Pose{};

    // acquireVariance, not the roughening and not the search width: one number
    // cannot be all three.
    for (int i = 0; i < mConfig.particleCount; i++) {
        Particle& p = mParticles[i];
        p.pose = {seed.x + mRng.normal() * mConfig.acquireVariance,
                  seed.y + mRng.normal() * mConfig.acquireVariance,
                  wrapRad(seed.theta + mRng.normal() * mConfig.acquireThetaVariance)};
        p.rot = Rotation2d::fromRadians(p.pose.theta);
        p.weight = 1.0f;
        p.logWeight = 0.0f;
    }

    mRawEstimate = {seed.x, seed.y, wrapRad(seed.theta)};
    mLastSpeed = 0.0f;
    mResampledLast = false;
    mResampleCount = 0;
    mHaveLogAvg = false;
    mHaveFit = false;
    mFitSigmas = 0.0f;
    mMismatch = 0.0f;
    mSearchMismatch = 0.0f;
    mInjectFrac = 0.0f;
    mDeadzoneFrac.fill(0.0f);
    mStepEstimate = estimate();
}

void MclCore::uniformReset() {
    const float half = mMap.halfWall;
    for (int i = 0; i < mConfig.particleCount; i++) {
        Particle& p = mParticles[i];
        // Uniform, not Gaussian: a normal draw for heading leaves the cloud
        // clustered at zero and is not a global search at all.
        p.pose = {mRng.uniform(-half, half), mRng.uniform(-half, half),
                  mRng.uniform(-PI, PI)};
        p.rot = Rotation2d::fromRadians(p.pose.theta);
        p.weight = 1.0f;
        p.logWeight = 0.0f;
    }
    mRawEstimate = Pose{};
    mLastSpeed = 0.0f;
    mResampledLast = false;
    mHaveLogAvg = false;
    mHaveFit = false;
    mFitSigmas = 0.0f;
    mMismatch = 0.0f;
    mSearchMismatch = 0.0f;
    mInjectFrac = 0.0f;
}

void MclCore::setParticles(const Particle* particles, int count) {
    if (particles == nullptr) return;
    const int n = std::min(count, mConfig.particleCount);
    for (int i = 0; i < n; i++) {
        mParticles[i] = particles[i];
        mParticles[i].rot = Rotation2d::fromRadians(mParticles[i].pose.theta);
    }
}

int MclCore::snapshotParticles(float* out, int maxParticles) const {
    if (out == nullptr || maxParticles <= 0) return 0;
    const int count = std::min(mConfig.particleCount, maxParticles);
    for (int i = 0; i < count; i++) {
        out[i * 4 + 0] = mParticles[i].pose.x;
        out[i * 4 + 1] = mParticles[i].pose.y;
        out[i * 4 + 2] = mParticles[i].pose.theta;
        out[i * 4 + 3] = mParticles[i].weight;
    }
    return count;
}

// ---------------------------------------------------------------------------
// Motion update
// ---------------------------------------------------------------------------

void MclCore::predict(const MotionInput& in) {
    const OdomDelta& d = in.delta;
    // A dead encoder or IMU reports PROS_ERR_F. Propagating it turns every
    // particle into a NaN, and nothing downstream recovers from that.
    if (!std::isfinite(d.dVert) || !std::isfinite(d.dHoriz) || !std::isfinite(d.dTheta)) {
        return;
    }

    const float dVert = d.dVert + mVertDrift;
    const float dHoriz = d.dHoriz + mHorizDrift;
    const float dTheta = d.dTheta;

    const float dt = (in.dtSeconds > 1e-6f) ? in.dtSeconds
                                            : std::max(mConfig.updatePeriodMs, 1.0f) * 0.001f;
    mLastSpeed = std::sqrt(dVert * dVert + dHoriz * dHoriz) / dt;

    // Exact SE(2) exponential with the twist-only trig hoisted. Pose::exp
    // integrates a body twist as k * R(dTheta/2) * (dx, dy) with
    // k = sin(dTheta/2)/(dTheta/2), so these two are sin(dTheta)/dTheta and
    // (1 - cos dTheta)/dTheta. Straight-line integration of the same twist is
    // what leaves a filter short on the inside of every turn.
    const float h = 0.5f * dTheta;
    const float k = sinc(h);
    const float ch = std::cos(h) * k;
    const float sh = std::sin(h) * k;

    const float driftSigma =
        0.5f * std::sqrt(mVertDrift * mVertDrift + mHorizDrift * mHorizDrift);
    const float vertVar = in.hasVertWheel ? mConfig.trackingWheelVariance
                                          : mConfig.imeVariance;
    // hasImu, not a finiteness test: a faulted IMU holds its last good heading,
    // which is finite, and clamping to a held value pins the whole cloud's
    // heading through a turn the robot really made.
    const bool clampTheta = in.hasImu && std::isfinite(in.imuHeadingRad);

    for (int i = 0; i < mConfig.particleCount; i++) {
        Particle& p = mParticles[i];

        // Noise is drawn in the body frame, where the sensors measure. Drawing
        // it in the world frame would spread the cloud the same way no matter
        // which way the robot was pointing.
        float bx = dVert * (1.0f + mRng.normal() * vertVar);
        if (mConfig.vertConstantNoise > 0.0f) {
            bx += mRng.normal() * mConfig.vertConstantNoise;
        }
        float by;
        if (in.hasHorizWheel) {
            by = dHoriz * (1.0f + mRng.normal() * mConfig.trackingWheelVariance);
        } else {
            by = mRng.normal() * mConfig.horizDependentVarianceProp * std::fabs(dVert) +
                 mRng.normal() * mConfig.horizConstantNoise;
        }
        if (driftSigma > 0.0f) {
            bx += mRng.normal() * driftSigma;
            by += mRng.normal() * driftSigma;
        }

        const float sx = bx * ch - by * sh;
        const float sy = bx * sh + by * ch;
        p.pose.x += sx * p.rot.cos - sy * p.rot.sin;
        p.pose.y += sx * p.rot.sin + sy * p.rot.cos;

        float theta = wrapRad(p.pose.theta + dTheta + mRng.normal() * mConfig.imuVariance);
        if (clampTheta) {
            // The IMU is the only heading source worth trusting on a V5, so a
            // particle is not allowed to argue with it by more than this.
            // Compared as an angular distance: a linear clamp is wrong wherever
            // the two straddle the +/-pi branch cut.
            const float diff =
                clamp(angleDiffRad(in.imuHeadingRad, theta), -mConfig.maxThetaDeviation,
                      mConfig.maxThetaDeviation);
            theta = wrapRad(in.imuHeadingRad + diff);
        }
        p.pose.theta = theta;
        p.rot = Rotation2d::fromRadians(theta);

        if (!std::isfinite(p.pose.x) || !std::isfinite(p.pose.y)) {
            p.pose.x = mRawEstimate.x;
            p.pose.y = mRawEstimate.y;
        }
    }
}

// ---------------------------------------------------------------------------
// Sensor update
// ---------------------------------------------------------------------------

float MclCore::beamSigma(const RawSensor& raw, float rangeIn, bool noReturn,
                         float incidenceRad) const {
    // A sensor reads long at a glancing angle. `incidenceRad` comes from the
    // cast, so a beam running nearly parallel to a barrier is penalised for the
    // 80 degrees it actually hit rather than the 10 it would have hit had the
    // robot been somewhere else.
    const float angleMult =
        1.0f + clamp(incidenceRad, 0.0f, HALF_PI) * (2.0f / PI * 1.2f);

    const bool closeIn = !noReturn && raw.mm >= 0 && raw.mm <= kCloseRangeMm;
    float sigma = closeIn ? mConfig.rangeSigmaCloseIn : rangeIn * mConfig.rangeSigmaProp;
    sigma *= angleMult;
    // A no-return carries no confidence figure, so scaling by one would only
    // inflate its sigma until it said nothing.
    if (!closeIn && !noReturn) {
        sigma *= 32.0f / std::max(static_cast<float>(raw.confidence), 1.0f);
    }
    return std::max(sigma, 1e-3f);
}

int MclCore::buildBeams(const RawSensor* raw, int count, const Pose& reference,
                        Beam* out, bool* valid) const {
    // Clamped like everything else below: there are SENSOR_COUNT slots, so a
    // caller that passes a longer count gets its extra flags left alone rather
    // than the array written past its end.
    const int n = std::min(count, SENSOR_COUNT);
    if (valid != nullptr) {
        for (int i = 0; i < n; i++) valid[i] = false;
    }
    if (raw == nullptr || out == nullptr) return 0;

    bool ok[SENSOR_COUNT] = {};
    bool noReturn[SENSOR_COUNT] = {};
    float rangeIn[SENSOR_COUNT] = {};
    float incidence[SENSOR_COUNT] = {};

    const Rotation2d ref = Rotation2d::fromRadians(reference.theta);

    for (int i = 0; i < n; i++) {
        const RawSensor& r = raw[i];
        if (!r.present || r.suppressed) continue;
        if (r.mm < 0 || r.mm >= kSensorErrorMm) continue;   // PROS_ERR, not a range

        if (r.mm > kNoReturnMm) {
            if (!mConfig.useNoReturnBeams) continue;
            noReturn[i] = true;
            rangeIn[i] = mConfig.maxSensorRange;
        } else {
            if (static_cast<float>(r.mm) < mConfig.minSensorRange) continue;
            if (r.mm > kCloseRangeMm && r.confidence < mConfig.confidenceThreshold) continue;
            rangeIn[i] = static_cast<float>(r.mm) * MM_TO_INCH;
        }
        if (!std::isfinite(rangeIn[i])) continue;

        // Both the blocker test and the incidence angle are judged from the
        // reference pose, because there is nothing else to judge from before
        // weighting; per-particle geometry is the deadzone circles.
        const float sx = reference.x + mMounts[i].x * ref.cos - mMounts[i].y * ref.sin;
        const float sy = reference.y + mMounts[i].x * ref.sin + mMounts[i].y * ref.cos;
        const float dc = ref.cos * mMountRots[i].cos - ref.sin * mMountRots[i].sin;
        const float ds = ref.sin * mMountRots[i].cos + ref.cos * mMountRots[i].sin;

        if (mBlockerCount > 0) {
            bool blocked = false;
            for (int b = 0; b < mBlockerCount; b++) {
                if (intersectLine(sx, sy, dc, ds, mBlockers[b], mConfig.maxSensorRange) <
                    mConfig.maxSensorRange) {
                    blocked = true;
                    break;
                }
            }
            if (blocked) continue;
        }

        castRayIncidence(sx, sy, dc, ds, &incidence[i]);
        ok[i] = true;
    }

    int nb = 0;
    for (int i = 0; i < n; i++) {
        if (!ok[i]) continue;
        const float sigma = beamSigma(raw[i], rangeIn[i], noReturn[i], incidence[i]);
        out[nb].sensor = i;
        out[nb].rangeIn = rangeIn[i];
        out[nb].invSigma = 1.0f / sigma;
        nb++;
        if (valid != nullptr) valid[i] = true;
    }
    return nb;
}

void MclCore::updateWeights(const Beam* beams, int count) {
    const int n = mConfig.particleCount;
    if (beams == nullptr) count = 0;

    // Hoist the per-beam constants out of the particle loop; there are at most
    // SENSOR_COUNT of them and the loop below runs a thousand times.
    int sensor[SENSOR_COUNT] = {};
    float mx[SENSOR_COUNT] = {}, my[SENSOR_COUNT] = {};
    float mc[SENSOR_COUNT] = {}, ms[SENSOR_COUNT] = {};
    float range[SENSOR_COUNT] = {}, invSigma[SENSOR_COUNT] = {};
    float logDensity[SENSOR_COUNT] = {};
    int nb = 0;
    for (int b = 0; b < count && nb < SENSOR_COUNT; b++) {
        const Beam& beam = beams[b];
        if (beam.sensor < 0 || beam.sensor >= SENSOR_COUNT) continue;
        if (!std::isfinite(beam.rangeIn) || !std::isfinite(beam.invSigma)) continue;
        if (beam.invSigma <= 0.0f) continue;
        sensor[nb] = beam.sensor;
        mx[nb] = mMounts[beam.sensor].x;
        my[nb] = mMounts[beam.sensor].y;
        mc[nb] = mMountRots[beam.sensor].cos;
        ms[nb] = mMountRots[beam.sensor].sin;
        range[nb] = beam.rangeIn;
        invSigma[nb] = beam.invSigma;
        // What turns the sigma-space mixture into a density per inch. Constant
        // across particles, so it cancels between them -- but not against
        // deadzoneLikelihood, which is what made the two incomparable.
        logDensity[nb] = std::log(beam.invSigma) - mLogMixtureNorm;
        nb++;
    }
    mScoredBeams = nb;

    const float half = mMap.halfWall;
    // Already a density per inch, so no logDensity term: what makes a blocked
    // beam a real alternative to the map is that its likelihood does NOT shrink
    // as the wall measurement's sigma grows.
    const float logDeadzone = std::log(std::max(mConfig.deadzoneLikelihood, 1e-30f));
    int deadzoneHits[SENSOR_COUNT] = {};

    for (int i = 0; i < n; i++) {
        Particle& p = mParticles[i];

        const float ex = std::fabs(p.pose.x) - half;
        const float ey = std::fabs(p.pose.y) - half;
        if (!(ex <= 0.0f) || !(ey <= 0.0f)) {
            // Strictly worse than any in-field particle, but ranked by how far
            // out it is. A flat floor leaves an escaped cloud with identical
            // weights, so N_eff stays at N, nothing resamples, and the filter
            // never finds its way back inside.
            p.logWeight = LOG_FLOOR - 1.0f - std::max(ex, 0.0f) - std::max(ey, 0.0f);
            continue;
        }

        float logWeight = 0.0f;
        for (int b = 0; b < nb; b++) {
            const float sx = p.pose.x + mx[b] * p.rot.cos - my[b] * p.rot.sin;
            const float sy = p.pose.y + mx[b] * p.rot.sin + my[b] * p.rot.cos;
            const float dc = p.rot.cos * mc[b] - p.rot.sin * ms[b];
            const float ds = p.rot.sin * mc[b] + p.rot.cos * ms[b];

            // Blocked for this hypothesis? Then the reading is of the obstacle
            // and says nothing about where this particle is.
            if (mDeadzoneCircleCount > 0 && beamDeadzoned(sx, sy, dc, ds, range[b])) {
                deadzoneHits[sensor[b]]++;
                logWeight += logDeadzone;
                continue;
            }
            const float expected = castRay(sx, sy, dc, ds);
            // Signed, in sigmas: negative means the return came back shorter
            // than this particle expected, which the mixture treats gently.
            logWeight += logBeamLikelihood((range[b] - expected) * invSigma[b]) +
                         logDensity[b];
        }
        p.logWeight = std::max(logWeight, LOG_FLOOR);
    }

    // Exponentiate relative to the best particle: the offset cancels in every
    // downstream ratio (resampling, the weighted mean, N_eff) but keeps the
    // whole cloud inside float range.
    float maxLog = mParticles[0].logWeight;
    for (int i = 1; i < n; i++) maxLog = std::max(maxLog, mParticles[i].logWeight);
    if (!std::isfinite(maxLog)) maxLog = 0.0f;

    float sumRel = 0.0f;
    for (int i = 0; i < n; i++) {
        const float w = std::exp(mParticles[i].logWeight - maxLog);
        mParticles[i].weight = std::isfinite(w) ? w : 0.0f;
        sumRel += mParticles[i].weight;
    }
    if (!(sumRel > 0.0f)) {
        // Unreachable after the shift above, since the best particle
        // exponentiates to exactly 1. Cheap insurance against a NaN pose.
        for (int i = 0; i < n; i++) mParticles[i].weight = 1.0f;
        sumRel = static_cast<float>(n);
    }

    // Augmented MCL, in logs. The arithmetic slow/fast averages of the textbook
    // become geometric ones here, which is what keeps the ratio meaningful when
    // likelihoods span many orders of magnitude.
    if (nb > 0) {
        const float invNb = 1.0f / static_cast<float>(nb);
        // Per beam, so a beam dropping in or out of range does not shift the
        // average by its own contribution and read as a displacement.
        const float logWAvg =
            (maxLog + std::log(sumRel / static_cast<float>(n))) * invNb;
        if (!mHaveLogAvg) {
            mLogWSlow = mLogWFast = logWAvg;
            mHaveLogAvg = true;
        } else {
            mLogWSlow += mConfig.alphaSlow * (logWAvg - mLogWSlow);
            mLogWFast += mConfig.alphaFast * (logWAvg - mLogWFast);
        }
        mMismatch =
            clamp(1.0f - std::exp(std::min(mLogWFast - mLogWSlow, 0.0f)), 0.0f, 1.0f);
        mInjectFrac = std::min(mMismatch, std::max(mConfig.maxRandomFrac, 0.0f));
    } else {
        mMismatch = 0.0f;
        mInjectFrac = 0.0f;
    }

    const float inv = 1.0f / static_cast<float>(n);
    for (int i = 0; i < SENSOR_COUNT; i++) {
        mDeadzoneFrac[i] = static_cast<float>(deadzoneHits[i]) * inv;
    }
}

// ---------------------------------------------------------------------------
// Resampling
// ---------------------------------------------------------------------------

void MclCore::systematicResample(const float* weights, int count, float u01, int* out,
                                 int outCount) {
    if (weights == nullptr || out == nullptr || count <= 0 || outCount <= 0) return;

    float total = 0.0f;
    for (int i = 0; i < count; i++) {
        if (weights[i] > 0.0f && std::isfinite(weights[i])) total += weights[i];
    }
    if (!(total > 0.0f)) {
        // No preference expressed. Spread the draw so the caller gets its cloud
        // back rather than outCount copies of particle zero.
        for (int m = 0; m < outCount; m++) {
            out[m] = static_cast<int>(static_cast<long long>(m) * count / outCount);
        }
        return;
    }

    const float step = total / static_cast<float>(outCount);
    float target = clamp(u01, 0.0f, 1.0f) * step;
    float cum = (weights[0] > 0.0f && std::isfinite(weights[0])) ? weights[0] : 0.0f;
    int index = 0;

    for (int m = 0; m < outCount; m++) {
        // >= not >, so a leading zero weight is stepped over even when the
        // single uniform draw lands exactly on zero.
        while (target >= cum && index < count - 1) {
            index++;
            if (weights[index] > 0.0f && std::isfinite(weights[index])) cum += weights[index];
        }
        out[m] = index;
        target += step;
    }
}

float MclCore::effectiveSampleSize(const float* weights, int count) {
    if (weights == nullptr || count <= 0) return 0.0f;
    float sum = 0.0f, sumSq = 0.0f;
    for (int i = 0; i < count; i++) {
        const float w = (std::isfinite(weights[i]) && weights[i] > 0.0f) ? weights[i] : 0.0f;
        sum += w;
        sumSq += w * w;
    }
    if (!(sumSq > 0.0f)) return 0.0f;
    return sum * sum / sumSq;
}

float MclCore::circularMean(const float* angles, const float* weights, int count) {
    if (angles == nullptr || count <= 0) return 0.0f;
    float s = 0.0f, c = 0.0f, wSum = 0.0f;
    for (int i = 0; i < count; i++) {
        const float w = (weights != nullptr) ? weights[i] : 1.0f;
        if (!std::isfinite(w) || w <= 0.0f || !std::isfinite(angles[i])) continue;
        s += std::sin(angles[i]) * w;
        c += std::cos(angles[i]) * w;
        wSum += w;
    }
    // Antipodal cancellation leaves no mean direction, and the atan2 of what
    // survives it is float rounding: sinf(PI) is -8.7e-8, not 0, and answers
    // -pi/2. Tested against the resultant's length relative to the total weight,
    // because that is the quantity that vanishes, not either component alone.
    if (!(std::sqrt(s * s + c * c) > wSum * 1e-6f)) return 0.0f;
    return std::atan2(s, c);
}

void MclCore::updateFit(const Pose& pose, const Beam* beams, int count) {
    if (beams == nullptr || count <= 0) return;

    const Rotation2d rot = Rotation2d::fromRadians(pose.theta);
    const float lost = mConfig.lostFitSigmas;
    // Per-beam residual cap. Uncapped, a blocker across a sensor is 800 sigmas --
    // a close reading's sigma is 0.068 in -- and one tick of that left the cloud
    // searching for half a second. Capping at lostFitSigmas itself is the
    // opposite mistake: the rise goes so slow that kidnap recovery falls from 18
    // of 19 to 15.
    //
    // What it buys, exactly: with f1 = f0(1-a) + a*C and the ramp floor at
    // T = lost*floor, one tick crosses T only from f0 > 0.2*T/(1-a) -- 1.33
    // sigmas at the shipped 0.8/12/0.5/0.10. So a single outlier tick cannot scatter
    // a filter that is tracking, which is what it is for, but it CAN from a
    // filter already carrying ~1.5 sigmas of real residual. That is not fixable
    // by bounding the per-tick rise instead: bounding it by the distance left to
    // T makes the level asymptote below T and the ramp never fires at all.
    const float cap = (lost > 0.0f)
        ? std::max(mConfig.lostFitCapProp, 0.0f) * lost *
              std::max(mConfig.searchMismatchFloor, 0.0f) /
              std::max(mConfig.alphaFast, 1e-3f)
        : 0.0f;
    float sq[SENSOR_COUNT] = {};
    int used = 0;
    // Clamped like updateWeights and buildBeams: `count` is caller-supplied, and
    // several beams can share one sensor slot, so this is not bounded by
    // SENSOR_COUNT on its own.
    for (int b = 0; b < count && used < SENSOR_COUNT; b++) {
        const Beam& beam = beams[b];
        if (beam.sensor < 0 || beam.sensor >= SENSOR_COUNT) continue;
        if (!std::isfinite(beam.rangeIn) || !(beam.invSigma > 0.0f)) continue;
        const Pose& m = mMounts[beam.sensor];
        const Rotation2d& mr = mMountRots[beam.sensor];
        const float sx = pose.x + m.x * rot.cos - m.y * rot.sin;
        const float sy = pose.y + m.x * rot.sin + m.y * rot.cos;
        const float dc = rot.cos * mr.cos - rot.sin * mr.sin;
        const float ds = rot.sin * mr.cos + rot.cos * mr.sin;
        // A beam an obstacle owns says nothing about the estimate, so counting
        // its residual would read a mapped post as a lost filter.
        if (mDeadzoneCircleCount > 0 && beamDeadzoned(sx, sy, dc, ds, beam.rangeIn)) {
            continue;
        }
        float r = (beam.rangeIn - castRay(sx, sy, dc, ds)) * beam.invSigma;
        if (!std::isfinite(r)) continue;
        if (cap > 0.0f) r = clamp(r, -cap, cap);
        sq[used] = r * r;
        used++;
    }
    if (used == 0) return;   // no evidence this tick; hold the last reading

    // MEDIAN, not mean. One beam reading something the map does not have -- a
    // post whose real position is off by more than obstacleRangeTol, a robot
    // parked in front of a sensor -- is tens of sigmas, and a mean of four lets
    // it declare a healthy filter lost and scatter the cloud to the search
    // width. A displacement moves every beam, so the median still sees a kidnap.
    // Insertion sort over at most SENSOR_COUNT entries; std::nth_element on 8
    // floats is not cheaper.
    for (int i = 1; i < used; i++) {
        const float v = sq[i];
        int j = i - 1;
        while (j >= 0 && sq[j] > v) {
            sq[j + 1] = sq[j];
            j--;
        }
        sq[j + 1] = v;
    }
    const float med = (used % 2 == 1) ? sq[used / 2]
                                      : 0.5f * (sq[used / 2 - 1] + sq[used / 2]);
    const float rms = std::sqrt(med);
    if (!mHaveFit) {
        mFitSigmas = rms;
        mHaveFit = true;
    } else {
        // One smoother, not a fast/slow ratio: this is a level, so a cloud that
        // has been wrong for a long time still reads wrong.
        mFitSigmas += mConfig.alphaFast * (rms - mFitSigmas);
    }
    mSearchMismatch = (lost > 0.0f) ? clamp(mFitSigmas / lost, 0.0f, 1.0f) : 0.0f;
}

void MclCore::roughening(float* distSigma, float* thetaSigma) const {
    const float floor = std::max(mConfig.searchMismatchFloor, 0.0f);
    const float span = std::max(mConfig.searchMismatchFull - floor, 1e-3f);
    const float blend = clamp((mSearchMismatch - floor) / span, 0.0f, 1.0f);
    *distSigma = mConfig.distResampleVariance +
                 blend * std::max(mConfig.distSearchVariance -
                                      mConfig.distResampleVariance, 0.0f);
    *thetaSigma = mConfig.thetaResampleVariance +
                  blend * std::max(mConfig.thetaSearchVariance -
                                       mConfig.thetaResampleVariance, 0.0f);
}

void MclCore::resample(float imuHeadingRad, bool hasImu) {
    const int n = mConfig.particleCount;

    float total = 0.0f;
    for (int i = 0; i < n; i++) {
        const float w = mParticles[i].weight;
        mWeightScratch[i] = (std::isfinite(w) && w > 0.0f) ? w : 0.0f;
        total += mWeightScratch[i];
    }
    if (!(total > 0.0f)) {
        // Every weight underflowed, so there is nothing to select on. Flatten
        // and keep the cloud: scattering it across the field here would throw
        // away a filter that may have been only slightly wrong.
        for (int i = 0; i < n; i++) {
            mParticles[i].weight = 1.0f;
            mParticles[i].logWeight = 0.0f;
        }
        mResampledLast = false;
        return;
    }

    // Replace the tail of the draw with fresh field-wide samples when the
    // likelihood says the cloud is in the wrong place. Their heading comes from
    // the IMU rather than being random: on a V5 the heading is the one thing
    // never in doubt, and fixing it turns a 3-D search into a 2-D one.
    // Only above the floor. Injecting a trickle on every resample is
    // self-sustaining: the fresh particles carry no weight, so N_eff drops,
    // which triggers the next resample, which injects again.
    const float frac = (mInjectFrac >= mConfig.injectResampleFloor) ? mInjectFrac : 0.0f;
    const int injected = clamp(static_cast<int>(frac * static_cast<float>(n)), 0, n - 1);
    const int drawn = n - injected;

    systematicResample(mWeightScratch, n, mRng.uniform(), mIndexScratch, drawn);

    // Tight while the cloud is tracking, widening to the search width once the
    // measurements stop fitting. One fixed width cannot be both: wide enough to
    // recover from a displacement is wide enough to stop the cloud contracting.
    float distSigma = 0.0f, thetaSigma = 0.0f;
    roughening(&distSigma, &thetaSigma);

    for (int m = 0; m < drawn; m++) {
        Particle p = mParticles[mIndexScratch[m]];
        p.pose.x += mRng.normal() * distSigma;
        p.pose.y += mRng.normal() * distSigma;
        p.pose.theta = wrapRad(p.pose.theta + mRng.normal() * thetaSigma);
        p.rot = Rotation2d::fromRadians(p.pose.theta);
        p.weight = 1.0f;
        p.logWeight = 0.0f;
        mScratch[m] = p;
    }

    if (injected > 0) {
        const float half = mMap.halfWall;
        const float heading = (hasImu && std::isfinite(imuHeadingRad))
                                  ? imuHeadingRad
                                  : mRawEstimate.theta;
        for (int m = drawn; m < n; m++) {
            Particle& p = mScratch[m];
            p.pose = {mRng.uniform(-half, half), mRng.uniform(-half, half),
                      wrapRad(heading + mRng.normal() * mConfig.thetaSearchVariance)};
            p.rot = Rotation2d::fromRadians(p.pose.theta);
            p.weight = 1.0f;
            p.logWeight = 0.0f;
        }
    }

    std::swap(mParticles, mScratch);
    mResampledLast = true;
    mResampleCount++;
}

// ---------------------------------------------------------------------------
// Estimate
// ---------------------------------------------------------------------------

MclCore::Estimate MclCore::estimate() const {
    Estimate out;
    out.pose = mRawEstimate;

    const int n = mConfig.particleCount;
    float sx = 0.0f, sy = 0.0f, ss = 0.0f, sc = 0.0f;
    float total = 0.0f, sumSq = 0.0f;

    for (int i = 0; i < n; i++) {
        const Particle& p = mParticles[i];
        const float w = (std::isfinite(p.weight) && p.weight > 0.0f) ? p.weight : 0.0f;
        if (w == 0.0f) continue;
        if (!std::isfinite(p.pose.x) || !std::isfinite(p.pose.y)) continue;
        sx += p.pose.x * w;
        sy += p.pose.y * w;
        ss += p.rot.sin * w;
        sc += p.rot.cos * w;
        total += w;
        sumSq += w * w;
    }
    if (!(total > 0.0f) || !(sumSq > 0.0f)) return out;

    const float inv = 1.0f / total;
    // Circular mean for heading: averaging radians arithmetically is wrong
    // across the +/-pi branch cut, and a cloud straddling it would report zero.
    // Cancellation is judged on the resultant's length, since either component
    // alone can be near zero for a perfectly ordinary heading.
    const float theta = !(std::sqrt(ss * ss + sc * sc) > total * 1e-6f)
                            ? mRawEstimate.theta
                            : std::atan2(ss, sc);
    const Pose mean{sx * inv, sy * inv, theta};
    if (!std::isfinite(mean.x) || !std::isfinite(mean.y) || !std::isfinite(mean.theta)) {
        return out;
    }
    out.pose = mean;
    out.nEff = total * total / sumSq;

    // Weighted spread about that mean. This is the cloud's opinion of its own
    // error, which understates the truth whenever the cloud has converged on
    // the wrong place, so consumers should inflate it.
    float cxx = 0.0f, cxy = 0.0f, cyy = 0.0f, ctt = 0.0f;
    for (int i = 0; i < n; i++) {
        const Particle& p = mParticles[i];
        const float w = (std::isfinite(p.weight) && p.weight > 0.0f) ? p.weight : 0.0f;
        if (w == 0.0f) continue;
        if (!std::isfinite(p.pose.x) || !std::isfinite(p.pose.y)) continue;
        const float dx = p.pose.x - mean.x;
        const float dy = p.pose.y - mean.y;
        const float dt = angleDiffRad(mean.theta, p.pose.theta);
        cxx += w * dx * dx;
        cxy += w * dx * dy;
        cyy += w * dy * dy;
        ctt += w * dt * dt;
    }
    out.covXX = cxx * inv;
    out.covXY = cxy * inv;
    out.covYY = cyy * inv;
    out.covTheta = ctt * inv;
    return out;
}

// ---------------------------------------------------------------------------
// One iteration
// ---------------------------------------------------------------------------

Pose MclCore::step(const MotionInput& in, const Beam* beams, int beamCount) {
    predict(in);
    updateWeights(beams, beamCount);

    const Estimate est = estimate();
    mStepEstimate = est;
    updateFit(est.pose, beams, beamCount);
    mResampledLast = false;

    // Probabilistic Robotics table 8.2 resamples on every iteration, and that is
    // what turns the weight a tick earned into support the next tick inherits.
    // logWeight is overwritten rather than accumulated here, so an ESS gate can
    // never fire on its own: a single tick's weights over a converged cloud are
    // near-flat, ESS stays around 0.7 N, nothing resamples, and the filter
    // re-estimates from one tick forever instead of accumulating evidence.
    // The gate is kept for the case it was really guarding -- no measurements at
    // all, where resampling only adds a random walk.
    bool doResample = false;
    if (mScoredBeams > 0) {
        doResample = true;
    } else if (est.nEff < static_cast<float>(mConfig.resampleThreshold) * 0.5f) {
        // Part of the cloud has left the field, which the out-of-field ranking
        // shows up as a collapsed ESS even with nothing measured.
        doResample = true;
    }

    if (doResample) resample(in.imuHeadingRad, in.hasImu);

    mRawEstimate = est.pose;
    return est.pose;
}

}  // namespace chad
