#include "chad/tracking/ekf_core.hpp"

#include <algorithm>
#include <cmath>

namespace chad {

namespace {

constexpr float kWall = Field::FIELD_HALF_WALL;
constexpr float kVarFloor = 1e-9f;

void mat3Mul(const float* A, const float* B, float* out) {
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            out[r * 3 + c] = A[r * 3 + 0] * B[0 * 3 + c]
                           + A[r * 3 + 1] * B[1 * 3 + c]
                           + A[r * 3 + 2] * B[2 * 3 + c];
        }
    }
}

void mat3MulT(const float* A, const float* B, float* out) {
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            out[r * 3 + c] = A[r * 3 + 0] * B[c * 3 + 0]
                           + A[r * 3 + 1] * B[c * 3 + 1]
                           + A[r * 3 + 2] * B[c * 3 + 2];
        }
    }
}

/// Row-vector times matrix: out = v * M.
void rowTimesMat3(const float v[3], const float* M, float out[3]) {
    for (int c = 0; c < 3; c++) {
        out[c] = v[0] * M[0 * 3 + c] + v[1] * M[1 * 3 + c] + v[2] * M[2 * 3 + c];
    }
}

}  // namespace

// ============================================================================
// Covariance hygiene
// ============================================================================

// Joseph form is symmetric and positive-semidefinite in exact arithmetic. In
// float32 it is neither, by an amount that compounds: the asymmetry feeds the
// next update, and a variance that drifts negative makes S negative, which
// flips the sign of the gain and diverges the filter within a few cycles.
// Averaging the halves and floor-clamping costs nothing and makes the invariant
// hold by construction rather than by hope.
bool EkfCore::symmetrize(float* P) {
    bool repaired = false;
    for (int r = 0; r < 3; r++) {
        for (int c = r + 1; c < 3; c++) {
            const float m = 0.5f * (P[r * 3 + c] + P[c * 3 + r]);
            P[r * 3 + c] = P[c * 3 + r] = m;
        }
    }
    for (int i = 0; i < 3; i++) {
        float& d = P[i * 3 + i];
        if (!std::isfinite(d) || d < kVarFloor) {
            d = kVarFloor;
            repaired = true;
        }
    }
    // A correlation outside [-1, 1] is an indefinite P even with a positive
    // diagonal, and no Cholesky of it exists. Clamping only fires when float
    // has already lost the invariant, so it doubles as the health signal.
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

// ============================================================================
// Setup
// ============================================================================

void EkfCore::setObstacles(const RangeObstacle* obstacles, int count) {
    mObstacleCount = 0;
    if (obstacles == nullptr || count <= 0) return;
    const int n = std::min(count, MAX_OBSTACLES);
    for (int i = 0; i < n; i++) mObstacles[static_cast<std::size_t>(i)] = obstacles[i];
    mObstacleCount = n;
}

void EkfCore::reset(const Pose& pose, float posSigma, float thetaSigma,
                    std::uint32_t nowMs) {
    mState[0] = pose.x;
    mState[1] = pose.y;
    mState[2] = wrapRad(pose.theta);

    for (int i = 0; i < 9; i++) mP[i] = 0.0f;
    mP[0] = posSigma * posSigma;
    mP[4] = posSigma * posSigma;
    mP[8] = thetaSigma * thetaSigma;
    symmetrize(mP);

    mOdomTrail = pose;
    mTrail.clear();
    mTrail.add(nowMs, mOdomTrail);

    mBeamStates.fill(BeamState::Absent);
    mInnovations.fill(0.0f);
    mBeamsUsed = 0;
    mRangeRejectStreak = 0;
    mRangeBlindStreak = 0;
    mMclRejectStreak = 0;
    mRelocated = false;
}

EkfCore::Uncertainty EkfCore::uncertainty() const {
    return {std::sqrt(std::max(mP[0], 0.0f)),
            std::sqrt(std::max(mP[4], 0.0f)),
            std::sqrt(std::max(mP[8], 0.0f))};
}

bool EkfCore::consumeRelocation() {
    const bool was = mRelocated;
    mRelocated = false;
    return was;
}

// ============================================================================
// Models
// ============================================================================

Pose EkfCore::propagate(const Pose& pose, const OdomDelta& delta) {
    return pose.exp({delta.dVert, delta.dHoriz, delta.dTheta});
}

void EkfCore::transformJacobian(const Pose& pose, const Transform2d& t, float J[9]) {
    const Pose out = pose.transformBy(t);
    // Only the heading column is non-trivial: turning the robot swings the
    // rigid offset the transform adds.
    J[0] = 1.0f; J[1] = 0.0f; J[2] = -(out.y - pose.y);
    J[3] = 0.0f; J[4] = 1.0f; J[5] =  (out.x - pose.x);
    J[6] = 0.0f; J[7] = 0.0f; J[8] = 1.0f;
}

void EkfCore::processJacobian(const Pose& pose, const OdomDelta& delta, float F[9]) {
    const Pose out = propagate(pose, delta);
    F[0] = 1.0f; F[1] = 0.0f; F[2] = -(out.y - pose.y);
    F[3] = 0.0f; F[4] = 1.0f; F[5] =  (out.x - pose.x);
    F[6] = 0.0f; F[7] = 0.0f; F[8] = 1.0f;
}

// ============================================================================
// predict() — odometry drives the state, and inflates P
// ============================================================================

void EkfCore::predict(const OdomDelta& delta, bool hasHorizontalTracking,
                      std::uint32_t nowMs, bool headingValid) {
    const OdomDelta d{std::isfinite(delta.dVert) ? delta.dVert : 0.0f,
                      std::isfinite(delta.dHoriz) ? delta.dHoriz : 0.0f,
                      std::isfinite(delta.dTheta) ? delta.dTheta : 0.0f};

    mOdomTrail = propagate(mOdomTrail, d);
    mTrail.add(nowMs, mOdomTrail);

    const Pose prior{mState[0], mState[1], mState[2]};
    const Pose integrated = propagate(prior, d);

    float F[9];
    processJacobian(prior, d, F);

    mState[0] = integrated.x;
    mState[1] = integrated.y;
    mState[2] = integrated.theta;

    float FP[9], FPFt[9];
    mat3Mul(F, mP, FP);
    mat3MulT(FP, F, FPFt);

    // Odometry error accumulates with distance travelled, so noise is per-inch.
    // It is also anisotropic: a tank drive knows how far it drove far better
    // than whether it slid sideways, so build Q in the body frame and rotate it.
    const float alongSigma = mConfig.vertNoiseProp * std::fabs(d.dVert)
                           + mConfig.posNoiseFloor;
    const float lateralSigma =
        hasHorizontalTracking
            ? mConfig.horizNoiseProp * std::fabs(d.dHoriz) + mConfig.posNoiseFloor
            : mConfig.slipNoiseProp * std::fabs(d.dVert) + mConfig.slipNoiseFloor;

    // Systematic component. Charging bias^2 * |ds| * decorrLen per tick makes the
    // accumulated variance bias^2 * L * decorrLen, so sigma reaches the real
    // error at L == decorrLen and the bias renews from there.
    const float travel = std::fabs(d.dVert) + std::fabs(d.dHoriz);
    const float biasScale = travel * mConfig.biasDecorrIn;
    const float qBiasAlong = mConfig.vertBiasProp * mConfig.vertBiasProp * biasScale;
    const float qBiasLat = hasHorizontalTracking
        ? mConfig.horizBiasProp * mConfig.horizBiasProp * biasScale
        : mConfig.slipBiasProp * mConfig.slipBiasProp * biasScale;

    // The body noise acted over the whole tick, so it is rotated through the
    // tick's midpoint heading. The MEAN above uses the exact pose exponential;
    // these are different jobs and the midpoint is right for this one.
    const float mid = prior.theta + 0.5f * d.dTheta;
    const float c = std::cos(mid), s = std::sin(mid);

    const float qa = alongSigma * alongSigma + qBiasAlong;
    const float ql = lateralSigma * lateralSigma + qBiasLat;
    const float qxx = qa * c * c + ql * s * s;
    const float qyy = qa * s * s + ql * c * c;
    const float qxy = (qa - ql) * c * s;

    const float thetaSigma = mConfig.thetaNoiseProp * std::fabs(d.dTheta)
                           + mConfig.thetaNoisePerInch * std::fabs(d.dVert)
                           + mConfig.thetaNoiseFloor;
    float qTheta = thetaSigma * thetaSigma;

    // A held heading reports dTheta == 0 whatever the robot did, so the terms
    // above -- all proportional to motion the filter can see -- charge the floor
    // through a turn the filter is missing entirely. Charge the real growth rate
    // instead, so the covariance says it does not know and the gate widens
    // rather than rejecting every correct beam. Clocked off updatePeriodMs
    // rather than nowMs: a repeated or backwards timestamp must not zero it.
    if (!headingValid) {
        const float dt = 0.001f * std::max(mConfig.updatePeriodMs, 0.0f);
        qTheta += std::max(mConfig.thetaUnmeasuredVarPerSec, 0.0f) * dt;
    }

    for (int i = 0; i < 9; i++) mP[i] = FPFt[i];
    mP[0] += qxx;
    mP[1] += qxy;
    mP[3] += qxy;
    mP[4] += qyy;
    mP[8] += qTheta;
    if (symmetrize(mP)) mCovRepairs++;
}

// ============================================================================
// Scalar correction (Joseph form)
// ============================================================================

bool EkfCore::fuseScalar(const float H[3], float innovation, float R, float gateChi2) {
    if (!std::isfinite(innovation) || !std::isfinite(R) || R < 0.0f) return false;
    for (int i = 0; i < 3; i++) {
        if (!std::isfinite(H[i])) return false;
    }

    float PHt[3];
    for (int r = 0; r < 3; r++) {
        PHt[r] = mP[r * 3 + 0] * H[0] + mP[r * 3 + 1] * H[1] + mP[r * 3 + 2] * H[2];
    }
    const float S = H[0] * PHt[0] + H[1] * PHt[1] + H[2] * PHt[2] + R;
    if (!(S > 1e-12f)) return false;

    // Normalized innovation squared against a chi-square threshold. Without it
    // a beam that ranged a robot instead of a wall teleports the estimate.
    if (gateChi2 > 0.0f && (innovation * innovation) / S > gateChi2) return false;

    const float invS = 1.0f / S;
    const float K[3] = {PHt[0] * invS, PHt[1] * invS, PHt[2] * invS};
    for (int i = 0; i < 3; i++) {
        if (!std::isfinite(K[i])) return false;
    }

    mState[0] += K[0] * innovation;
    mState[1] += K[1] * innovation;
    mState[2] = wrapRad(mState[2] + K[2] * innovation);

    // Joseph form: P = (I-KH) P (I-KH)^T + K R K^T. The short form (I-KH)P is
    // cheaper but is not symmetric by construction and goes indefinite in float.
    float A[9];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            A[r * 3 + c] = (r == c ? 1.0f : 0.0f) - K[r] * H[c];
        }
    }
    float AP[9], APAt[9];
    mat3Mul(A, mP, AP);
    mat3MulT(AP, A, APAt);
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            mP[r * 3 + c] = APAt[r * 3 + c] + K[r] * R * K[c];
        }
    }
    if (symmetrize(mP)) mCovRepairs++;
    return true;
}

// ============================================================================
// Ray casting
// ============================================================================

EkfCore::WallHit EkfCore::castWalls(float sx, float sy,
                                    float rayCos, float raySin) const {
    // The field interior is a convex axis-aligned box, so a slab test gives both
    // the exit distance and the face -- and the face normal is what the range
    // Jacobian needs.
    WallHit hit;
    float best = mConfig.maxRangeIn;

    const float eps = 1e-4f;
    if (std::fabs(rayCos) > eps) {
        const float bound = (rayCos > 0.0f) ? kWall : -kWall;
        const float t = (bound - sx) / rayCos;
        if (t > 0.0f && t < best) {
            const float yAt = sy + raySin * t;
            if (std::fabs(yAt) <= kWall + 1.0f) {
                best = t;
                hit.valid = true;
                hit.nx = (rayCos > 0.0f) ? 1.0f : -1.0f;
                hit.ny = 0.0f;
                hit.cosIncidence = std::fabs(rayCos);
            }
        }
    }
    if (std::fabs(raySin) > eps) {
        const float bound = (raySin > 0.0f) ? kWall : -kWall;
        const float t = (bound - sy) / raySin;
        if (t > 0.0f && t < best) {
            const float xAt = sx + rayCos * t;
            if (std::fabs(xAt) <= kWall + 1.0f) {
                best = t;
                hit.valid = true;
                hit.nx = 0.0f;
                hit.ny = (raySin > 0.0f) ? 1.0f : -1.0f;
                hit.cosIncidence = std::fabs(raySin);
            }
        }
    }

    hit.range = best;
    return hit;
}

EkfCore::RangePrediction EkfCore::predictRange(const Pose& pose,
                                               const Pose& mount) const {
    RangePrediction out;
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) {
        return out;
    }

    const float ct = std::cos(pose.theta), st = std::sin(pose.theta);
    out.sx = pose.x + mount.x * ct - mount.y * st;
    out.sy = pose.y + mount.x * st + mount.y * ct;
    out.rayCos = std::cos(pose.theta + mount.theta);
    out.raySin = std::sin(pose.theta + mount.theta);

    const WallHit hit = castWalls(out.sx, out.sy, out.rayCos, out.raySin);
    if (!hit.valid) return out;

    out.range = hit.range;
    out.cosIncidence = hit.cosIncidence;
    out.nx = hit.nx;
    out.ny = hit.ny;
    if (hit.cosIncidence < std::cos(mConfig.maxIncidenceRad)) {
        out.state = BeamState::Grazing;
        return out;
    }

    // r = (d - n.s) / (n.u), so with D = n.u:
    //   dr/dx = -n_x / D,  dr/dy = -n_y / D
    //   dr/dtheta = -(n . ds/dtheta)/D - r (n . du/dtheta)/D
    const float D = hit.nx * out.rayCos + hit.ny * out.raySin;
    if (std::fabs(D) < 1e-4f) {
        out.state = BeamState::Grazing;
        return out;
    }
    const float invD = 1.0f / D;
    // ds/dtheta rotates the mount offset; du/dtheta is the ray turned 90 deg.
    const float dsx = -mount.x * st - mount.y * ct;
    const float dsy =  mount.x * ct - mount.y * st;
    const float dux = -out.raySin;
    const float duy =  out.rayCos;

    out.H[0] = -hit.nx * invD;
    out.H[1] = -hit.ny * invD;
    out.H[2] = (-(hit.nx * dsx + hit.ny * dsy)
                - hit.range * (hit.nx * dux + hit.ny * duy)) * invD;
    out.state = BeamState::Used;
    return out;
}

bool EkfCore::obstacleOwnsReading(float sx, float sy, float rayCos, float raySin,
                                  float readingIn, float wallRange) const {
    if (mObstacleCount == 0) return false;

    // Widened by the filter's own position sigma: an unsure filter is also
    // unsure which beams a post could be spoiling, and should drop more. All
    // three tests below are monotone in `pad` -- wider cone, longer reach past
    // the wall, looser range match -- and it is capped so a lost filter cannot
    // deadzone every beam and lock itself out of recovering. The cone gets
    // `pad` and not `tol` because obstacleRangeTol is slop on a range.
    const float posSigma = std::sqrt(std::max(mP[0], 0.0f) + std::max(mP[4], 0.0f));
    const float pad = std::min(mConfig.obstacleSigmaPad * posSigma,
                               std::max(mConfig.obstacleTolMaxIn
                                            - mConfig.obstacleRangeTol, 0.0f));
    const float tol = mConfig.obstacleRangeTol + pad;

    for (int i = 0; i < mObstacleCount; i++) {
        const RangeObstacle& circle = mObstacles[static_cast<std::size_t>(i)];
        const float ox = circle.x - sx;
        const float oy = circle.y - sy;
        const float along = ox * rayCos + oy * raySin;
        if (along < 0.0f) continue;
        const float perp = std::fabs(-ox * raySin + oy * rayCos);
        // Safe to widen only because the range match below still has to agree.
        if (perp > circle.radius + along * mConfig.obstacleConeFrac + pad) continue;

        const float postRange = std::sqrt(ox * ox + oy * oy) - circle.radius;
        // On top of the sensor: nothing this beam reads is a wall.
        if (postRange <= 0.0f) return true;
        // Behind the wall this ray would hit, so it cannot have produced the
        // reading -- unless the filter is unsure enough that it might be in
        // front after all.
        if (postRange >= wallRange + tol) continue;
        // Only a return consistent with the post surface is the post's. One that
        // matches the wall says the cone test was the over-estimate it is, and
        // belongs to the Mahalanobis gate; dropping those unconditionally
        // blinded an 18 inch strip of wall per post.
        if (std::fabs(readingIn - postRange) <= tol) return true;
    }
    return false;
}

// ============================================================================
// Latency rewind
// ============================================================================

Transform2d EkfCore::rewind(float latencyMs, std::uint32_t nowMs) const {
    if (!(latencyMs > 0.0f) || !std::isfinite(latencyMs)) return {};
    const std::uint32_t stamp = nowMs - static_cast<std::uint32_t>(latencyMs);
    if (!mTrail.covers(stamp)) return {};
    Pose then{};
    if (!mTrail.sample(stamp, then)) return {};
    return mOdomTrail.transformTo(then);
}

// ============================================================================
// Measurement families
// ============================================================================

void EkfCore::updateHeading(float headingRad, std::uint32_t nowMs) {
    if (!std::isfinite(headingRad)) return;

    const Pose now{mState[0], mState[1], mState[2]};
    const Transform2d back = rewind(mConfig.imuLatencyMs, nowMs);
    const Pose evalPose = now.transformBy(back);

    // Heading is measured directly, so H is the heading row of the rewind
    // Jacobian, which is [0 0 1] whatever the rewind was.
    const float H[3] = {0.0f, 0.0f, 1.0f};
    // Wrapped BEFORE the gain multiplies it: unwrapped, a heading that crosses
    // the branch cut hands the filter a 2*pi correction.
    const float innovation = angleDiffRad(evalPose.theta, headingRad);
    if (!fuseScalar(H, innovation, mConfig.imuSigmaRad * mConfig.imuSigmaRad,
                    mConfig.imuGateChi2)) {
        mHeadingRejects++;
    }
}

bool EkfCore::beamReadingIn(const Beam& beam, float* readingIn) const {
    if (!beam.present) return false;
    const float mm = beam.mm;
    // get_confidence() is a raw port read that ends up in a divisor: at
    // PROS_ERR it collapsed the range variance to 2e-8 square inches and
    // snapped the state onto one reading. Outside the hardware's range it is
    // not a confidence at all but a faulted read, and a sensor that cannot
    // say how sure it is has not given a trustworthy range either -- so it
    // is treated as no confidence, which minConfidence then drops. This is
    // the same treatment sentinelMm gives a faulted get().
    const int raw = beam.confidence;
    const int conf = (raw < 0 || raw > CONFIDENCE_MAX) ? 0 : raw;
    if (!std::isfinite(mm) || mm >= mConfig.sentinelMm || mm < mConfig.minRangeMm ||
        (mm > 200.0f && conf < mConfig.minConfidence)) {
        return false;
    }
    *readingIn = mm * MM_TO_INCH;
    return true;
}

float EkfCore::rangeSigma(const Beam& beam, float readingIn,
                          float cosIncidence) const {
    // Range noise grows with distance, and a glancing beam maps a given range
    // error into a larger position error, so divide by cos(incidence).
    float sigma = (mConfig.rangeSigmaBase + mConfig.rangeSigmaProp * readingIn)
                / std::max(cosIncidence, 0.25f);
    // Confidence INFLATES sigma and never shrinks it. rangeSigmaBase is
    // bench-measured, so a sensor merely reporting itself happy does not
    // entitle the filter to half of it -- at conf 63 that claimed 0.86 mm of
    // V5 time-of-flight accuracy.
    if (beam.mm > 200.0f) {
        const int raw = beam.confidence;
        const int conf = (raw < 0 || raw > CONFIDENCE_MAX) ? 0 : raw;
        sigma *= std::max(static_cast<float>(CONFIDENCE_NOMINAL)
                              / std::max(static_cast<float>(conf), 1.0f),
                          1.0f);
    }
    return sigma;
}

void EkfCore::updateRanges(const std::array<Beam, SENSOR_COUNT>& beams,
                           std::uint32_t nowMs) {
    mBeamsUsed = 0;
    int returns = 0;
    int present = 0;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        const std::size_t idx = static_cast<std::size_t>(i);
        mInnovations[idx] = 0.0f;
        if (!beams[idx].present) {
            mBeamStates[idx] = BeamState::Absent;
            continue;
        }
        present++;

        float reading = 0.0f;
        if (!beamReadingIn(beams[idx], &reading)) {
            mBeamStates[idx] = BeamState::NoReturn;
            mRangeNoReturns++;
            continue;
        }

        // Re-linearise per beam. Each fuseScalar moves the mean, so a beam that
        // predicted its range from the pre-correction pose would be answering a
        // question nobody asked -- several beams on one wall then stack their
        // corrections and overshoot.
        const Pose now{mState[0], mState[1], mState[2]};
        const Transform2d back = rewind(mConfig.rangeLatencyMs, nowMs);
        const Pose evalPose = now.transformBy(back);

        const RangePrediction rp = predictRange(evalPose, mMounts[idx]);
        if (rp.state == BeamState::NoReturn) {
            mBeamStates[idx] = BeamState::NoReturn;
            mRangeNoReturns++;
            continue;
        }
        if (rp.state == BeamState::Grazing) {
            mBeamStates[idx] = BeamState::Grazing;
            mRangeGrazed++;
            continue;
        }
        if (obstacleOwnsReading(rp.sx, rp.sy, rp.rayCos, rp.raySin, reading, rp.range)) {
            mBeamStates[idx] = BeamState::Deadzone;
            mRangeDeadzoned++;
            continue;
        }

        // The prediction was linearised at the rewound pose; carry its Jacobian
        // onto the current state through the rewind's own Jacobian.
        float H[3];
        if (back.translation.x == 0.0f && back.translation.y == 0.0f &&
            back.rotation.sin == 0.0f && back.rotation.cos == 1.0f) {
            H[0] = rp.H[0]; H[1] = rp.H[1]; H[2] = rp.H[2];
        } else {
            float J[9];
            transformJacobian(now, back, J);
            rowTimesMat3(rp.H, J, H);
        }

        const float sigma = rangeSigma(beams[idx], reading, rp.cosIncidence);

        returns++;
        const float innovation = reading - rp.range;
        mInnovations[idx] = innovation;
        if (fuseScalar(H, innovation, sigma * sigma, mConfig.rangeGateChi2)) {
            mBeamStates[idx] = BeamState::Used;
            mBeamsUsed++;
        } else {
            mBeamStates[idx] = BeamState::Rejected;
            mRangeRejects++;
        }
    }

    // Rejecting every beam, every cycle, is indistinguishable from having no
    // sensors at all unless it is counted. The blind streak covers the beams
    // that never reached the gate too: a filter permanently deadzoned by posts
    // is as deaf as one whose gate is too tight, and reported healthy.
    if (returns > 0 && mBeamsUsed == 0) {
        mRangeRejectStreak++;
    } else if (mBeamsUsed > 0) {
        mRangeRejectStreak = 0;
    }
    if (mBeamsUsed > 0) {
        mRangeBlindStreak = 0;
    } else if (present > 0) {
        mRangeBlindStreak++;
    }
}

int EkfCore::compareRangeFit(const std::array<Beam, SENSOR_COUNT>& beams,
                             const Pose& other, float* chi2Self,
                             float* chi2Other) const {
    const Pose self{mState[0], mState[1], mState[2]};
    const Pose alt{other.x, other.y, mState[2]};
    float sumSelf = 0.0f, sumOther = 0.0f;
    int scored = 0;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        const std::size_t idx = static_cast<std::size_t>(i);
        float reading = 0.0f;
        if (!beamReadingIn(beams[idx], &reading)) continue;

        const RangePrediction a = predictRange(self, mMounts[idx]);
        const RangePrediction b = predictRange(alt, mMounts[idx]);
        // Both hypotheses must be able to answer, or the comparison is between
        // different evidence for each and the ratio means nothing.
        if (!a.valid() || !b.valid()) continue;
        if (obstacleOwnsReading(a.sx, a.sy, a.rayCos, a.raySin, reading, a.range) ||
            obstacleOwnsReading(b.sx, b.sy, b.rayCos, b.raySin, reading, b.range)) {
            continue;
        }

        // One sigma per beam, at normal incidence, for BOTH hypotheses. Incidence
        // is a property of the hypothesis, so scoring each at its own would let
        // geometry alone decide: at maxIncidenceRad the two sigmas differ by
        // 2.45x, which is 6x in chi-square against an evidence ratio of 4. That
        // relocated the filter 96 in onto the hypothesis the same measurement
        // model calls less likely. Comparing raw chi-squares is only a
        // likelihood ratio when the noise scale is shared -- otherwise the
        // 2*sum(log sigma) term that penalises the wider hypothesis is missing.
        const float s = rangeSigma(beams[idx], reading, 1.0f);
        if (!(s > 0.0f)) continue;
        const float ra = (reading - a.range) / s;
        const float rb = (reading - b.range) / s;
        if (!std::isfinite(ra) || !std::isfinite(rb)) continue;
        sumSelf += ra * ra;
        sumOther += rb * rb;
        scored++;
    }

    if (chi2Self != nullptr) *chi2Self = sumSelf;
    if (chi2Other != nullptr) *chi2Other = sumOther;
    return scored;
}

bool EkfCore::rangesPreferMcl(const Inputs& in) const {
    float chi2Self = 0.0f, chi2Mcl = 0.0f;
    // One beam pins a range but not a direction, so it can prefer a hypothesis
    // that happens to sit at the same distance from a different wall. Two
    // non-parallel beams is the least that constrains a planar displacement.
    const int scored = compareRangeFit(in.beams, in.mclPose, &chi2Self, &chi2Mcl);
    if (scored < 2) return false;
    // A hypothesis that fits spends about one chi-square per beam, so below that
    // the filter's own pose explains the readings and there is nothing to fix.
    // Without this the ratio below is a comparison of two roundoff errors, and
    // two beams parallel to the displacement -- which cannot see it at all --
    // would relocate the filter on nothing.
    if (!(chi2Self > static_cast<float>(scored))) return false;
    const float ratio = std::max(mConfig.mclRelocEvidenceRatio, 1.0f);
    return chi2Self > chi2Mcl * ratio;
}

void EkfCore::updateMcl(const Inputs& in) {
    if (!in.mclValid) return;
    if (!std::isfinite(in.mclPose.x) || !std::isfinite(in.mclPose.y)) return;

    // MCL owns position; heading comes from the IMU, and MCL's particles are
    // clamped to the IMU anyway, so its heading carries no new information.
    const float scale = mConfig.mclSigmaScale * mConfig.mclSigmaScale;
    const float floorVar = mConfig.mclSigmaFloor * mConfig.mclSigmaFloor;
    const float rxx = std::max(in.mclCovXX * scale, floorVar);
    const float ryy = std::max(in.mclCovYY * scale, floorVar);
    const float rxy = in.mclCovXY * scale;

    const float S00 = mP[0] + rxx;
    const float S01 = mP[1] + rxy;
    const float S11 = mP[4] + ryy;
    const float det = S00 * S11 - S01 * S01;
    if (!(det > 1e-12f)) return;

    const float invDet = 1.0f / det;
    const float Si00 =  S11 * invDet;
    const float Si01 = -S01 * invDet;
    const float Si11 =  S00 * invDet;

    const float vx = in.mclPose.x - mState[0];
    const float vy = in.mclPose.y - mState[1];
    const float mahal = vx * (Si00 * vx + Si01 * vy) + vy * (Si01 * vx + Si11 * vy);
    if (!std::isfinite(mahal)) return;

    if (mahal > mConfig.mclGateChi2) {
        mMclRejectStreak++;
        mMclRejects++;
        // Gating is right for a bad reading and wrong for a kidnapped robot --
        // the two look identical for one cycle and nothing alike over ten. What
        // separates them is the distance readings, which are evidence about
        // where the robot is and are not MCL's opinion of itself: a cloud that
        // has genuinely relocated explains them, and one that has merely gone
        // flat does not.
        if (mMclRejectStreak >= mConfig.mclResetCycles && rangesPreferMcl(in)) {
            const float pTheta = mP[8];
            mState[0] = in.mclPose.x;
            mState[1] = in.mclPose.y;
            for (int i = 0; i < 9; i++) mP[i] = 0.0f;
            mP[0] = rxx;
            mP[4] = ryy;
            mP[1] = mP[3] = rxy;
            // Heading was not what MCL disagreed about; keep what the filter had
            // rather than asserting the IMU's sigma on a drivetrain that may not
            // even have an IMU.
            mP[8] = pTheta;
            if (symmetrize(mP)) mCovRepairs++;
            mMclRejectStreak = 0;
            mRelocateCount++;
            mRelocated = true;
        }
        return;
    }
    mMclRejectStreak = 0;

    // K = P H^T S^-1, with H selecting (x, y): P H^T is P's first two columns.
    float K[6];
    for (int r = 0; r < 3; r++) {
        const float p0 = mP[r * 3 + 0], p1 = mP[r * 3 + 1];
        K[r * 2 + 0] = p0 * Si00 + p1 * Si01;
        K[r * 2 + 1] = p0 * Si01 + p1 * Si11;
    }
    for (int i = 0; i < 6; i++) {
        if (!std::isfinite(K[i])) return;
    }

    mState[0] += K[0] * vx + K[1] * vy;
    mState[1] += K[2] * vx + K[3] * vy;
    mState[2] = wrapRad(mState[2] + K[4] * vx + K[5] * vy);

    float A[9];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            const float kh = (c == 0) ? K[r * 2 + 0] : (c == 1 ? K[r * 2 + 1] : 0.0f);
            A[r * 3 + c] = (r == c ? 1.0f : 0.0f) - kh;
        }
    }
    float AP[9], APAt[9];
    mat3Mul(A, mP, AP);
    mat3MulT(AP, A, APAt);

    for (int r = 0; r < 3; r++) {
        const float kr0 = K[r * 2 + 0] * rxx + K[r * 2 + 1] * rxy;
        const float kr1 = K[r * 2 + 0] * rxy + K[r * 2 + 1] * ryy;
        for (int c = 0; c < 3; c++) {
            mP[r * 3 + c] = APAt[r * 3 + c]
                          + kr0 * K[c * 2 + 0] + kr1 * K[c * 2 + 1];
        }
    }
    if (symmetrize(mP)) mCovRepairs++;
}

// ============================================================================
// One full cycle
// ============================================================================

Pose EkfCore::step(std::uint32_t nowMs, const Inputs& in) {
    predict(in.delta, in.hasHorizontalTracking, nowMs, in.headingValid);
    if (mConfig.useImuHeading && in.headingValid) updateHeading(in.headingRad, nowMs);
    if (mConfig.useRangeUpdates) updateRanges(in.beams, nowMs);
    if (mConfig.useMclUpdates) updateMcl(in);
    return state();
}

}  // namespace chad
