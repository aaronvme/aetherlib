#include "chad/tracking/ekf_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

namespace chad {

namespace {

constexpr float kWall = Field::FIELD_HALF_WALL;

// --- 3x3 helpers, row-major -------------------------------------------------

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

// Float arithmetic leaves P slightly asymmetric after every update, and the
// asymmetry compounds until a variance goes negative and the filter diverges.
void symmetrize3(float* P) {
    for (int r = 0; r < 3; r++) {
        for (int c = r + 1; c < 3; c++) {
            const float m = 0.5f * (P[r * 3 + c] + P[c * 3 + r]);
            P[r * 3 + c] = P[c * 3 + r] = m;
        }
    }
    for (int i = 0; i < 3; i++) P[i * 3 + i] = std::max(P[i * 3 + i], 1e-9f);
}

}  // namespace

// ============================================================================
// Construction
// ============================================================================

EkfTracking::EkfTracking(OdomTracking& odom,
                         std::array<pros::Distance*, SENSOR_COUNT> distanceSensors,
                         const Pose& startPose,
                         const Config& config)
    : mConfig(config),
      mOdom(odom),
      mDistanceSensors(distanceSensors) {
    mSensorMounts.fill({});
    setPose(startPose);
}

EkfTracking::~EkfTracking() {
    stopTracking();
}

// ============================================================================
// Lifecycle
// ============================================================================

void EkfTracking::startTracking() {
    if (mTask != nullptr) return;
    // Drop motion that accumulated while the filter was idle; it predates the
    // current state and would be integrated as though it had just happened.
    mOdom.consumeDelta(OdomTracking::CHANNEL_EKF);
    mRunning = true;
    mTask = new pros::Task([this]() { run(); });
}

void EkfTracking::stopTracking() {
    mRunning = false;
    if (mTask == nullptr) return;

    const uint32_t start = pros::millis();
    while (pros::millis() - start < 200) {
        const uint32_t state = mTask->get_state();
        if (state == pros::E_TASK_STATE_DELETED ||
            state == pros::E_TASK_STATE_INVALID) break;
        pros::delay(5);
    }
    const uint32_t state = mTask->get_state();
    if (state != pros::E_TASK_STATE_DELETED &&
        state != pros::E_TASK_STATE_INVALID) {
        mTask->remove();
    }
    delete mTask;
    mTask = nullptr;
}

bool EkfTracking::isTracking() const {
    return mRunning && mTask != nullptr;
}

void EkfTracking::run() {
    while (mRunning) {
        const uint32_t startTime = pros::millis();
        update();
        const int32_t elapsed = pros::millis() - startTime;
        const int32_t remaining = static_cast<int32_t>(mConfig.updatePeriodMs) - elapsed;
        pros::delay(remaining > 2 ? remaining : 2);
    }
}

// ============================================================================
// Pose management
// ============================================================================

void EkfTracking::setPose(const Pose& pose) {
    setPose(pose, 1.0f, 0.02f);
}

void EkfTracking::setPose(const Pose& pose, float posSigma, float thetaSigma) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mOdom.consumeDelta(OdomTracking::CHANNEL_EKF);

    mState[0] = pose.x;
    mState[1] = pose.y;
    mState[2] = wrapRad(pose.theta);

    for (int i = 0; i < 9; i++) mP[i] = 0.0f;
    mP[0] = posSigma * posSigma;
    mP[4] = posSigma * posSigma;
    mP[8] = thetaSigma * thetaSigma;

    mMclRejectStreak = 0;
    mRelocateCount = 0;
    mBeamsUsed = 0;
    mBeamStates.fill(BeamState::Absent);
    mInnovations.fill(0.0f);
}

Pose EkfTracking::getEstimate() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return {mState[0], mState[1], mState[2]};
}

EkfTracking::Uncertainty EkfTracking::uncertainty() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return {std::sqrt(std::max(mP[0], 0.0f)),
            std::sqrt(std::max(mP[4], 0.0f)),
            std::sqrt(std::max(mP[8], 0.0f))};
}

void EkfTracking::setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts) {
    mSensorMounts = mounts;
}

void EkfTracking::setDistanceSensors(
        const std::array<pros::Distance*, SENSOR_COUNT>& sensors) {
    mDistanceSensors = sensors;
}

// ============================================================================
// predict() — odometry drives the state, and inflates P
// ============================================================================

void EkfTracking::predict(const OdomDelta& delta) {
    const float theta = mState[2];
    const float mid = theta + delta.dTheta * 0.5f;
    const float c = std::cos(mid), s = std::sin(mid);

    const float dx = delta.dVert * c - delta.dHoriz * s;
    const float dy = delta.dVert * s + delta.dHoriz * c;

    mState[0] += dx;
    mState[1] += dy;
    mState[2] = wrapRad(theta + delta.dTheta);

    // F = I except for the heading column: rotating the robot swings the
    // displacement it just made.
    const float F[9] = {
        1.0f, 0.0f, -dy,
        0.0f, 1.0f,  dx,
        0.0f, 0.0f, 1.0f,
    };
    float FP[9], FPFt[9];
    mat3Mul(F, mP, FP);
    mat3MulT(FP, F, FPFt);

    // Odometry error accumulates with distance travelled, so noise is per-inch.
    // It is also anisotropic: a tank drive knows how far it drove far better
    // than whether it slid sideways, so build Q in the body frame and rotate it.
    const float alongSigma = mConfig.vertNoiseProp * std::fabs(delta.dVert)
                           + mConfig.posNoiseFloor;
    const float lateralSigma =
        mOdom.hasHorizontalTracking()
            ? mConfig.horizNoiseProp * std::fabs(delta.dHoriz) + mConfig.posNoiseFloor
            : mConfig.slipNoiseProp * std::fabs(delta.dVert) + mConfig.slipNoiseFloor;

    // Systematic component. Charging bias^2 * |ds| * decorrLen per tick makes the
    // accumulated variance bias^2 * L * decorrLen, so sigma grows like the real
    // error does over the distances a route covers.
    const float travel = std::fabs(delta.dVert) + std::fabs(delta.dHoriz);
    const float biasScale = travel * mConfig.biasDecorrIn;
    const float qBiasAlong = mConfig.vertBiasProp * mConfig.vertBiasProp * biasScale;
    const float qBiasLat = mOdom.hasHorizontalTracking()
        ? mConfig.horizNoiseProp * mConfig.horizNoiseProp * biasScale
        : mConfig.slipBiasProp * mConfig.slipBiasProp * biasScale;

    const float qa = alongSigma * alongSigma + qBiasAlong;
    const float ql = lateralSigma * lateralSigma + qBiasLat;
    const float qxx = qa * c * c + ql * s * s;
    const float qyy = qa * s * s + ql * c * c;
    const float qxy = (qa - ql) * c * s;

    const float thetaSigma = mConfig.thetaNoiseProp * std::fabs(delta.dTheta)
                           + mConfig.thetaNoisePerInch * std::fabs(delta.dVert)
                           + mConfig.thetaNoiseFloor;

    for (int i = 0; i < 9; i++) mP[i] = FPFt[i];
    mP[0] += qxx;
    mP[1] += qxy;
    mP[3] += qxy;
    mP[4] += qyy;
    mP[8] += thetaSigma * thetaSigma;
    symmetrize3(mP);
}

// ============================================================================
// Scalar correction (Joseph form)
// ============================================================================

bool EkfTracking::applyScalarUpdate(const float H[3], float innovation, float R,
                                    float gateChi2) {
    // S = H P H^T + R
    float PHt[3];
    for (int r = 0; r < 3; r++) {
        PHt[r] = mP[r * 3 + 0] * H[0] + mP[r * 3 + 1] * H[1] + mP[r * 3 + 2] * H[2];
    }
    const float S = H[0] * PHt[0] + H[1] * PHt[1] + H[2] * PHt[2] + R;
    if (!(S > 1e-12f)) return false;

    if (gateChi2 > 0.0f && (innovation * innovation) / S > gateChi2) return false;

    const float invS = 1.0f / S;
    const float K[3] = {PHt[0] * invS, PHt[1] * invS, PHt[2] * invS};

    mState[0] += K[0] * innovation;
    mState[1] += K[1] * innovation;
    mState[2] = wrapRad(mState[2] + K[2] * innovation);

    // Joseph form: P = (I-KH) P (I-KH)^T + K R K^T. The short form
    // (I-KH)P is cheaper but loses symmetry and can go indefinite in float.
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
    symmetrize3(mP);
    return true;
}

// ============================================================================
// Ray casting
// ============================================================================

EkfTracking::WallHit EkfTracking::castWalls(float sx, float sy,
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

bool EkfTracking::obstacleOwnsReading(float sx, float sy,
                                      float rayCos, float raySin,
                                      float readingIn, float wallRange) const {
    if (mCircleObstacles == nullptr) return false;

    // Widened by the filter's own position sigma: an unsure filter is also
    // unsure which beams a post could be spoiling, and should drop more.
    const float posSigma = std::sqrt(std::max(mP[0], 0.0f) + std::max(mP[4], 0.0f));
    const float tol = mConfig.obstacleRangeTol + mConfig.obstacleSigmaPad * posSigma;

    for (const auto& circle : *mCircleObstacles) {
        const float ox = circle.x - sx;
        const float oy = circle.y - sy;
        const float along = ox * rayCos + oy * raySin;
        if (along < 0.0f) continue;
        const float perp = std::fabs(-ox * raySin + oy * rayCos);
        if (perp > circle.radius + along * mConfig.obstacleConeFrac) continue;

        const float postRange = std::sqrt(ox * ox + oy * oy) - circle.radius;
        // Either the return matches the post surface, or the post stands in
        // front of the wall this beam would otherwise have measured. Both mean
        // the reading says nothing about where the wall is.
        if (std::fabs(readingIn - postRange) <= tol) return true;
        if (postRange > 0.0f && postRange + tol < wallRange) return true;
    }
    return false;
}

// ============================================================================
// updateRanges() — distance sensors against the field walls
// ============================================================================

void EkfTracking::updateRanges() {
    mBeamsUsed = 0;
    const float x = mState[0], y = mState[1], theta = mState[2];
    const float ct = std::cos(theta), st = std::sin(theta);

    for (int i = 0; i < SENSOR_COUNT; i++) {
        mInnovations[i] = 0.0f;
        if (mDistanceSensors[i] == nullptr) {
            mBeamStates[i] = BeamState::Absent;
            continue;
        }

        const float mm = static_cast<float>(mDistanceSensors[i]->get());
        const int conf = mDistanceSensors[i]->get_confidence();
        if (!std::isfinite(mm) || mm >= mConfig.sentinelMm || mm < mConfig.minRangeMm ||
            (mm > 200.0f && conf < mConfig.minConfidence)) {
            mBeamStates[i] = BeamState::NoReturn;
            continue;
        }
        const float reading = mm * MM_TO_INCH;

        // Sensor origin and ray direction in the world, from the current mean.
        const Pose& m = mSensorMounts[i];
        const float sx = x + m.x * ct - m.y * st;
        const float sy = y + m.x * st + m.y * ct;
        const float phi = m.theta;
        const float rayCos = std::cos(theta + phi);
        const float raySin = std::sin(theta + phi);

        const WallHit hit = castWalls(sx, sy, rayCos, raySin);
        if (!hit.valid) {
            mBeamStates[i] = BeamState::NoReturn;
            continue;
        }
        if (hit.cosIncidence < std::cos(mConfig.maxIncidenceRad)) {
            mBeamStates[i] = BeamState::Grazing;
            continue;
        }
        if (obstacleOwnsReading(sx, sy, rayCos, raySin, reading, hit.range)) {
            mBeamStates[i] = BeamState::Deadzone;
            continue;
        }

        // r = (d - n.s) / (n.u), so with N = d - n.s and D = n.u:
        //   dr/dx = -n_x / D,  dr/dy = -n_y / D
        //   dr/dtheta = -(n . ds/dtheta)/D - r (n . du/dtheta)/D
        const float D = hit.nx * rayCos + hit.ny * raySin;
        if (std::fabs(D) < 1e-4f) {
            mBeamStates[i] = BeamState::Grazing;
            continue;
        }
        const float invD = 1.0f / D;
        // ds/dtheta rotates the mount offset; du/dtheta is the ray turned 90 deg.
        const float dsx = -m.x * st - m.y * ct;
        const float dsy =  m.x * ct - m.y * st;
        const float dux = -raySin;
        const float duy =  rayCos;

        const float H[3] = {
            -hit.nx * invD,
            -hit.ny * invD,
            (-(hit.nx * dsx + hit.ny * dsy) - hit.range * (hit.nx * dux + hit.ny * duy)) * invD,
        };

        // Range noise grows with distance, and a glancing beam maps a given
        // range error into a larger position error, so divide by cos(incidence).
        float sigma = (mConfig.rangeSigmaBase + mConfig.rangeSigmaProp * reading)
                    / std::max(hit.cosIncidence, 0.25f);
        if (mm > 200.0f) sigma *= 32.0f / std::max(static_cast<float>(conf), 1.0f);

        const float innovation = reading - hit.range;
        mInnovations[i] = innovation;
        if (applyScalarUpdate(H, innovation, sigma * sigma, mConfig.rangeGateChi2)) {
            mBeamStates[i] = BeamState::Used;
            mBeamsUsed++;
        } else {
            mBeamStates[i] = BeamState::Rejected;
        }
    }
}

// ============================================================================
// updateImuHeading()
// ============================================================================

void EkfTracking::updateImuHeading() {
    const float imu = mOdom.imuHeadingRad();
    if (!std::isfinite(imu)) return;
    const float H[3] = {0.0f, 0.0f, 1.0f};
    // No gate: on this drivetrain the IMU is the only absolute heading source,
    // so gating it out would leave the filter's heading unobserved.
    applyScalarUpdate(H, angleDiffRad(mState[2], imu),
                      mConfig.imuSigmaRad * mConfig.imuSigmaRad, 0.0f);
}

// ============================================================================
// updateMcl() — the particle filter as a 2-dof position measurement
// ============================================================================

void EkfTracking::updateMcl() {
    if (mMcl == nullptr || !mMcl->isTracking()) return;

    const MclTracking::Estimate est = mMcl->estimate();
    if (!(est.nEff > 0.0f)) return;

    // MCL owns position; heading comes from the IMU, and MCL's particles are
    // clamped to the IMU anyway, so its heading carries no new information.
    const float scale = mConfig.mclSigmaScale * mConfig.mclSigmaScale;
    const float floorVar = mConfig.mclSigmaFloor * mConfig.mclSigmaFloor;
    const float rxx = std::max(est.covXX * scale, floorVar);
    const float ryy = std::max(est.covYY * scale, floorVar);
    const float rxy = est.covXY * scale;

    const float S00 = mP[0] + rxx;
    const float S01 = mP[1] + rxy;
    const float S11 = mP[4] + ryy;
    const float det = S00 * S11 - S01 * S01;
    if (!(std::fabs(det) > 1e-12f)) return;

    const float invDet = 1.0f / det;
    const float Si00 =  S11 * invDet;
    const float Si01 = -S01 * invDet;
    const float Si11 =  S00 * invDet;

    const float vx = est.pose.x - mState[0];
    const float vy = est.pose.y - mState[1];
    const float mahal = vx * (Si00 * vx + Si01 * vy) + vy * (Si01 * vx + Si11 * vy);

    if (mahal > mConfig.mclGateChi2) {
        mMclRejectStreak++;
        // Gating is right for a bad reading and wrong for a kidnapped robot --
        // the two look identical for one cycle and nothing alike over ten. A
        // healthy cloud that keeps insisting is telling the truth.
        const float neffFrac = est.nEff / static_cast<float>(mMcl->config().particleCount);
        if (mMclRejectStreak >= mConfig.mclResetCycles &&
            neffFrac >= mConfig.mclMinNeffFrac) {
            mState[0] = est.pose.x;
            mState[1] = est.pose.y;
            for (int i = 0; i < 9; i++) mP[i] = 0.0f;
            mP[0] = rxx;
            mP[4] = ryy;
            mP[1] = mP[3] = rxy;
            mP[8] = mConfig.imuSigmaRad * mConfig.imuSigmaRad;
            symmetrize3(mP);
            mMclRejectStreak = 0;
            mRelocateCount++;
            mSnapNextSync = true;
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

    // K R K^T
    for (int r = 0; r < 3; r++) {
        const float kr0 = K[r * 2 + 0] * rxx + K[r * 2 + 1] * rxy;
        const float kr1 = K[r * 2 + 0] * rxy + K[r * 2 + 1] * ryy;
        for (int c = 0; c < 3; c++) {
            mP[r * 3 + c] = APAt[r * 3 + c]
                          + kr0 * K[c * 2 + 0] + kr1 * K[c * 2 + 1];
        }
    }
    symmetrize3(mP);
}

// ============================================================================
// update() — one full cycle
// ============================================================================

Pose EkfTracking::update() {
    const OdomDelta delta = mOdom.consumeDelta(OdomTracking::CHANNEL_EKF);

    Pose fused;
    {
        std::lock_guard<pros::Mutex> lock(mMutex);
        predict(delta);
        if (mConfig.useImuHeading) updateImuHeading();
        if (mConfig.useRangeUpdates) updateRanges();
        if (mConfig.useMclUpdates) updateMcl();
        fused = {mState[0], mState[1], mState[2]};
    }

    // Written outside the lock: OdomTracking takes its own mutex, and holding
    // both in one order here while getPose() callers take them in the other is
    // how a deadlock gets built.
    if (mConfig.autoSync) {
        const Pose odomPose = mOdom.getPose();
        if (mSnapNextSync) {
            mOdom.setPose(fused);
            mSnapNextSync = false;
        } else {
            const float k = clamp(mConfig.syncProp, 0.0f, 1.0f);
            const float kt = clamp(mConfig.thetaSyncProp, 0.0f, 1.0f);
            const float dTheta = angleDiffRad(odomPose.theta, fused.theta);
            mOdom.setPose({odomPose.x + k * (fused.x - odomPose.x),
                           odomPose.y + k * (fused.y - odomPose.y),
                           wrapRad(odomPose.theta + kt * dTheta)});
        }
    }
    return fused;
}

}  // namespace chad
