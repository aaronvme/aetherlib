#include "chad/tracking/rcl_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <tuple>

#include "pros/error.h"

namespace chad {
namespace {
/// Median of a small scratch array, in place.
float medianOf(float* v, int n) {
    if (n <= 0) return 0.0f;
    std::sort(v, v + n);
    return (n & 1) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

/// Absolute expiry for a lifetime; 0 = never.
std::uint32_t expiryFor(float lifetimeMs) {
    if (lifetimeMs <= 0.0f) return 0;
    const std::uint32_t t = pros::millis() + static_cast<std::uint32_t>(lifetimeMs);
    return t != 0 ? t : 1;
}

bool expired(std::uint32_t expiresAtMs, std::uint32_t nowMs) {
    return expiresAtMs != 0 && static_cast<std::int32_t>(nowMs - expiresAtMs) >= 0;
}

CorrectionLimiter::Config limiterConfig(const RclTracking::Config& c) {
    return {c.maxCorrectionRate, c.maxCorrectionBurst};
}

ReadingTrust::Config trustConfig(const RclTracking::Config& c) {
    ReadingTrust::Config t;
    t.tolIn = c.trustTolIn;
    t.tolFrac = c.trustTolFrac;
    t.longTolIn = c.maxWallInnovation;
    t.graceMs = c.trustGraceMs;
    t.rampMs = c.trustRampMs;
    t.shortSeverityIn = c.trustShortSeverityIn;
    t.longWeight = c.trustLongWeight;
    t.holdMs = c.trustHoldMs;
    t.recoverMs = c.trustRecoverMs;
    t.floor = c.trustFloor;
    return t;
}

void resetTrust(RclTracking::RclSensor& s) {
    s.trustState.reset();
    s.trust = 1.0f;
    s.distrustSamples = 0;
}
}  // namespace

float RclTracking::intersectLineSegment(float rx, float ry,
                                         float rCos, float rSin,
                                         float x1, float y1,
                                         float x2, float y2,
                                         float maxRange) const {
    float xMin = std::min(x1, x2);
    float xMax = std::max(x1, x2);
    float yMin = std::min(y1, y2);
    float yMax = std::max(y1, y2);

    if ((rCos > 0.0f && rx > xMax) || (rCos < 0.0f && rx < xMin) ||
        (rSin > 0.0f && ry > yMax) || (rSin < 0.0f && ry < yMin)) {
        return maxRange;
    }

    float x3 = rx, y3 = ry;
    float x4 = rx + rCos * maxRange;
    float y4 = ry + rSin * maxRange;

    float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::fabs(den) < 1e-6f) return maxRange;

    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
    float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;

    if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) {
        return u * maxRange;
    }
    return maxRange;
}

std::pair<int, float> RclTracking::nearestWall(const RclSensor& s) const {
    float minDist = 1e9f;
    int wall = -1;

    // Choose the wall from a point INSIDE the field, whatever the estimate
    // currently claims. The robot is always inside it, so a beam pointing east
    // can only be looking at the east wall -- but an estimate that has run out
    // through a wall reverses that: from x = -180 the west wall is the one
    // ahead of an east-facing beam, so the solve picks it, computes a
    // coordinate against the wrong wall, and the recovery it was supposed to
    // drive lands the pose somewhere else wrong. That is the 69 in residual a
    // post-wedge used to settle at instead of recovering.
    //
    // Clamped to just INSIDE the wall rather than onto it. Exactly on it, the
    // wall a ray points at sits at distance zero, which fails the `> 0` test
    // and leaves that ray with no wall at all -- and the ray that loses out is
    // the one aimed at the near wall, which is the short, unambiguous reading
    // recovery depends on. A wedged robot had its one good sensor discarded
    // for that reason and sat 136 in out with nothing to pull it back.
    constexpr float kInsetIn = 1.0f;
    const float W = Field::FIELD_HALF_WALL;
    const float px = clamp(s.worldX, -W + kInsetIn, W - kInsetIn);
    const float py = clamp(s.worldY, -W + kInsetIn, W - kInsetIn);

    if (std::fabs(s.rayCos) > 1e-6f) {
        float dEast = (W - px) / s.rayCos;
        if (dEast > 0.0f && dEast < minDist) { minDist = dEast; wall = 2; }

        float dWest = (-W - px) / s.rayCos;
        if (dWest > 0.0f && dWest < minDist) { minDist = dWest; wall = 4; }
    }

    if (std::fabs(s.raySin) > 1e-6f) {
        float dNorth = (W - py) / s.raySin;
        if (dNorth > 0.0f && dNorth < minDist) { minDist = dNorth; wall = 1; }

        float dSouth = (-W - py) / s.raySin;
        if (dSouth > 0.0f && dSouth < minDist) { minDist = dSouth; wall = 3; }
    }

    if (wall < 0) return {-1, 0.0f};
    return {wall, minDist};
}

bool RclTracking::farSide(float px, float py) const {
    if (!mConfig.farSideGate) return false;
    return mConfig.farSideNx * px + mConfig.farSideNy * py > mConfig.farSideD;
}

RclTracking::RclTracking(OdomTracking& odom, const Config& config)
    : mConfig(config), mOdom(odom), mCorrLimiter(limiterConfig(config)) {}

RclTracking::~RclTracking() {
    stopTracking();
}

void RclTracking::setSensor(int index, pros::Distance* sensor, const Pose& mountOffset) {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return;
    std::lock_guard<pros::Mutex> lock(mMutex);
    mSensors[index].sensor = sensor;
    mSensors[index].mountOffset = mountOffset;
    mSensors[index].enabled = true;
    mSensors[index].rawMm = -1;
    mSensors[index].heldMs = 0.0f;
    mSensors[index].faulted = false;
    resetTrust(mSensors[index]);
    mFaultedMask &= ~(1u << index);
    mSampleGen++;
}

void RclTracking::configureSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors,
                                     const std::array<Pose, MAX_DISTANCE_SENSORS>& mounts) {
    int activeCount = 0;
    std::lock_guard<pros::Mutex> lock(mMutex);
    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        mSensors[i].rawMm = -1;
        mSensors[i].heldMs = 0.0f;
        mSensors[i].fresh = false;
        mSensors[i].faulted = false;
        mSensors[i].cls = ReadingClass::Invalid;
        resetTrust(mSensors[i]);
        mDisableUntilMs[i] = 0;
        if (sensors[i] != nullptr) {
            mSensors[i].sensor       = sensors[i];
            mSensors[i].mountOffset  = mounts[i];
            mSensors[i].enabled      = true;
            activeCount++;
        } else {
            mSensors[i].sensor       = nullptr;
            mSensors[i].mountOffset  = {};
            mSensors[i].enabled      = false;
        }
    }
    mFaultedMask = 0;
    mSampleGen++;

    // A count, not a loop bound. Every walk below covers all the slots and
    // skips the empty ones, so a config that leaves a hole -- ports 0, 1, 2 and
    // 5 populated, say -- is read in full instead of stopping at slot 3 and
    // calling an empty one a sensor.
    mConfig.sensorCount = activeCount;
}

void RclTracking::enableSensor(int index) {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return;
    std::lock_guard<pros::Mutex> lock(mMutex);
    mSensors[index].enabled = true;
    mDisableUntilMs[index] = 0;
}

void RclTracking::disableSensor(int index) {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return;
    std::lock_guard<pros::Mutex> lock(mMutex);
    mSensors[index].enabled = false;
    mDisableUntilMs[index] = 0;
}

void RclTracking::disableSensorFor(int index, float durationMs) {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return;
    std::lock_guard<pros::Mutex> lock(mMutex);
    mSensors[index].enabled = false;
    if (durationMs <= 0.0f) {           // until enableSensor()
        mDisableUntilMs[index] = 0;
        return;
    }
    // 0 marks no deadline, hence the nudge on wraparound.
    const std::uint32_t until = pros::millis() + static_cast<std::uint32_t>(durationMs);
    mDisableUntilMs[index] = until != 0 ? until : 1;
}

void RclTracking::addLineObstacle(float x1, float y1, float x2, float y2, float lifetimeMs) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mLineObstacles.push_back({x1, y1, x2, y2, lifetimeMs, expiryFor(lifetimeMs)});
}

void RclTracking::addCircleObstacle(float x, float y, float radius, float lifetimeMs) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mCircleObstacles.push_back({x, y, radius, lifetimeMs, expiryFor(lifetimeMs)});
}

void RclTracking::clearObstacles() {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mLineObstacles.clear();
    mCircleObstacles.clear();
}

void RclTracking::armFarSideLocked(float nx, float ny, float d, float lifetimeMs) {
    const float len = std::hypot(nx, ny);
    if (len < 1e-6f) return;
    mConfig.farSideNx = nx / len;
    mConfig.farSideNy = ny / len;
    mConfig.farSideD = d / len;
    mConfig.farSideGate = true;
    mFarSideOrientPending = false;
    mFarSideClearRequested = false;
    mFarSideArmedMs = pros::millis();
    mFarSideTimed = lifetimeMs > 0.0f;
    mFarSideDeadlineMs =
        mFarSideTimed ? mFarSideArmedMs + static_cast<std::uint32_t>(lifetimeMs) : 0;
}

void RclTracking::clearFarSideLocked() {
    mConfig.farSideGate = false;
    mFarSideTimed = false;
    mFarSideOrientPending = false;
    mFarSideDeadlineMs = 0;
}

void RclTracking::expireFarSideLocked(std::uint32_t nowMs) {
    if (mFarSideClearRequested.exchange(false) ||
        (mConfig.farSideGate && mFarSideTimed &&
         static_cast<std::int32_t>(nowMs - mFarSideDeadlineMs) >= 0)) {
        clearFarSideLocked();
    }
}

void RclTracking::orientFarSideLocked(const Pose& p) {
    if (mConfig.farSideNx * p.x + mConfig.farSideNy * p.y > mConfig.farSideD) {
        mConfig.farSideNx = -mConfig.farSideNx;
        mConfig.farSideNy = -mConfig.farSideNy;
        mConfig.farSideD = -mConfig.farSideD;
    }
}

bool RclTracking::farSideGateActive() const {
    return mConfig.farSideGate && !mFarSideClearRequested &&
           !(mFarSideTimed &&
             static_cast<std::int32_t>(pros::millis() - mFarSideDeadlineMs) >= 0);
}

float RclTracking::farSideGateRemainingMs() const {
    if (!mConfig.farSideGate || mFarSideClearRequested || !mFarSideTimed) return 0.0f;
    const std::int32_t left = static_cast<std::int32_t>(mFarSideDeadlineMs - pros::millis());
    return left > 0 ? static_cast<float>(left) : 0.0f;
}

void RclTracking::setFarSideGate(float nx, float ny, float d, float lifetimeMs) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    armFarSideLocked(nx, ny, d, lifetimeMs);
}

void RclTracking::setFarSideGateFromLine(float x1, float y1, float x2, float y2,
                                         float lifetimeMs) {
    // Normal of the segment, either orientation.
    float nx = -(y2 - y1);
    float ny =  (x2 - x1);
    const float len = std::hypot(nx, ny);
    if (len < 1e-6f) return;
    nx /= len;
    ny /= len;

    // Keep the half the robot stands in; see the header for the re-pick rule.
    const Pose p = mOdom.getPose();
    std::lock_guard<pros::Mutex> lock(mMutex);
    armFarSideLocked(nx, ny, nx * x1 + ny * y1, lifetimeMs);
    orientFarSideLocked(p);
    const std::int32_t sincePose =
        static_cast<std::int32_t>(mFarSideArmedMs - mLastHardResetMs);
    mFarSideOrientPending = !(mHaveHardReset && sincePose >= 0 &&
                              sincePose <= static_cast<std::int32_t>(kFarSideOrientWindowMs));
}

void RclTracking::clearFarSideGate() {
    // Called from opcontrol; the mutex may be orphaned by a deleted auton task.
    if (mMutex.take(20)) {
        clearFarSideLocked();
        mMutex.give();
    } else {
        mFarSideClearRequested = true;
    }
}

void RclTracking::setConfig(const Config& config) {
    std::lock_guard<pros::Mutex> lock(mMutex);

    const int count = mConfig.sensorCount;
    const bool gate = mConfig.farSideGate;
    const float gnx = mConfig.farSideNx, gny = mConfig.farSideNy;
    const float gd = mConfig.farSideD;

    mConfig = config;

    mConfig.sensorCount = count;
    // The gate is match state, not tuning. It is armed by the routine and
    // expires on its own clock, so a Config written over the top of it -- which
    // is what every slider in the simulator does -- must not take it down.
    mConfig.farSideGate = gate;
    mConfig.farSideNx = gnx;
    mConfig.farSideNy = gny;
    mConfig.farSideD = gd;

    mCorrLimiter.setConfig(limiterConfig(mConfig));
}

void RclTracking::setAutoSync(bool enabled) {
    std::lock_guard<pros::Mutex> lock(mMutex);
    mConfig.autoSync = enabled;
}

void RclTracking::pollDevices(RawSamples& out) {
    {
        std::lock_guard<pros::Mutex> lock(mMutex);
        expireDisableTimersLocked(pros::millis());
        out.gen = mSampleGen;
        for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
            const auto& s = mSensors[i];
            out.s[i].dev = s.enabled ? s.sensor : nullptr;
        }
    }
    for (auto& r : out.s) {
        if (r.dev == nullptr) continue;
        r.mm = r.dev->get();
        r.conf = r.mm == PROS_ERR ? PROS_ERR : r.dev->get_confidence();
    }
}

bool RclTracking::ingestSamples(const RawSamples& raw) {
    bool anyFresh = false;
    const bool current = raw.gen == mSampleGen;

    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        auto& s = mSensors[i];
        // A slot swapped, re-enabled or reset since the poll has no sample.
        if (!current || s.sensor == nullptr || !s.enabled || raw.s[i].dev != s.sensor) {
            s.fresh = false;
            continue;
        }

        const std::int32_t readingMm = raw.s[i].mm;
        const std::int32_t conf = raw.s[i].conf;

        // PROS_ERR (ENODEV unplugged or wrong device, EAGAIN still booting) is
        // not a reading. Taken as one it lands past 2000 mm, which near a wall
        // is Impossible -- displacement evidence every cycle, from a cable --
        // and being constant the stale hold would keep re-admitting it. The
        // verdict is cleared here because classifyAll() keeps the last one for
        // a sensor with nothing new, and the evidence count reads it.
        if (readingMm == PROS_ERR || conf == PROS_ERR) {
            s.faulted = true;
            s.fresh = false;
            s.valid = false;
            s.pairVerified = false;
            s.outlier = false;
            s.cls = ReadingClass::Invalid;
            s.innovationIn = 0.0f;
            s.rawMm = -1;               // the first sample back counts as new
            s.heldMs = 0.0f;
            mFaultedMask |= 1u << i;
            continue;
        }
        s.faulted = false;
        mFaultedMask &= ~(1u << i);

        // The device holds its last value between its own refreshes -- about
        // every 30 ms against this loop's 10 -- so an unchanged raw reading is
        // the signature of a sample already counted. Folding it in again lets
        // one physical measurement vote three times, which triples the
        // effective sync rate and makes the noise look three times more
        // consistent than it is.
        //
        // A value that genuinely has not moved for staleHoldMs is let through
        // again: a robot sitting still against a wall reports the same
        // millimetre for as long as you leave it there, and refusing that would
        // blind the filter exactly where it is most sure of itself.
        if (readingMm != s.rawMm) {
            s.fresh = true;
            s.heldMs = 0.0f;
        } else {
            s.heldMs += mDtMs;
            if (mConfig.staleHoldMs > 0.0f && s.heldMs >= mConfig.staleHoldMs) {
                s.fresh = true;
                s.heldMs = 0.0f;
            } else {
                s.fresh = false;
            }
        }
        s.rawMm = readingMm;
        s.readingInch = readingMm * MM_TO_INCH;
        s.confidence = conf;
        if (s.fresh) anyFresh = true;
    }

    return anyFresh;
}

void RclTracking::updateSensorPoses(const Pose& robotPose) {
    float rCos = std::cos(robotPose.theta);
    float rSin = std::sin(robotPose.theta);

    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        auto& s = mSensors[i];
        if (s.sensor == nullptr || !s.enabled) {
            s.valid = false;
            s.fresh = false;
            s.cls = ReadingClass::Invalid;
            continue;
        }

        s.mountWorldX = s.mountOffset.x * rCos - s.mountOffset.y * rSin;
        s.mountWorldY = s.mountOffset.x * rSin + s.mountOffset.y * rCos;
        s.worldX = robotPose.x + s.mountWorldX;
        s.worldY = robotPose.y + s.mountWorldY;

        float sensorWorldAngle = robotPose.theta + s.mountOffset.theta;
        s.worldHeading = sensorWorldAngle;
        s.rayCos = std::cos(sensorWorldAngle);
        s.raySin = std::sin(sensorWorldAngle);
    }

    classifyAll();
    checkPairs();

    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        mSensors[i].valid = isValidReading(i);
    }
}

RclTracking::ReadingClass RclTracking::classifyReading(int index, float& wallDist) const {
    wallDist = 0.0f;
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return ReadingClass::Invalid;
    const auto& s = mSensors[index];
    if (!s.enabled || s.sensor == nullptr) return ReadingClass::Invalid;

    // A cone that is sweeping cannot be pinned to one heading, and the smear is
    // common to every sensor, so the gate suppresses the cycle rather than
    // picking winners inside it.
    if (mTurnHeld) return ReadingClass::Turning;

    if (!s.fresh) return ReadingClass::Stale;

    float readingInch = s.readingInch;
    // The V5 sensor's rated minimum is 20 mm; anything under it is not a range.
    if (readingInch < 20.0f * MM_TO_INCH) return ReadingClass::Invalid;

    if (readingInch > 2000.0f * MM_TO_INCH) {
        auto [w, range] = nearestWall(s);
        // Silence with a wall well inside reach is itself a measurement: the
        // wall is not where the pose says it is.
        if (w >= 0 && range < mConfig.noReturnWallLimit) {
            // Unless the gate says we should not have been looking there.
            if (farSide(s.worldX + s.rayCos * range, s.worldY + s.raySin * range)) {
                return ReadingClass::FarSide;
            }
            return ReadingClass::Impossible;
        }
        return ReadingClass::Invalid;
    }

    if (readingInch > 200.0f * MM_TO_INCH && s.confidence < mConfig.confidenceThreshold) {
        return ReadingClass::LowConfidence;
    }

    float angleTolRad = degToRad(mConfig.angleTolerance);
    float angleMod = std::fmod(std::fabs(s.worldHeading), HALF_PI);
    if (angleMod > angleTolRad && angleMod < HALF_PI - angleTolRad) {
        return ReadingClass::OffAxis;
    }

    auto [wall, wallRange] = nearestWall(s);
    if (wall < 0) return ReadingClass::Invalid;
    wallDist = wallRange;

    // The return itself, not the wall behind it: a reading that lands across
    // the auton line is ranging one of the two robots over there.
    if (farSide(s.worldX + s.rayCos * readingInch,
                s.worldY + s.raySin * readingInch)) {
        return ReadingClass::FarSide;
    }

    // The post map, applied whatever state the filter is in -- including a
    // recovery, where the pose it is evaluated against is known to be wrong.
    //
    // Suspending it during a recovery was tried, on the reasoning that a wrong
    // pose puts the posts in the wrong places and the test only costs readings.
    // It does cost readings -- a pose 34 in out put a post in front of all four
    // beams at once and left the recovery with nothing -- but suspending it was
    // measurably worse: the filter recovered onto a post and settled 14 in from
    // the truth calling itself healthy, where keeping it leaves the pose 34 in
    // out and still flying the recovery flag.
    //
    // Wrong and quiet loses matches; wrong and loud does not. The beam reset in
    // the routines scores every reading against this same map at once and can
    // tell a post from a wall, which is the tool for the job, and isDisplaced()
    // is what tells it to run.
    for (const auto& obs : mLineObstacles) {
        float dist = intersectLineSegment(s.worldX, s.worldY, s.rayCos, s.raySin,
                                          obs.x1, obs.y1, obs.x2, obs.y2, wallRange);
        if (dist < wallRange) return ReadingClass::PostOwned;
    }

    for (const auto& obs : mCircleObstacles) {
        const float ox = obs.x - s.worldX;
        const float oy = obs.y - s.worldY;
        const float along = ox * s.rayCos + oy * s.raySin;
        if (along < 0.0f) continue;
        const float perp = std::fabs(-ox * s.raySin + oy * s.rayCos);
        if (perp > obs.radius + along * mConfig.obstacleConeFrac) continue;

        const float postRange = std::sqrt(ox * ox + oy * oy) - obs.radius;
        if (std::fabs(readingInch - postRange) <= mConfig.obstacleRangeTol) {
            return ReadingClass::PostOwned;
        }
        if (postRange > 0.0f && postRange + mConfig.obstacleRangeTol < wallRange) {
            return ReadingClass::PostOwned;
        }
    }

    float innovation = readingInch - wallRange;
    if (innovation > mConfig.maxWallInnovation) {
        return ReadingClass::TooFar;
    }
    if (innovation < -mConfig.maxWallInnovation) {
        return ReadingClass::Blocked;
    }

    return ReadingClass::WallHit;
}

void RclTracking::classifyAll() {
    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        auto& s = mSensors[i];
        s.pairVerified = false;
        s.outlier = false;
        if (s.sensor == nullptr || !s.enabled) {
            s.cls = ReadingClass::Invalid;
            s.innovationIn = 0.0f;
            continue;
        }
        // A sensor with nothing new keeps the verdict it already had. It cannot
        // vote either way -- voteTier() requires a fresh sample -- so
        // reclassifying it would only replace a useful answer with "no new
        // sample" on most cycles, which is what made the diagnostics flicker
        // rather than read.
        if (!s.fresh) continue;

        float wallDist = 0.0f;
        s.cls = classifyReading(i, wallDist);
        s.innovationIn = (wallDist > 0.0f) ? (s.readingInch - wallDist) : 0.0f;
    }
}

void RclTracking::updateTrustLocked(std::uint32_t nowMs) {
    const ReadingTrust::Config tc = trustConfig(mConfig);
    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        auto& s = mSensors[i];
        if (s.sensor == nullptr || !s.enabled || !s.fresh || s.faulted) continue;
        // While displaced the prediction is known wrong, so disagreeing with it
        // says nothing about the sensor.
        if (!mDisplaced) {
            ReadingTrust::Verdict v = ReadingTrust::Verdict::None;
            // A spanning pair needs no prediction to be right.
            if (s.pairVerified) {
                v = ReadingTrust::Verdict::Consistent;
            } else if (s.cls == ReadingClass::WallHit || s.cls == ReadingClass::Blocked ||
                       s.cls == ReadingClass::TooFar) {
                v = ReadingTrust::judge(s.innovationIn, s.readingInch, tc);
            }
            s.trust = s.trustState.observe(v, s.innovationIn, s.readingInch, nowMs, tc);
            s.distrustSamples = s.trust < mConfig.trustFloor ? s.distrustSamples + 1 : 0;
        }
        s.valid = isValidReading(i);
    }
}

int RclTracking::voteTier(int index) const {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return -1;
    const auto& s = mSensors[index];
    if (s.sensor == nullptr || !s.enabled) return -1;
    // One physical sample, one vote. Without this a 30 ms device feeding a
    // 10 ms loop is counted three times.
    if (!s.fresh) return -1;

    // An opposing pair that spans the field is right about the wall whatever
    // the estimate thinks -- which is the whole point of checking it, and why
    // it outranks the innovation gate that would otherwise throw it away for
    // disagreeing with the pose that is itself the thing in the wrong.
    if (s.pairVerified) return 2;
    // Repeatedly short of the wall: something is standing in the beam. A long
    // reading cannot be an obstruction, so while displaced one still votes
    // (tier 0 below) whatever trust it lost to the pose being wrong.
    if (s.trust < mConfig.trustFloor &&
        !(mDisplaced && s.cls == ReadingClass::TooFar)) return -1;
    if (s.cls == ReadingClass::WallHit) return 1;

    // Readings the innovation gate rejected. While the pose is known to be
    // wrong that gate is meaningless -- it is measuring against the very thing
    // under suspicion -- so these become usable, but only as a last resort:
    // an axis that still has a clean reading uses that instead. Mixing the two
    // is what made a wedged robot worse rather than better, because the beam
    // ranging a post outvoted the one ranging the wall.
    if (mDisplaced && (s.cls == ReadingClass::TooFar ||
                       s.cls == ReadingClass::Blocked)) return 0;
    return -1;
}

void RclTracking::checkPairs() {
    // Two beams pointing opposite ways along one axis must span the field
    // between them. That identity involves the difference of their two origins,
    // which depends on the heading and the mount offsets but NOT on where the
    // robot is -- so a pair that spans correctly is confirmed without ever
    // consulting the estimate it is about to correct. It is the only check here
    // a wrong pose cannot talk its way through.
    const float span = 2.0f * Field::FIELD_HALF_WALL;

    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        auto& a = mSensors[i];
        if (a.sensor == nullptr || !a.enabled) continue;
        if (a.cls != ReadingClass::WallHit && a.cls != ReadingClass::TooFar &&
            a.cls != ReadingClass::Blocked) continue;

        for (int j = i + 1; j < MAX_DISTANCE_SENSORS; j++) {
            auto& b = mSensors[j];
            if (b.sensor == nullptr || !b.enabled) continue;
            if (b.cls != ReadingClass::WallHit && b.cls != ReadingClass::TooFar &&
                b.cls != ReadingClass::Blocked) continue;

            const float dot = a.rayCos * b.rayCos + a.raySin * b.raySin;
            if (dot > -0.98f) continue;

            float measured;
            if (std::fabs(a.rayCos) > 0.7f) {
                measured = a.readingInch * a.rayCos - b.readingInch * b.rayCos
                         + (a.worldX - b.worldX);
            } else if (std::fabs(a.raySin) > 0.7f) {
                measured = a.readingInch * a.raySin - b.readingInch * b.raySin
                         + (a.worldY - b.worldY);
            } else {
                continue;
            }

            if (std::fabs(std::fabs(measured) - span) <= mConfig.pairSpanTolIn) {
                a.pairVerified = true;
                b.pairVerified = true;
            }
        }
    }
}

bool RclTracking::isValidReading(int index) const {
    return voteTier(index) >= 0;
}

std::pair<CoordType, float> RclTracking::getBotCoordFromSensor(int index) const {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return {CoordType::INVALID, 0.0f};
    const auto& s = mSensors[index];
    if (!isValidReading(index)) return {CoordType::INVALID, 0.0f};

    float cosA = s.rayCos;
    float sinA = s.raySin;
    float reading = s.readingInch;

    int wall = nearestWall(s).first;
    if (wall < 0) return {CoordType::INVALID, 0.0f};

    float result;
    CoordType type;

    if (wall == 1) {
        type = CoordType::Y;
        result = Field::FIELD_HALF_WALL - sinA * reading;
    } else if (wall == 2) {
        type = CoordType::X;
        result = Field::FIELD_HALF_WALL - cosA * reading;
    } else if (wall == 3) {
        type = CoordType::Y;
        result = -Field::FIELD_HALF_WALL - sinA * reading;
    } else {
        type = CoordType::X;
        result = -Field::FIELD_HALF_WALL - cosA * reading;
    }

    // Back out the mount offset in the SAME frame the geometry above was built
    // in: the one updateSensorPoses rotated at sample time, not whatever the
    // IMU says now.
    if (type == CoordType::X) result -= s.mountWorldX;
    else                      result -= s.mountWorldY;

    return {type, result};
}

RclTracking::AxisEstimate RclTracking::solveAxes(bool* keptOut) const {
    if (keptOut != nullptr) {
        for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) keptOut[i] = false;
    }

    float xVal[MAX_DISTANCE_SENSORS], yVal[MAX_DISTANCE_SENSORS];
    float xW[MAX_DISTANCE_SENSORS], yW[MAX_DISTANCE_SENSORS];
    int xIdx[MAX_DISTANCE_SENSORS], yIdx[MAX_DISTANCE_SENSORS];
    int xTier[MAX_DISTANCE_SENSORS], yTier[MAX_DISTANCE_SENSORS];
    int nx = 0, ny = 0;
    int xBest = -1, yBest = -1;

    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        const auto& s = mSensors[i];
        if (s.sensor == nullptr) continue;
        const int tier = voteTier(i);
        if (tier < 0) continue;
        auto [type, coord] = getBotCoordFromSensor(i);
        const float w = readingWeight(s.trust, s.readingInch, mConfig.rangeWeighting);
        if (type == CoordType::X) {
            xVal[nx] = coord; xW[nx] = w; xIdx[nx] = i; xTier[nx] = tier; nx++;
            if (tier > xBest) xBest = tier;
        } else if (type == CoordType::Y) {
            yVal[ny] = coord; yW[ny] = w; yIdx[ny] = i; yTier[ny] = tier; ny++;
            if (tier > yBest) yBest = tier;
        }
    }

    // Best evidence only, then a median filter inside it.
    //
    // The tiers matter more than the median does. Averaging a pair-verified
    // reading with an unverified one dilutes the only measurement on the robot
    // that does not depend on the pose being corrected; averaging a clean wall
    // hit with one the innovation gate rejected is worse still, because during
    // a recovery the rejected one is usually a beam ranging a post.
    //
    // Within a tier, median-then-drop rather than a plain mean: one beam that
    // clipped a robot would otherwise pull the axis by half its error, and with
    // two sensors per axis the mean of a good reading and a blocked one looks
    // like a slightly-off pose rather than a rejected sample. Two readings that
    // disagree leave the axis with nothing, which is the honest answer -- one
    // of them is wrong and this cannot tell which.
    //
    // The survivors are averaged by weight (trust, and the sensor's rated
    // accuracy at its range), so a near beam outweighs a far one and a sensor
    // on its way out counts for less before it is dropped.
    auto reduce = [&](float* val, float* wt, int* idx, int* tier, int n, int best,
                      float& outVal, int& outCount, float& outSpread) {
        outCount = 0;
        outSpread = 0.0f;
        if (n <= 0 || best < 0) return;

        float keep[MAX_DISTANCE_SENSORS];
        float keepW[MAX_DISTANCE_SENSORS];
        int keepIdx[MAX_DISTANCE_SENSORS];
        int nk = 0;
        for (int k = 0; k < n; k++) {
            if (tier[k] != best) continue;
            keep[nk] = val[k]; keepW[nk] = wt[k]; keepIdx[nk] = idx[k]; nk++;
        }
        if (nk <= 0) return;

        if (nk > 1) {
            float scratch[MAX_DISTANCE_SENSORS];
            for (int k = 0; k < nk; k++) scratch[k] = keep[k];
            const float med = medianOf(scratch, nk);
            int w = 0;
            for (int k = 0; k < nk; k++) {
                if (std::fabs(keep[k] - med) > mConfig.axisAgreeIn) continue;
                keep[w] = keep[k]; keepW[w] = keepW[k]; keepIdx[w] = keepIdx[k]; w++;
            }
            nk = w;
        }
        if (nk <= 0) return;

        float lo = keep[0], hi = keep[0];
        for (int k = 0; k < nk; k++) {
            lo = std::fmin(lo, keep[k]);
            hi = std::fmax(hi, keep[k]);
            if (keptOut != nullptr) keptOut[keepIdx[k]] = true;
        }
        outVal = weightedMean(keep, keepW, nk);
        outCount = nk;
        outSpread = hi - lo;
    };

    AxisEstimate est;
    const Pose fallback = mOdom.getPose();
    est.x = fallback.x;
    est.y = fallback.y;
    reduce(xVal, xW, xIdx, xTier, nx, xBest, est.x, est.xCount, est.xSpread);
    reduce(yVal, yW, yIdx, yTier, ny, yBest, est.y, est.yCount, est.ySpread);
    est.xPaired = (xBest == 2 && est.xCount > 0);
    est.yPaired = (yBest == 2 && est.yCount > 0);

    // The solve ran against the pose at sample time; carry the answer forward
    // by whatever the robot did while the reading was in flight, so what comes
    // out is an estimate of where it is NOW and syncUpdate differences like
    // with like.
    if (est.xCount > 0) est.x += mLagShiftX;
    if (est.yCount > 0) est.y += mLagShiftY;

    return est;
}

RclTracking::AxisEstimate RclTracking::estimateAxes() const {
    return solveAxes(nullptr);
}

std::pair<Pose, int> RclTracking::computePose() const {
    AxisEstimate est = estimateAxes();
    return {
        {est.x, est.y, mOdom.getPose().theta},
        est.xCount + est.yCount
    };
}

void RclTracking::expireDisableTimersLocked(std::uint32_t nowMs) {
    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        if (mDisableUntilMs[i] != 0 &&
            static_cast<std::int32_t>(nowMs - mDisableUntilMs[i]) >= 0) {
            mSensors[i].enabled = true;
            mDisableUntilMs[i] = 0;
        }
    }
}

void RclTracking::expireObstaclesLocked(std::uint32_t nowMs) {
    mLineObstacles.erase(
        std::remove_if(mLineObstacles.begin(), mLineObstacles.end(),
                       [nowMs](const LineObstacle& o) { return expired(o.expiresAtMs, nowMs); }),
        mLineObstacles.end());
    mCircleObstacles.erase(
        std::remove_if(mCircleObstacles.begin(), mCircleObstacles.end(),
                       [nowMs](const CircleObstacle& o) { return expired(o.expiresAtMs, nowMs); }),
        mCircleObstacles.end());
}

void RclTracking::update() {
    RawSamples raw;
    pollDevices(raw);
    std::lock_guard<pros::Mutex> lock(mMutex);

    // Measured, not nominal: the loop can overrun, and velocity, the stale
    // hold and the sync rate limit are all per-second quantities. A stall past
    // 10 periods, or a clock that went backwards, is a gap: no velocity is
    // differenced across it.
    const std::uint32_t now = pros::millis();
    const std::int32_t elapsed = static_cast<std::int32_t>(now - mLastUpdateMs);
    const bool gap = !mHaveLastUpdate || elapsed < 0 ||
                     elapsed > static_cast<std::int32_t>(10.0f * mConfig.updatePeriodMs);
    mDtMs = gap ? mConfig.updatePeriodMs : std::fmax(static_cast<float>(elapsed), 1.0f);
    mLastUpdateMs = now;
    mHaveLastUpdate = true;

    expireFarSideLocked(now);
    expireObstaclesLocked(now);

    const Pose currentPose = mOdom.getPose();
    const float dtSec = mDtMs / 1000.0f;

    // Motion, from successive poses, MINUS whatever the last sync put there.
    // Odometry moves for two reasons -- the robot drove, and this filter
    // corrected it -- and only the first is velocity. During a recovery the
    // correction is up to maxSyncDist every 10 ms, which reads as 500 in/s and
    // feeds the latency compensation a 15 in shift built entirely out of the
    // filter watching itself work.
    //
    // Low-passed hard on top of that: a 10 ms difference of a pose quantised to
    // encoder ticks is mostly quantisation.
    if (mHavePrevPose && !gap) {
        const float k = 0.25f;
        const float dxDriven = (currentPose.x - mPrevPose.x) - mAppliedDx;
        const float dyDriven = (currentPose.y - mPrevPose.y) - mAppliedDy;
        mVelX += k * (dxDriven / dtSec - mVelX);
        mVelY += k * (dyDriven / dtSec - mVelY);
        const float omega = angleDiffRad(currentPose.theta, mPrevPose.theta) / dtSec;
        mTurnRate += k * (std::fabs(omega) - mTurnRate);
    }
    mPrevPose = currentPose;
    mHavePrevPose = true;
    mAppliedDx = mAppliedDy = 0.0f;

    mTurnHeld = mConfig.maxTurnRateRadS > 0.0f && mTurnRate > mConfig.maxTurnRateRadS;

    // Roll the pose back to when the light actually left, solve there, and let
    // solveAxes carry the answer forward by the same amount.
    const float lagSec = mConfig.sensorLatencyMs / 1000.0f;
    Pose samplePose = currentPose;
    if (lagSec > 0.0f) {
        samplePose.x -= mVelX * lagSec;
        samplePose.y -= mVelY * lagSec;
    }
    mLagShiftX = currentPose.x - samplePose.x;
    mLagShiftY = currentPose.y - samplePose.y;

    if (!ingestSamples(raw)) {
        // Nothing new arrived. Every reported number below -- the classes, the
        // vote count, the spread -- belongs to the last cycle that had real
        // data, and recomputing them from a held sample would say the same
        // thing while letting that one sample vote a second and third time.
        // Leaving them alone is both the honest answer and the stable one: the
        // diagnostics stop flickering between a verdict and "no new sample" two
        // frames in three.
        mLastSyncStep = 0.0f;
        return;
    }

    updateSensorPoses(samplePose);
    updateTrustLocked(now);

    // Displacement evidence. Only cycles that saw new samples are counted, so
    // the recovery timers are measured in readings rather than in loop ticks.
    int evidence = 0;
    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        const auto& s = mSensors[i];
        if (s.sensor == nullptr || !s.enabled) continue;
        // A sensor distrusted for longer than it takes to declare a
        // displacement has something parked in front of it; its disagreement is
        // about that, not the pose. The onset still counts for displaceCycles
        // of its samples: one gross miss is also what a pose running away from
        // a stalled body looks like. Trust keeps the reading itself out of
        // every vote, recovery included, so the most a parked robot can cause
        // is one recovery that has nothing from that sensor to move toward.
        // Only a short reading can be an obstruction: a sensor distrusted for
        // reading long is the pose disagreeing with a wall, and still testifies.
        if (s.distrustSamples > mConfig.displaceCycles &&
            s.cls != ReadingClass::TooFar) continue;
        // Scale with how badly the reading misses. A beam 9 in out on an 8 in
        // gate is noise at the edge of a threshold; one 125 in out is the pose
        // being somewhere else entirely, and waiting for a second sensor to
        // agree before admitting that wastes the only cycles there are.
        const float gross = 4.0f * mConfig.maxWallInnovation;
        const bool huge = std::fabs(s.innovationIn) > gross;

        switch (s.cls) {
            case ReadingClass::TooFar:     evidence += 2; break;
            // A pose that has run PAST the truth reads long; one that has
            // fallen short reads short. Scoring only the first meant a robot
            // wedged against a post -- drive wheels turning, body not moving,
            // odom running ahead of it -- recovered, but the same robot pushed
            // backwards never did.
            case ReadingClass::Blocked:    evidence += huge ? 2 : 1; break;
            case ReadingClass::Impossible: evidence += 1; break;
            default: break;
        }
    }

    // One sensor disagreeing by a moderate amount, however long it keeps
    // saying so, is NOT evidence of a displacement. A beam reading 15 in short
    // of the wall is a robot parked in front of it just as readily as it is a
    // 15 in pose error, and no amount of repetition separates the two: the
    // parked robot goes on reading the same 15 in. Treating persistence as
    // proof was measured relocating the pose 15.7 in onto an alliance partner
    // standing against a wall -- the filter driving itself into the exact
    // failure the innovation gate exists to prevent.
    //
    // So the bar stays at two: either two sensors disagree at once, or one
    // disagrees grossly (see the magnitude weighting above), which no
    // obstruction inside the field can produce. Everything short of that holds
    // the pose and drops the reading, and the honest fixes for a genuine small
    // error are the other axis, or a beam reset that scores against the map.
    if (evidence >= 2) {
        mDisplaceCycles++;
        if (!mDisplaced && mDisplaceCycles >= mConfig.displaceCycles) {
            mDisplaced = true;
            // Clean cycles counted before the onset would otherwise clear the
            // displacement in this same call.
            mCleanCycles = 0;
        }
    } else {
        mDisplaceCycles = 0;
    }

    // Leaving a recovery is decided by convergence, never by the readings
    // agreeing again. Two reasons, and both of them were measured:
    //
    // Agreement is not recovery. nearestWall() reads the walls from a position
    // clamped into the field, so a pose that has run outside it predicts a wall
    // about an inch away and then agrees with any sensor that really is near
    // one -- evidence falls to zero while the pose is still most of a field
    // out, and the recovery ends before it has done anything.
    //
    // And disagreement is not proof it is still needed. A recovery suspends the
    // post-occlusion test, so the beams that really are ranging posts start
    // reading short, which scores as evidence, which holds the recovery open,
    // which keeps the test suspended. That loop never exits on its own: the
    // pose came back to 0.4 in and the filter was still calling itself lost.
    //
    // Odometry having caught up with what this filter believes is the whole
    // definition of the job being done, and it cannot latch.
    const float dx = mLatestEstimate.x - currentPose.x;
    const float dy = mLatestEstimate.y - currentPose.y;
    if (std::hypot(dx, dy) < mConfig.maxSyncDist) {
        mCleanCycles++;
        // Hysteresis. Leaving on a single clean reading made the filter flap
        // between a 0.25 and a 0.9 sync proportion, which reads as the pose
        // twitching rather than converging.
        if (mDisplaced && mCleanCycles >= mConfig.recoverCycles) {
            mDisplaced = false;
            mDisplaceCycles = 0;
        }
    } else {
        mCleanCycles = 0;
    }

    bool kept[MAX_DISTANCE_SENSORS];
    AxisEstimate est = solveAxes(kept);
    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        auto& s = mSensors[i];
        s.outlier = !kept[i] && isValidReading(i) &&
                    getBotCoordFromSensor(i).first != CoordType::INVALID;
    }

    mXMeasured = est.xCount > 0;
    mYMeasured = est.yCount > 0;
    int sensorCount = est.xCount + est.yCount;
    mActiveSensorCount = sensorCount;
    mSpreadIn = std::fmax(est.xSpread, est.ySpread);

    if (sensorCount > 0) {
        if (mConfig.useAccumulation) {
            float alpha = mConfig.accumulationAlpha;
            if (mXMeasured) {
                mResidualX = alpha * (est.x - currentPose.x) + (1.0f - alpha) * mResidualX;
            }
            if (mYMeasured) {
                mResidualY = alpha * (est.y - currentPose.y) + (1.0f - alpha) * mResidualY;
            }
            mLatestEstimate.x = currentPose.x + (mXMeasured ? mResidualX : 0.0f);
            mLatestEstimate.y = currentPose.y + (mYMeasured ? mResidualY : 0.0f);
        } else {
            mLatestEstimate.x = mXMeasured ? est.x : currentPose.x;
            mLatestEstimate.y = mYMeasured ? est.y : currentPose.y;
        }
        mLatestEstimate.theta = currentPose.theta;
    }

    // While recovering, an axis may only move the pose if enough sensors agree
    // on it. The snap is hard and unbounded, and the readings feeding it are
    // ones the innovation gate had already rejected -- one of those on its own
    // is as likely to be a robot parked in front of the sensor as a wall.
    if (mDisplaced) {
        if (est.xCount < mConfig.recoveryMinVotes && !est.xPaired) mXMeasured = false;
        if (est.yCount < mConfig.recoveryMinVotes && !est.yPaired) mYMeasured = false;
    }

    const bool ownsPose =
        mConfig.autoSync || (mDisplaced && mConfig.syncWhileDisplaced);
    if (ownsPose && (mXMeasured || mYMeasured)) {
        syncUpdateLocked();
    } else {
        mLastSyncStep = 0.0f;
    }
}

void RclTracking::syncUpdate() {
    std::lock_guard<pros::Mutex> lock(mMutex);
    syncUpdateLocked();
}

void RclTracking::syncUpdateLocked() {
    if (!mXMeasured && !mYMeasured) { mLastSyncStep = 0.0f; return; }

    Pose odomPose = mOdom.getPose();

    const float prop = mConfig.hardSync ? 1.0f
                     : (mDisplaced ? mConfig.recoverySyncProp : mConfig.syncProp);

    const float rateStep = mConfig.maxSyncRate * (mDtMs / 1000.0f);
    // Recovery is fast but not instant. maxSyncDist per reading crosses the
    // whole field in well under a second, which is quick enough inside a 15 s
    // routine, while a single wrong reading can only move the pose that far
    // before the next one gets a say. Teleporting it -- which an unbounded step
    // does -- means one beam that caught a robot relocates the robot.
    const float maxStep = mConfig.hardSync
                              ? Field::FIELD_LENGTH
                              : (mDisplaced ? mConfig.maxSyncDist
                                            : std::min(mConfig.maxSyncDist, rateStep));

    mLastResidualX = mXMeasured ? (mLatestEstimate.x - odomPose.x) : 0.0f;
    mLastResidualY = mYMeasured ? (mLatestEstimate.y - odomPose.y) : 0.0f;

    float dx = mLastResidualX * prop;
    float dy = mLastResidualY * prop;
    float dist = std::hypot(dx, dy);

    if (dist > maxStep) {
        float scale = maxStep / dist;
        dx *= scale;
        dy *= scale;
        dist = maxStep;
    }
    // The per-reading caps bound one step, not a run of them: recovery at
    // maxSyncDist per reading is ~150 in/s, which is a teleport. The bucket
    // bounds the total over real time. hardSync is a deliberate snap.
    if (!mConfig.hardSync) dist = mCorrLimiter.limit(pros::millis(), dx, dy);
    mLastSyncStep = dist;
    // Remembered so the next cycle can tell this correction apart from the
    // robot actually moving.
    mAppliedDx += dx;
    mAppliedDy += dy;

    mOdom.syncPose({
        odomPose.x + dx,
        odomPose.y + dy,
        odomPose.theta
    });
}

bool RclTracking::updateBotPoseFromBestSensor() {
    // Called from a routine's task, which PROS deletes on a mode change, so
    // nothing here may hold mMutex across a device read or another lock: a
    // task deleted mid-section leaves the mutex taken and the loop dead.
    RawSamples raw;
    pollDevices(raw);
    const Pose odomPose = mOdom.getPose();

    CoordType type = CoordType::INVALID;
    float coord = 0.0f;
    {
        std::lock_guard<pros::Mutex> lock(mMutex);
        expireFarSideLocked(pros::millis());
        ingestSamples(raw);
        // Whatever the device last reported is the best that is available to
        // a caller asking for a fix right now; refusing it for being the same
        // value as last cycle would make this return false for a stationary
        // robot, which is the one case it is most often called in.
        const bool current = raw.gen == mSampleGen;
        for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
            auto& s = mSensors[i];
            s.fresh = current && s.sensor != nullptr && s.enabled && !s.faulted &&
                      raw.s[i].dev == s.sensor;
        }
        updateSensorPoses(odomPose);

        int bestIndex = -1;
        float bestDist = 999.0f;
        for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
            if (mSensors[i].sensor == nullptr) continue;
            if (isValidReading(i) && mSensors[i].readingInch < bestDist) {
                bestDist = mSensors[i].readingInch;
                bestIndex = i;
            }
        }
        if (bestIndex < 0) return false;

        std::tie(type, coord) = getBotCoordFromSensor(bestIndex);
        if (type == CoordType::INVALID) return false;
    }

    // Re-read so motion since the solve survives on the axis not measured.
    const Pose now = mOdom.getPose();
    const Pose fixed = type == CoordType::X ? Pose{coord, now.y, now.theta}
                                            : Pose{now.x, coord, now.theta};
    mOdom.syncPose(fixed);
    std::lock_guard<pros::Mutex> lock(mMutex);
    mLatestEstimate = fixed;
    // Not motion: keep it out of the next cycle's velocity.
    mAppliedDx += fixed.x - now.x;
    mAppliedDy += fixed.y - now.y;
    return true;
}

void RclTracking::hardReset(const Pose& pose) {
    std::lock_guard<pros::Mutex> lock(mMutex);

    mLatestEstimate = pose;

    const std::uint32_t now = pros::millis();
    if (mConfig.farSideGate && mFarSideOrientPending) {
        const std::int32_t sinceArm = static_cast<std::int32_t>(now - mFarSideArmedMs);
        if (sinceArm >= 0 && sinceArm <= static_cast<std::int32_t>(kFarSideOrientWindowMs)) {
            orientFarSideLocked(pose);
        }
    }
    mFarSideOrientPending = false;
    mLastHardResetMs = now;
    mHaveHardReset = true;
    mSampleGen++;

    mResidualX = mResidualY = 0.0f;
    mLastResidualX = mLastResidualY = 0.0f;
    mLastSyncStep = 0.0f;
    mSpreadIn = 0.0f;

    mDisplaced = false;
    mDisplaceCycles = 0;
    mCleanCycles = 0;

    mXMeasured = mYMeasured = false;
    mActiveSensorCount = 0;

    // Velocity history belongs to the pose that was just discarded, and a beam
    // reset is taken standing still anyway.
    mPrevPose = pose;
    mHavePrevPose = true;
    mVelX = mVelY = 0.0f;
    mAppliedDx = mAppliedDy = 0.0f;
    mTurnRate = 0.0f;
    mTurnHeld = false;
    mLagShiftX = mLagShiftY = 0.0f;
    mCorrLimiter.reset();

    // Every held sample was measured against the old pose. Letting them stand
    // would have the next cycle classify them against the new one and call the
    // disagreement evidence of a displacement.
    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        mSensors[i].rawMm = -1;
        mSensors[i].heldMs = 0.0f;
        mSensors[i].fresh = false;
        mSensors[i].valid = false;
        mSensors[i].pairVerified = false;
        mSensors[i].outlier = false;
        mSensors[i].cls = ReadingClass::Invalid;
        mSensors[i].innovationIn = 0.0f;
        // Distrust was disagreement with the pose just replaced.
        resetTrust(mSensors[i]);
    }
}

Pose RclTracking::latestEstimate() const {
    std::lock_guard<pros::Mutex> lock(mMutex);
    return mLatestEstimate;
}

void RclTracking::startTracking() {
    if (mTask == nullptr) {
        mRunning = true;
        mLatestEstimate = mOdom.getPose();
        mPrevPose = mLatestEstimate;
        mHavePrevPose = true;
        mHaveLastUpdate = false;
        mTask = new pros::Task([this]() { run(); });
    }
}

void RclTracking::stopTracking() {
    mRunning = false;
    if (mTask != nullptr) {
        uint32_t start = pros::millis();
        while (pros::millis() - start < 200) {
            uint32_t state = mTask->get_state();
            if (state == pros::E_TASK_STATE_DELETED ||
                state == pros::E_TASK_STATE_INVALID) break;
            pros::delay(5);
        }
        uint32_t state = mTask->get_state();
        if (state != pros::E_TASK_STATE_DELETED &&
            state != pros::E_TASK_STATE_INVALID) {
            mTask->remove();
        }
        delete mTask;
        mTask = nullptr;
    }
}

bool RclTracking::isTracking() const {
    return mRunning && mTask != nullptr;
}

void RclTracking::run() {
    while (mRunning) {
        uint32_t startTime = pros::millis();

        update();
        int32_t elapsed = pros::millis() - startTime;
        int32_t remaining = static_cast<int32_t>(mConfig.updatePeriodMs) - elapsed;
        pros::delay(remaining > 1 ? remaining : 1);
    }
}
}
