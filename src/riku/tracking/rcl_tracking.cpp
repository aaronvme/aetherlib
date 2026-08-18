// Ported from FBLib - https://github.com/jiazegao/FBLib
#include "riku/tracking/rcl_tracking.hpp"

#include <algorithm>
#include <cmath>

namespace riku {

// ============================================================================
// Ray-line intersection
// ============================================================================

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

// ============================================================================
// nearestWall()
// ============================================================================

std::pair<int, float> RclTracking::nearestWall(const RclSensor& s) const {
    // Distance to each of the 4 field walls; the closest positive one is the
    // wall the ray reaches first.
    float minDist = 1e9f;
    int wall = -1;  // 1=N(+Y), 2=E(+X), 3=S(-Y), 4=W(-X)

    // X-walls (vertical planes at x = ±FIELD_HALF_WALL)
    if (std::fabs(s.rayCos) > 1e-6f) {
        float dEast = (Field::FIELD_HALF_WALL - s.worldX) / s.rayCos;
        if (dEast > 0.0f && dEast < minDist) { minDist = dEast; wall = 2; }

        float dWest = (-Field::FIELD_HALF_WALL - s.worldX) / s.rayCos;
        if (dWest > 0.0f && dWest < minDist) { minDist = dWest; wall = 4; }
    }

    // Y-walls (horizontal planes at y = ±FIELD_HALF_WALL)
    if (std::fabs(s.raySin) > 1e-6f) {
        float dNorth = (Field::FIELD_HALF_WALL - s.worldY) / s.raySin;
        if (dNorth > 0.0f && dNorth < minDist) { minDist = dNorth; wall = 1; }

        float dSouth = (-Field::FIELD_HALF_WALL - s.worldY) / s.raySin;
        if (dSouth > 0.0f && dSouth < minDist) { minDist = dSouth; wall = 3; }
    }

    if (wall < 0) return {-1, 0.0f};
    return {wall, minDist};
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

RclTracking::RclTracking(OdomTracking& odom, const Config& config)
    : mConfig(config), mOdom(odom) {}

RclTracking::~RclTracking() {
    stopTracking();
}

// ============================================================================
// Sensor management
// ============================================================================

void RclTracking::setSensor(int index, pros::Distance* sensor, const Pose& mountOffset) {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return;
    mSensors[index].sensor = sensor;
    mSensors[index].mountOffset = mountOffset;
    mSensors[index].enabled = true;
}

void RclTracking::configureSensors(const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors,
                                     const std::array<Pose, MAX_DISTANCE_SENSORS>& mounts) {
    int activeCount = 0;
    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        if (sensors[i] != nullptr) {
            mSensors[i].sensor       = sensors[i];
            mSensors[i].mountOffset  = mounts[i];
            mSensors[i].enabled      = true;
            mDisableTimers[i]        = 0.0f;
            activeCount++;
        } else {
            mSensors[i].sensor       = nullptr;
            mSensors[i].mountOffset  = {};
            mSensors[i].enabled      = false;
            mDisableTimers[i]        = 0.0f;
        }
    }
    // Clamp to [1, MAX_DISTANCE_SENSORS] so loop bounds remain safe
    mConfig.sensorCount = (activeCount > 0) ? activeCount : 1;
}

void RclTracking::enableSensor(int index) {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return;
    mSensors[index].enabled = true;
}

void RclTracking::disableSensor(int index) {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return;
    mSensors[index].enabled = false;
}

void RclTracking::disableSensorFor(int index, float durationMs) {
    if (index < 0 || index >= MAX_DISTANCE_SENSORS) return;
    mSensors[index].enabled = false;
    mDisableTimers[index] = durationMs;
}

// ============================================================================
// Obstacle management
// ============================================================================

void RclTracking::addLineObstacle(float x1, float y1, float x2, float y2, float lifetimeMs) {
    mLineObstacles.push_back({x1, y1, x2, y2, lifetimeMs});
}

void RclTracking::addCircleObstacle(float x, float y, float radius, float lifetimeMs) {
    mCircleObstacles.push_back({x, y, radius, lifetimeMs});
}

void RclTracking::clearObstacles() {
    mLineObstacles.clear();
    mCircleObstacles.clear();
}

// ============================================================================
// updateSensorPoses()
// ============================================================================

void RclTracking::updateSensorPoses(const Pose& robotPose) {
    float rCos = std::cos(robotPose.theta);
    float rSin = std::sin(robotPose.theta);

    for (int i = 0; i < mConfig.sensorCount; i++) {
        auto& s = mSensors[i];
        if (!s.enabled || s.sensor == nullptr) {
            s.valid = false;
            continue;
        }

        // Rotate mount offset into world frame
        s.worldX = robotPose.x + (s.mountOffset.x * rCos - s.mountOffset.y * rSin);
        s.worldY = robotPose.y + (s.mountOffset.x * rSin + s.mountOffset.y * rCos);

        // Sensor pointing direction in world frame
        float sensorWorldAngle = robotPose.theta + s.mountOffset.theta;
        s.worldHeading = sensorWorldAngle;
        s.rayCos = std::cos(sensorWorldAngle);
        s.raySin = std::sin(sensorWorldAngle);

        // Read sensor
        int readingMm = s.sensor->get();
        s.readingInch = readingMm * MM_TO_INCH;
        s.confidence = s.sensor->get_confidence();

        // Publish validity; without this the flag is never set true.
        s.valid = isValidReading(i);
    }
}

// ============================================================================
// isValidReading()
// ============================================================================

bool RclTracking::isValidReading(int index) const {
    float wallDist = 0.0f;
    ReadingClass c = classifyReading(index, wallDist);
    // A TooFar reading is a real wall measurement from somewhere other than where
    // odometry thinks the robot is: useless as a correction to a pose assumed
    // good, and exactly what is needed once that assumption is dropped.
    // Once the pose is known to be wrong, "that reading is too short to be the
    // wall" is an inference drawn from a pose we no longer believe. Every
    // well-formed reading becomes a candidate wall measurement again — which is
    // what lets an axis recover when its only beam reads short.
    if (c == ReadingClass::WallHit) return true;
    return mDisplaced && (c == ReadingClass::TooFar || c == ReadingClass::Blocked);
}

RclTracking::ReadingClass RclTracking::classifyReading(int index, float& wallDist) const {
    wallDist = 0.0f;
    const auto& s = mSensors[index];
    if (!s.enabled || s.sensor == nullptr) return ReadingClass::Invalid;

    // — Range checks —
    // V5 distance sensor practical maximum is ~2000 mm (~78.7 in). Readings
    // above this are unreliable; above 9000 is the "no object" sentinel.
    // Use the stored reading from updateSensorPoses() to avoid a TOCTOU race
    // where the sensor value changes between the range check here and the
    // coordinate computation in getBotCoordFromSensor().
    float readingInch = s.readingInch;
    if (readingInch < 1.0f * MM_TO_INCH) return ReadingClass::Invalid;

    if (readingInch > 2000.0f * MM_TO_INCH) {
        // Silence where the pose predicts a near wall is as solid a contradiction
        // as reading through one. It fixes no coordinate, but it is what tells an
        // axis whose beams have all fallen silent that the robot has moved.
        float range = 0.0f;
        auto [w, d] = nearestWall(s);
        range = d;
        if (w >= 0 && range < mConfig.noReturnWallLimit) return ReadingClass::Impossible;
        return ReadingClass::Invalid;
    }

    // — Confidence check —
    // V5 sensor confidence is 0–63. Low-confidence readings produce noise.
    if (readingInch > 200.0f * MM_TO_INCH && s.confidence < mConfig.confidenceThreshold) {
        return ReadingClass::Invalid;
    }

    // — Heading alignment check —
    // RCL requires the sensor ray to be near-orthogonal to a field wall
    // (within angleTolerance of 0°, 90°, 180°, or 270° in world frame).
    // A diagonal ray cannot determine which wall was hit and the distance
    // measurement has amplified angle error.
    float angleTolRad = degToRad(mConfig.angleTolerance);
    float angleMod = std::fmod(std::fabs(s.worldHeading), HALF_PI);
    if (angleMod > angleTolRad && angleMod < HALF_PI - angleTolRad) {
        return ReadingClass::Invalid;
    }

    auto [wall, wallRange] = nearestWall(s);
    if (wall < 0) return ReadingClass::Invalid;
    wallDist = wallRange;

    // — Occlusion check —
    // A ray crossing an obstacle is measuring the obstacle, not the wall behind
    // it. Reject on any crossing before the wall: comparing the obstacle distance
    // against the reading misses the case that matters, since a sensor seeing the
    // obstacle reports the obstacle's own distance.
    for (const auto& obs : mLineObstacles) {
        float dist = intersectLineSegment(s.worldX, s.worldY, s.rayCos, s.raySin,
                                           obs.x1, obs.y1, obs.x2, obs.y2, wallRange);
        if (dist < wallRange) return ReadingClass::Invalid;
    }

    // Cone test, matching MCL and the EKF: a post owns the return either because
    // the reading sits on its surface or because it stands in front of the wall.
    for (const auto& obs : mCircleObstacles) {
        const float ox = obs.x - s.worldX;
        const float oy = obs.y - s.worldY;
        const float along = ox * s.rayCos + oy * s.raySin;
        if (along < 0.0f) continue;
        const float perp = std::fabs(-ox * s.raySin + oy * s.rayCos);
        if (perp > obs.radius + along * mConfig.obstacleConeFrac) continue;

        const float postRange = std::sqrt(ox * ox + oy * oy) - obs.radius;
        if (std::fabs(readingInch - postRange) <= mConfig.obstacleRangeTol) {
            return ReadingClass::Invalid;
        }
        if (postRange > 0.0f && postRange + mConfig.obstacleRangeTol < wallRange) {
            return ReadingClass::Invalid;
        }
    }

    // — Agreement check —
    // The reading has to be consistent with the wall it will be converted
    // against: 31 in against a wall predicted at 127 in is not that wall.
    float innovation = readingInch - wallRange;
    if (innovation > mConfig.maxWallInnovation) {
        // Further than the wall this pose puts in front of the beam. No object
        // can lengthen a reading and nothing is visible through a wall, so the
        // pose is wrong — the robot has been moved.
        return ReadingClass::TooFar;
    }
    if (innovation < -mConfig.maxWallInnovation) {
        // Nearer than predicted: something unmodelled is in the beam. Common,
        // and it carries no information about where the robot is — unless the
        // pose itself turns out to be wrong.
        return ReadingClass::Blocked;
    }

    return ReadingClass::WallHit;
}

// ============================================================================
// getBotCoordFromSensor()
// ============================================================================

std::pair<CoordType, float> RclTracking::getBotCoordFromSensor(int index) const {
    // ========================================================================
    // Derive robot coordinate from a single sensor intersecting a field wall.
    //
    // A distance sensor pointed at a wall determines ONE spatial dimension:
    //   • east/west wall → X coordinate
    //   • north/south wall → Y coordinate
    //
    // The unconstrained coordinate is NOT returned — it must come from
    // another sensor or from odometry.  This is physically necessary:
    // a single ray-to-wall distance cannot constrain the axis parallel
    // to the wall.
    // ========================================================================
    const auto& s = mSensors[index];
    if (!isValidReading(index)) return {CoordType::INVALID, 0.0f};

    float cosA = s.rayCos;
    float sinA = s.raySin;
    float reading = s.readingInch;

    int wall = nearestWall(s).first;  // 1=N(+Y), 2=E(+X), 3=S(-Y), 4=W(-X)
    if (wall < 0) return {CoordType::INVALID, 0.0f};

    // Derive coordinate from the wall that was hit.
    //   sensor_world = wall_coord - rayComponent * reading
    //   robot = sensor_world - sensorOffsetInWorldFrame
    float result;
    CoordType type;

    if (wall == 1) {       // NORTH — horizontal wall at y = +FIELD_HALF_WALL
        type = CoordType::Y;
        result = Field::FIELD_HALF_WALL - sinA * reading;
    } else if (wall == 2) { // EAST — vertical wall at x = +FIELD_HALF_WALL
        type = CoordType::X;
        result = Field::FIELD_HALF_WALL - cosA * reading;
    } else if (wall == 3) { // SOUTH — horizontal wall at y = -FIELD_HALF_WALL
        type = CoordType::Y;
        result = -Field::FIELD_HALF_WALL - sinA * reading;
    } else {               // wall == 4, WEST — vertical wall at x = -FIELD_HALF_WALL
        type = CoordType::X;
        result = -Field::FIELD_HALF_WALL - cosA * reading;
    }

    // Subtract sensor mount offset from robot center
    float robotHeading = mOdom.getPose().theta;
    float rCos = std::cos(robotHeading);
    float rSin = std::sin(robotHeading);
    float offX = s.mountOffset.x * rCos - s.mountOffset.y * rSin;
    float offY = s.mountOffset.x * rSin + s.mountOffset.y * rCos;

    if (type == CoordType::X) result -= offX;
    else                      result -= offY;

    return {type, result};
}

// ============================================================================
// computePose()
// ============================================================================

RclTracking::AxisEstimate RclTracking::estimateAxes() const {
    // Independently average X and Y coordinates from different sensors.
    // A sensor pointed at an east/west wall constrains X; a sensor pointed
    // at a north/south wall constrains Y.  The two axes are independent —
    // a sensor that determines X says nothing about Y (and vice versa).
    float sumX = 0.0f, sumY = 0.0f;
    int xCount = 0, yCount = 0;

    for (int i = 0; i < mConfig.sensorCount; i++) {
        auto [type, coord] = getBotCoordFromSensor(i);
        if (type == CoordType::X) {
            sumX += coord;
            xCount++;
        } else if (type == CoordType::Y) {
            sumY += coord;
            yCount++;
        }
    }

    AxisEstimate est;
    est.xCount = xCount;
    est.yCount = yCount;
    // When no sensor constrains an axis, fall back to odometry
    est.x = (xCount > 0) ? sumX / static_cast<float>(xCount) : mOdom.getPose().x;
    est.y = (yCount > 0) ? sumY / static_cast<float>(yCount) : mOdom.getPose().y;
    return est;
}

std::pair<Pose, int> RclTracking::computePose() const {
    AxisEstimate est = estimateAxes();
    return {
        {est.x, est.y, mOdom.getPose().theta},
        est.xCount + est.yCount
    };
}

// ============================================================================
// update()
// ============================================================================

void RclTracking::update() {
    // Update sensor disable timers
    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        if (mDisableTimers[i] > 0.0f) {
            mDisableTimers[i] -= mConfig.updatePeriodMs;
            if (mDisableTimers[i] <= 0.0f) {
                mSensors[i].enabled = true;
                mDisableTimers[i] = 0.0f;
            }
        }
    }

    Pose currentPose = mOdom.getPose();
    updateSensorPoses(currentPose);

    // Displacement detection.
    //
    // An object can only shorten a reading, so one LONGER than the predicted wall
    // cannot happen if the pose is right -- proof the robot moved, not a hint.
    // Require persistence so a stray sample cannot unlock a hard correction.
    // Weighted, not counted: reading through a wall is conclusive alone, silence
    // is weaker (a glancing beam drops out legitimately) so it takes two.
    int evidence = 0;
    for (int i = 0; i < mConfig.sensorCount; i++) {
        float wallDist = 0.0f;
        switch (classifyReading(i, wallDist)) {
            case ReadingClass::TooFar:     evidence += 2; break;
            case ReadingClass::Impossible: evidence += 1; break;
            default: break;
        }
    }
    mDisplaceCycles = (evidence >= 2) ? (mDisplaceCycles + 1) : 0;
    if (mDisplaceCycles >= mConfig.displaceCycles) {
        mDisplaced = true;
    } else if (evidence == 0) {
        mDisplaced = false;
    }

    AxisEstimate est = estimateAxes();
    mXMeasured = est.xCount > 0;
    mYMeasured = est.yCount > 0;
    int sensorCount = est.xCount + est.yCount;
    mActiveSensorCount = sensorCount;

    // Measured axes only, and smooth the residual rather than the coordinate: an
    // EMA over positions lags a moving robot, and sync writes that lag back.
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

    // Decrement temporary obstacle lifetimes and mark expired ones
    // Permanent obstacles (lifetimeMs == 0) are never removed.
    // Temporary obstacles (lifetimeMs > 0) count down each update;
    // when they cross 0 they are set to -1.0f to flag removal.
    for (auto& o : mLineObstacles) {
        if (o.lifetimeMs > 0.0f) {
            o.lifetimeMs -= mConfig.updatePeriodMs;
            if (o.lifetimeMs <= 0.0f) o.lifetimeMs = -1.0f;
        }
    }
    for (auto& o : mCircleObstacles) {
        if (o.lifetimeMs > 0.0f) {
            o.lifetimeMs -= mConfig.updatePeriodMs;
            if (o.lifetimeMs <= 0.0f) o.lifetimeMs = -1.0f;
        }
    }

    // Remove obstacles flagged for deletion
    mLineObstacles.erase(
        std::remove_if(mLineObstacles.begin(), mLineObstacles.end(),
                       [](const LineObstacle& o) { return o.lifetimeMs < 0.0f; }),
        mLineObstacles.end());
    mCircleObstacles.erase(
        std::remove_if(mCircleObstacles.begin(), mCircleObstacles.end(),
                       [](const CircleObstacle& o) { return o.lifetimeMs < 0.0f; }),
        mCircleObstacles.end());

    if (mConfig.autoSync && sensorCount >= 1) {
        syncUpdate();
    }
}

// ============================================================================
// syncUpdate()
// ============================================================================

void RclTracking::syncUpdate() {
    if (mActiveSensorCount == 0) return;

    Pose odomPose = mOdom.getPose();

    // Measured axes only, and a fraction of the error: applying all of it every
    // cycle turns one bad reading into a pose teleport.
    // Once the pose is disproved this is not refining an estimate, it is replacing
    // a discredited one with a direct wall measurement.
    const float prop = mConfig.hardSync ? 1.0f
                     : (mDisplaced ? mConfig.recoverySyncProp : mConfig.syncProp);
    // Rate-limited so whatever consumes the pose sees it continuous; a proven
    // displacement is exempt, and so is hardSync.
    const float rateStep = mConfig.maxSyncRate * (mConfig.updatePeriodMs / 1000.0f);
    const float maxStep = (mDisplaced || mConfig.hardSync)
                              ? Field::FIELD_LENGTH
                              : std::min(mConfig.maxSyncDist, rateStep);

    float dx = mXMeasured ? (mLatestEstimate.x - odomPose.x) * prop : 0.0f;
    float dy = mYMeasured ? (mLatestEstimate.y - odomPose.y) * prop : 0.0f;
    float dist = std::hypot(dx, dy);

    if (dist > maxStep) {
        float scale = maxStep / dist;
        dx *= scale;
        dy *= scale;
    }

    mOdom.setPose({
        odomPose.x + dx,
        odomPose.y + dy,
        odomPose.theta
    });
}

// ============================================================================
// updateBotPoseFromBestSensor()
// ============================================================================

bool RclTracking::updateBotPoseFromBestSensor() {
    updateSensorPoses(mOdom.getPose());

    int bestIndex = -1;
    float bestDist = 999.0f;

    for (int i = 0; i < mConfig.sensorCount; i++) {
        if (isValidReading(i)) {
            float dist = mSensors[i].readingInch;
            if (dist < bestDist) {
                bestDist = dist;
                bestIndex = i;
            }
        }
    }

    if (bestIndex < 0) return false;

    auto [type, coord] = getBotCoordFromSensor(bestIndex);
    if (type == CoordType::INVALID) return false;

    // Only update the coordinate that this sensor constrains.
    // The other coordinate stays at the odometry value.
    Pose odomPose = mOdom.getPose();
    if (type == CoordType::X) {
        mOdom.setPose({coord, odomPose.y, odomPose.theta});
    } else {
        mOdom.setPose({odomPose.x, coord, odomPose.theta});
    }
    mLatestEstimate = mOdom.getPose();
    return true;
}

// ============================================================================
// Lifecycle
// ============================================================================

void RclTracking::startTracking() {
    if (mTask == nullptr) {
        mRunning = true;
        mLatestEstimate = mOdom.getPose();
        mTask = new pros::Task([this]() { run(); });
    }
}

void RclTracking::stopTracking() {
    mRunning = false;
    if (mTask != nullptr) {
        // Wait for the task to exit on its own (it self-deletes when run()
        // returns). Hard-killing with remove() while the task holds the
        // odometry mutex (inside getPose/setPose) would deadlock every other
        // odometry consumer permanently — only remove() as a last resort.
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
        // Odometry freshness is provided by the background odometry task inside
        // OdomTracking (the sole caller of mOdom.update()). Calling update()
        // here too would race on odometry state and consume delta baselines.
        update();
        int32_t elapsed = pros::millis() - startTime;
        int32_t remaining = static_cast<int32_t>(mConfig.updatePeriodMs) - elapsed;
        pros::delay(remaining > 1 ? remaining : 1);
    }
}

}  // namespace riku
