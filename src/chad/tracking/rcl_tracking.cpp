#include "chad/tracking/rcl_tracking.hpp"

#include <algorithm>
#include <cmath>

namespace chad {
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

    if (std::fabs(s.rayCos) > 1e-6f) {
        float dEast = (Field::FIELD_HALF_WALL - s.worldX) / s.rayCos;
        if (dEast > 0.0f && dEast < minDist) { minDist = dEast; wall = 2; }

        float dWest = (-Field::FIELD_HALF_WALL - s.worldX) / s.rayCos;
        if (dWest > 0.0f && dWest < minDist) { minDist = dWest; wall = 4; }
    }

    if (std::fabs(s.raySin) > 1e-6f) {
        float dNorth = (Field::FIELD_HALF_WALL - s.worldY) / s.raySin;
        if (dNorth > 0.0f && dNorth < minDist) { minDist = dNorth; wall = 1; }

        float dSouth = (-Field::FIELD_HALF_WALL - s.worldY) / s.raySin;
        if (dSouth > 0.0f && dSouth < minDist) { minDist = dSouth; wall = 3; }
    }

    if (wall < 0) return {-1, 0.0f};
    return {wall, minDist};
}

RclTracking::RclTracking(OdomTracking& odom, const Config& config)
    : mConfig(config), mOdom(odom) {}

RclTracking::~RclTracking() {
    stopTracking();
}

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

void RclTracking::updateSensorPoses(const Pose& robotPose) {
    float rCos = std::cos(robotPose.theta);
    float rSin = std::sin(robotPose.theta);

    for (int i = 0; i < mConfig.sensorCount; i++) {
        auto& s = mSensors[i];
        if (!s.enabled || s.sensor == nullptr) {
            s.valid = false;
            continue;
        }

        s.worldX = robotPose.x + (s.mountOffset.x * rCos - s.mountOffset.y * rSin);
        s.worldY = robotPose.y + (s.mountOffset.x * rSin + s.mountOffset.y * rCos);

        float sensorWorldAngle = robotPose.theta + s.mountOffset.theta;
        s.worldHeading = sensorWorldAngle;
        s.rayCos = std::cos(sensorWorldAngle);
        s.raySin = std::sin(sensorWorldAngle);

        int readingMm = s.sensor->get();
        s.readingInch = readingMm * MM_TO_INCH;
        s.confidence = s.sensor->get_confidence();

        s.valid = isValidReading(i);
    }
}

bool RclTracking::isValidReading(int index) const {
    float wallDist = 0.0f;
    ReadingClass c = classifyReading(index, wallDist);

    if (c == ReadingClass::WallHit) return true;
    return mDisplaced && (c == ReadingClass::TooFar || c == ReadingClass::Blocked);
}

RclTracking::ReadingClass RclTracking::classifyReading(int index, float& wallDist) const {
    wallDist = 0.0f;
    const auto& s = mSensors[index];
    if (!s.enabled || s.sensor == nullptr) return ReadingClass::Invalid;

    float readingInch = s.readingInch;
    if (readingInch < 1.0f * MM_TO_INCH) return ReadingClass::Invalid;

    if (readingInch > 2000.0f * MM_TO_INCH) {
        float range = 0.0f;
        auto [w, d] = nearestWall(s);
        range = d;
        if (w >= 0 && range < mConfig.noReturnWallLimit) return ReadingClass::Impossible;
        return ReadingClass::Invalid;
    }

    if (readingInch > 200.0f * MM_TO_INCH && s.confidence < mConfig.confidenceThreshold) {
        return ReadingClass::Invalid;
    }

    float angleTolRad = degToRad(mConfig.angleTolerance);
    float angleMod = std::fmod(std::fabs(s.worldHeading), HALF_PI);
    if (angleMod > angleTolRad && angleMod < HALF_PI - angleTolRad) {
        return ReadingClass::Invalid;
    }

    auto [wall, wallRange] = nearestWall(s);
    if (wall < 0) return ReadingClass::Invalid;
    wallDist = wallRange;

    for (const auto& obs : mLineObstacles) {
        float dist = intersectLineSegment(s.worldX, s.worldY, s.rayCos, s.raySin,
                                           obs.x1, obs.y1, obs.x2, obs.y2, wallRange);
        if (dist < wallRange) return ReadingClass::Invalid;
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
            return ReadingClass::Invalid;
        }
        if (postRange > 0.0f && postRange + mConfig.obstacleRangeTol < wallRange) {
            return ReadingClass::Invalid;
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

std::pair<CoordType, float> RclTracking::getBotCoordFromSensor(int index) const {
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

    float robotHeading = mOdom.getPose().theta;
    float rCos = std::cos(robotHeading);
    float rSin = std::sin(robotHeading);
    float offX = s.mountOffset.x * rCos - s.mountOffset.y * rSin;
    float offY = s.mountOffset.x * rSin + s.mountOffset.y * rCos;

    if (type == CoordType::X) result -= offX;
    else                      result -= offY;

    return {type, result};
}

RclTracking::AxisEstimate RclTracking::estimateAxes() const {
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

void RclTracking::update() {
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

void RclTracking::syncUpdate() {
    if (mActiveSensorCount == 0) return;

    Pose odomPose = mOdom.getPose();

    const float prop = mConfig.hardSync ? 1.0f
                     : (mDisplaced ? mConfig.recoverySyncProp : mConfig.syncProp);

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

    mOdom.syncPose({
        odomPose.x + dx,
        odomPose.y + dy,
        odomPose.theta
    });
}

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

    Pose odomPose = mOdom.getPose();
    if (type == CoordType::X) {
        mOdom.syncPose({coord, odomPose.y, odomPose.theta});
    } else {
        mOdom.syncPose({odomPose.x, coord, odomPose.theta});
    }
    mLatestEstimate = mOdom.getPose();
    return true;
}

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
