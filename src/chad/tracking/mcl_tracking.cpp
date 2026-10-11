#include "chad/tracking/mcl_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

#include "chad/drivetrain.hpp"
#include "chad/util/body_clearance.hpp"
#include "chad/util/fasttrig.hpp"
#include "pros/error.h"
#include "pros/llemu.hpp"

namespace chad {
float MclTracking::intersectLine(float rayX, float rayY,
                                 const LineObstacle& wall,
                                 float maxRange,
                                 float rayCos, float raySin) const {
    float xMin = std::min(wall.p1.x, wall.p2.x);
    float xMax = std::max(wall.p1.x, wall.p2.x);
    float yMin = std::min(wall.p1.y, wall.p2.y);
    float yMax = std::max(wall.p1.y, wall.p2.y);

    if ((rayCos > 0.0f && rayX > xMax) ||
        (rayCos < 0.0f && rayX < xMin) ||
        (raySin > 0.0f && rayY > yMax) ||
        (raySin < 0.0f && rayY < yMin)) {
        return maxRange;
    }

    float x1 = wall.p1.x, y1 = wall.p1.y;
    float x2 = wall.p2.x, y2 = wall.p2.y;
    float x3 = rayX,       y3 = rayY;
    float x4 = rayX + rayCos * maxRange;
    float y4 = rayY + raySin * maxRange;

    float den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::fabs(den) < 1e-6f) return maxRange;

    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
    float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;

    if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) {
        return u * maxRange;
    }
    return maxRange;
}

float MclTracking::intersectCircle(float rayX, float rayY,
                                    const CircleObstacle& circle,
                                    float maxRange,
                                    float dx, float dy) const {
    float xDiff = rayX - circle.x;
    float yDiff = rayY - circle.y;
    float cTemp = maxRange + circle.radius;
    if (xDiff * xDiff + yDiff * yDiff > cTemp * cTemp) {
        return maxRange;
    }

    float fx = rayX - circle.x;
    float fy = rayY - circle.y;
    float b = 2.0f * (fx * dx + fy * dy);
    float cVal = (fx * fx + fy * fy) - (circle.radius * circle.radius);
    float discriminant = b * b - 4.0f * cVal;

    if (discriminant < 0.0f) return maxRange;
    discriminant = std::sqrt(discriminant);

    float t1 = (-b - discriminant) * 0.5f;
    float t2 = (-b + discriminant) * 0.5f;

    if (t1 >= 0.0f && t1 <= maxRange) return t1;
    if (t2 >= 0.0f && t2 <= maxRange) return t2;
    return maxRange;
}

float MclTracking::nextNoise() {
    mNoiseIdx = (mNoiseIdx + 123) & NOISE_MASK;
    return mNoisePool[mNoiseIdx];
}

void MclTracking::regenerateNoise() {
    mNoisePool[mRegenIdx] = mDist(mGen);
    mRegenIdx = (mRegenIdx + 1) & NOISE_MASK;
}

void MclTracking::buildGaussianLUT() {
    const float step = (2.0f * LUT_SIGMA_SPAN) / GAUSSIAN_LUT_SIZE;
    for (int i = 0; i < GAUSSIAN_LUT_SIZE; i++) {
        const float u = -LUT_SIGMA_SPAN + (static_cast<float>(i) + 0.5f) * step;
        float p = mConfig.hitWeight * std::exp(-0.5f * u * u) + mConfig.randWeight;

        if (u < 0.0f) {
            p += mConfig.shortWeight *
                 std::exp(u / std::max(mConfig.shortDecaySigmas, 0.05f));
        }
        mGaussianLUT[i] = std::log(std::max(p, 1e-30f));
    }
}

float MclTracking::logLikelihood(float sigmas) const {
    const float t = (sigmas + LUT_SIGMA_SPAN) / (2.0f * LUT_SIGMA_SPAN);
    // Clamp before the cast: a NaN or huge t has no defined int value, and a
    // NaN lands in the tail like any hypothesis the reading cannot support.
    if (!(t > 0.0f)) return mGaussianLUT[0];
    if (!(t < 1.0f)) return mGaussianLUT[GAUSSIAN_LUT_SIZE - 1];
    return mGaussianLUT[static_cast<int>(t * GAUSSIAN_LUT_SIZE)];
}

MclTracking::MclTracking(OdomTracking& odom,
                         std::array<pros::Distance*, SENSOR_COUNT> distanceSensors,
                         const Pose& startPose,
                         const Config& config)
    : mConfig(config),
      mOdom(odom),
      mDistanceSensors(distanceSensors),
      mGen(std::random_device{}()),
      mDist(0.0f, 1.0f) {
    if (mConfig.particleCount > PARTICLE_MAX) {
        mConfig.particleCount = PARTICLE_MAX;
    }

    mSensorMounts.fill({});
    for (int i = 0; i < SENSOR_COUNT; i++) {
        mMountTrigs[i] = {1.0f, 0.0f};
    }

    for (int i = 0; i < NOISE_POOL_SIZE; i++) {
        mNoisePool[i] = mDist(mGen);
    }

    buildGaussianLUT();

    mLastImuHeading = mOdom.imuHeadingRad();

    setPose(startPose);
}

MclTracking::~MclTracking() {
    stopTracking();
}

void MclTracking::startTracking() {
    if (mTask == nullptr) {
        // Odometry has been accumulating since the constructor, which is
        // whole minutes of calibration and handling. Taking it now would
        // move every particle by all of it at once and read as a lurch the
        // filter then has to recover from. The EKF already drops it here.
        mOdom.consumeDelta();

        mRunning = true;
        mTask = new pros::Task([this]() { run(); });
    }
}

void MclTracking::stopTracking() {
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

bool MclTracking::isTracking() const {
    return mRunning && mTask != nullptr;
}

void MclTracking::run() {
    while (mRunning) {
        const uint32_t startTime = pros::millis();

        update();

        const int32_t elapsed = static_cast<int32_t>(pros::millis() - startTime);
        mLastCycleMs.store(static_cast<std::uint32_t>(elapsed < 0 ? 0 : elapsed));

        const int32_t period = static_cast<int32_t>(mConfig.updatePeriodMs);
        int32_t remaining = period - elapsed;
        if (remaining < 2) {
            // Over budget. A flat 2 ms here holds the filter at a ~95% duty
            // cycle and starves the control loop it exists to serve. Give
            // back as long as the cycle took: half rate, not the whole brain.
            remaining = (elapsed > period) ? std::min<int32_t>(elapsed, 100) : 2;
        }
        pros::delay(static_cast<uint32_t>(remaining));
    }
}

void MclTracking::setPose(const Pose& pose) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) return;
    mLastImuHeading = mOdom.imuHeadingRad();

    mOdom.consumeDelta();

    std::normal_distribution<float> xDist(pose.x, mConfig.distResampleVariance);
    std::normal_distribution<float> yDist(pose.y, mConfig.distResampleVariance);
    std::normal_distribution<float> tDist(pose.theta, mConfig.thetaResampleVariance);

    auto& particles = *mParticles;
    for (auto& p : particles) {
        p.pose = {xDist(mGen), yDist(mGen), tDist(mGen)};
        p.weight = 1.0f;
    }

    mRawEstimate = pose;
    mLastResamplePose = pose;
    mLatestSpeed = 0.0f;
    mHaveLogAvg = false;
    mInjectFrac = 0.0f;

    publishEstimate(computeEstimate());
}

void MclTracking::uniformReset() {
    mLastImuHeading = mOdom.imuHeadingRad();

    std::uniform_real_distribution<float> xDist(-Field::FIELD_HALF_WALL, Field::FIELD_HALF_WALL);
    std::uniform_real_distribution<float> yDist(-Field::FIELD_HALF_WALL, Field::FIELD_HALF_WALL);
    std::normal_distribution<float> tDist(0.0f, PI);

    auto& particles = *mParticles;
    for (auto& p : particles) {
        p.pose = {xDist(mGen), yDist(mGen), tDist(mGen)};
        p.weight = 1.0f;
    }

    mRawEstimate = Pose{};
    mLastResamplePose = Pose{};
    mLatestSpeed = 0.0f;
    mHaveLogAvg = false;
    mInjectFrac = 0.0f;

    publishEstimate(computeEstimate());
}

// The robot is at least as wide as its track, so its centre can never come
// within half a track width of a wall.
float MclTracking::reachLimit() const {
    const auto* dt = mOdom.sensors().drivetrain;
    return kW - ((dt != nullptr) ? dt->trackWidth * 0.5f : 0.0f);
}

void MclTracking::predict() {
    OdomDelta delta = mOdom.consumeDelta();
    if (!std::isfinite(delta.dVert) || !std::isfinite(delta.dHoriz) ||
        !std::isfinite(delta.dTheta)) delta = {};

    float currentImuHeadingRad = mOdom.imuHeadingRad();
    float dThetaRad = delta.dTheta;

    mRawEstimate.theta += dThetaRad;
    mRawEstimate.theta = wrapRad(mRawEstimate.theta);

    float halfDThetaRad = dThetaRad * 0.5f;
    float moveScale;
    if (std::fabs(dThetaRad) < 1e-6f) {
        moveScale = 1.0f;
    } else {
        moveScale = std::sin(halfDThetaRad) / halfDThetaRad;
    }

    float dVertPure  = delta.dVert  + mVertDrift;
    float dHorizPure = delta.dHoriz + mHorizDrift;

    mLatestSpeed = std::hypot(dVertPure, dHorizPure) * (1000.0f / mConfig.updatePeriodMs);
    mLatestTurnRate = std::fabs(dThetaRad) * (1000.0f / mConfig.updatePeriodMs);

    float driftVariance = std::hypot(mVertDrift, mHorizDrift) * 0.5f;

    bool hasVertWheel  = mOdom.hasVerticalTrackingWheel();
    bool hasHorizWheel = mOdom.hasHorizontalTracking();

    float vertVariance = hasVertWheel ? mConfig.trackingWheelVariance : mConfig.imeVariance;

    const float reach = reachLimit();
    auto& particles = *mParticles;
    for (int i = 0; i < mConfig.particleCount; i++) {
        auto& p = particles[i];

        float vertNoise = 1.0f + nextNoise() * vertVariance;
        float forwardDist = dVertPure * vertNoise + nextNoise() * driftVariance;

        float localHoriz;
        if (hasHorizWheel) {
            float horizNoise = 1.0f + nextNoise() * mConfig.trackingWheelVariance;
            float strafeDist = dHorizPure * horizNoise + nextNoise() * driftVariance;
            localHoriz = strafeDist * moveScale;
        } else {
            float movementNoise = nextNoise() * driftVariance
                                + nextNoise() * mConfig.horizDependentVarianceProp * std::fabs(dVertPure);
            localHoriz = movementNoise * moveScale
                       + nextNoise() * mConfig.horizConstantNoise;
        }

        float localVert = forwardDist * moveScale;

        p.pose.theta += halfDThetaRad + nextNoise() * mConfig.imuVariance;

        float pCos = FastTrig::cos(p.pose.theta);
        float pSin = FastTrig::sin(p.pose.theta);

        p.pose.x += localVert * pCos - localHoriz * pSin;
        p.pose.y += localVert * pSin + localHoriz * pCos;

        // Wheels that turn while a wall holds the robot still integrate
        // travel that never happened. Letting it push the cloud somewhere
        // the robot does not fit is what turns a scrape into a lost filter.
        p.pose.x = clamp(p.pose.x, -reach, reach);
        p.pose.y = clamp(p.pose.y, -reach, reach);

        {
            float rawTheta = p.pose.theta + halfDThetaRad;
            float diff = angleDiffRad(currentImuHeadingRad, rawTheta);
            float clampedDiff = clamp(diff, -mConfig.maxThetaDeviation, mConfig.maxThetaDeviation);
            p.pose.theta = wrapRad(currentImuHeadingRad + clampedDiff);
        }
    }

    mLastImuHeading = currentImuHeadingRad;
}

void MclTracking::updateWeights() {
    float activeSensors = 0.0f;

    float currCos = FastTrig::cos(mRawEstimate.theta);
    float currSin = FastTrig::sin(mRawEstimate.theta);

    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (mDistanceSensors[i] == nullptr) {
            mValidSensors[i] = false;
            continue;
        }

        mSensorReadingsMm[i] = mDistanceSensors[i]->get();
        mSensorReadingsInch[i] = mSensorReadingsMm[i] * MM_TO_INCH;
        mSensorConfs[i] = (mSensorReadingsMm[i] == PROS_ERR)
            ? PROS_ERR : mDistanceSensors[i]->get_confidence();
        // PROS_ERR is an unplugged or still-booting sensor, not a reading.
        mValidSensors[i] = mSensorReadingsMm[i] != PROS_ERR && mSensorConfs[i] != PROS_ERR;
        if (!mValidSensors[i]) continue;

        float sx = mRawEstimate.x + (mSensorMounts[i].x * currCos - mSensorMounts[i].y * currSin);
        float sy = mRawEstimate.y + (mSensorMounts[i].x * currSin + mSensorMounts[i].y * currCos);

        float sCos = currCos * mMountTrigs[i].cosVal - currSin * mMountTrigs[i].sinVal;
        float sSin = currSin * mMountTrigs[i].cosVal + currCos * mMountTrigs[i].sinVal;

        if (mDisabledSensors[i]) { mValidSensors[i] = false; continue; }
        if (mSensorReadingsMm[i] > 9000) { mValidSensors[i] = false; continue; }
        if (mSensorReadingsMm[i] < std::max(mConfig.minSensorRange, kMinV5RangeMm)) {
            mValidSensors[i] = false;
            continue;
        }

        if (mSensorReadingsMm[i] > 200 && mSensorConfs[i] < mConfig.confidenceThreshold) {
            mValidSensors[i] = false; continue;
        }

        // A reading sitting at the sensor ceiling means nothing was found
        // within range, not that a wall is standing at the ceiling. The
        // raycast is capped at the same distance, so a longer reading cannot
        // be matched to the map at all -- taken at face value it argues for
        // whatever pose does put a wall there.
        if (mSensorReadingsInch[i] > mConfig.maxSensorRange) {
            mValidSensors[i] = false; continue;
        }

        {
            bool disabled = false;
            for (const auto& line : kDisablingLines) {
                if (intersectLine(sx, sy, line, mConfig.maxSensorRange, sCos, sSin) < mConfig.maxSensorRange) {
                    disabled = true;
                    break;
                }
            }
            if (disabled) { mValidSensors[i] = false; continue; }
        }

        if (mLineObstacles != nullptr) {
            for (const auto& line : *mLineObstacles) {
                if (intersectLine(sx, sy, line, mConfig.maxSensorRange, sCos, sSin) < mConfig.maxSensorRange) {
                    mValidSensors[i] = false;
                    break;
                }
            }
        }
        if (!mValidSensors[i]) continue;

        activeSensors += 1.0f;
    }

    float invSigmas[SENSOR_COUNT];
    float sensorCountMult = (activeSensors > 0.0f) ? std::sqrt(activeSensors * 0.25f) : 1.0f;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (!mValidSensors[i]) continue;

        float angleOffset = std::fmod(std::fabs(mRawEstimate.theta + mSensorMounts[i].theta), HALF_PI);
        if (angleOffset > QUARTER_PI) {
            angleOffset = HALF_PI - angleOffset;
        }
        float angleMultiplier = 1.0f + angleOffset * (2.0f / PI * 1.2f);

        float sigma = (mSensorReadingsMm[i] <= 200)
            ? mConfig.rangeSigmaCloseIn
            : mSensorReadingsInch[i] * mConfig.rangeSigmaProp;

        sigma *= angleMultiplier;

        if (mSensorReadingsMm[i] > 200) {
            float safeConf = std::max(static_cast<float>(mSensorConfs[i]), 1.0f);
            sigma *= 32.0f / safeConf;
        }

        sigma *= sensorCountMult;
        invSigmas[i] = 1.0f / sigma;
    }

    auto& particles = *mParticles;
    mDeadzoneHits.fill(0);
    const float logDeadzone = std::log(std::max(mConfig.deadzoneLikelihood, 1e-30f));
    static constexpr float kLogFloor = -80.0f;
    int bodyBlocked = 0;

    for (int i = 0; i < mConfig.particleCount; i++) {
        float logWeight = 0.0f;
        auto& p = particles[i];

        if (std::fabs(p.pose.x) > Field::FIELD_HALF_WALL ||
            std::fabs(p.pose.y) > Field::FIELD_HALF_WALL) {
            p.logWeight = kLogFloor;
            continue;
        }
        // A body overlapping a goal post: a post-edge cone return can drag
        // the cloud into one while the pods say the robot barely moved.
        if (mCircleObstacles != nullptr &&
            bodyOverlapsCircle(p.pose.x, p.pose.y, mConfig.bodyRadiusIn,
                               *mCircleObstacles)) {
            p.logWeight = kLogFloor;
            bodyBlocked++;
            continue;
        }

        float pCos = FastTrig::cos(p.pose.theta);
        float pSin = FastTrig::sin(p.pose.theta);

        for (int j = 0; j < SENSOR_COUNT; j++) {
            if (!mValidSensors[j]) continue;

            float sCos = pCos * mMountTrigs[j].cosVal - pSin * mMountTrigs[j].sinVal;
            float sSin = pSin * mMountTrigs[j].cosVal + pCos * mMountTrigs[j].sinVal;

            float sx = p.pose.x + (mSensorMounts[j].x * pCos - mSensorMounts[j].y * pSin);
            float sy = p.pose.y + (mSensorMounts[j].x * pSin + mSensorMounts[j].y * pCos);

            if (mCircleObstacles != nullptr) {
                bool blocked = false;
                for (const auto& circle : *mCircleObstacles) {
                    const float ox = circle.x - sx;
                    const float oy = circle.y - sy;
                    const float along = ox * sCos + oy * sSin;
                    if (along < 0.0f) continue;
                    const float perp = std::fabs(-ox * sSin + oy * sCos);
                    if (perp > circle.radius + along * mConfig.obstacleConeFrac) continue;
                    const float postRange = std::sqrt(ox * ox + oy * oy) - circle.radius;
                    if (std::fabs(mSensorReadingsInch[j] - postRange) <= mConfig.obstacleRangeTol) {
                        blocked = true;
                        break;
                    }
                }
                if (blocked) {
                    mDeadzoneHits[j]++;
                    logWeight += logDeadzone;
                    continue;
                }
            }

            float pDist = mConfig.maxSensorRange;

            for (const auto& target : kFieldTargets) {
                float dist = intersectLine(sx, sy, target, mConfig.maxSensorRange, sCos, sSin);
                if (dist < pDist) pDist = dist;
            }

            if (pDist >= mConfig.maxSensorRange - 1e-6f) {
                for (const auto& circle : kFieldCircles) {
                    float dist = intersectCircle(sx, sy, circle, mConfig.maxSensorRange, sCos, sSin);
                    if (dist < pDist) pDist = dist;
                }
            }

            if (pDist >= mConfig.maxSensorRange - 1e-6f) {
                for (const auto& wall : kFieldWalls) {
                    float dist = intersectLine(sx, sy, wall, mConfig.maxSensorRange, sCos, sSin);
                    if (dist < pDist) pDist = dist;
                }
            }

            if (pDist >= mConfig.maxSensorRange - 1e-6f) {
                logWeight += logLikelihood(-LUT_SIGMA_SPAN);
                continue;
            }

            logWeight += logLikelihood((mSensorReadingsInch[j] - pDist) * invSigmas[j]);
        }
        // Written so a NaN takes the floor; std::max would keep it.
        p.logWeight = (logWeight > kLogFloor) ? logWeight : kLogFloor;
    }

    float maxLog = kLogFloor;
    for (int i = 0; i < mConfig.particleCount; i++) {
        maxLog = std::max(maxLog, particles[i].logWeight);
    }
    float sumRel = 0.0f;
    for (int i = 0; i < mConfig.particleCount; i++) {
        particles[i].weight = std::exp(particles[i].logWeight - maxLog);
        sumRel += particles[i].weight;
    }

    // Body-blocked particles say nothing about how well the beams fit; counting
    // them drags the slow average to the floor while a cloud sits against a
    // post, and the collapse that follows a real displacement goes unseen.
    const int possible = mConfig.particleCount - bodyBlocked;
    if (activeSensors > 0.0f && sumRel > 0.0f && possible > 0) {
        const float logWAvg =
            maxLog + std::log(sumRel / static_cast<float>(possible));
        if (!mHaveLogAvg) {
            mLogWSlow = mLogWFast = logWAvg;
            mHaveLogAvg = true;
        } else {
            mLogWSlow += mConfig.alphaSlow * (logWAvg - mLogWSlow);
            mLogWFast += mConfig.alphaFast * (logWAvg - mLogWFast);
        }
        const float ratio = std::exp(std::min(mLogWFast - mLogWSlow, 0.0f));
        mInjectFrac = clamp(1.0f - ratio, 0.0f, mConfig.maxRandomFrac);
    } else if (activeSensors > 0.0f && possible == 0) {
        // The whole cloud is inside a post: wrong by construction, so search.
        mInjectFrac = mConfig.maxRandomFrac;
    } else {
        mInjectFrac = 0.0f;
    }

    const float inv = 1.0f / static_cast<float>(mConfig.particleCount);
    for (int i = 0; i < SENSOR_COUNT; i++) mDeadzoneFrac[i] = mDeadzoneHits[i] * inv;

    // A reading the map cannot explain constrains nothing, so a deadzoned beam
    // counts for as little as a missing one.
    mObsX = 0.0f;
    mObsY = 0.0f;
    const float eCos = FastTrig::cos(mRawEstimate.theta);
    const float eSin = FastTrig::sin(mRawEstimate.theta);
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (!mValidSensors[i]) continue;
        const float w = 1.0f - clamp(mDeadzoneFrac[i], 0.0f, 1.0f);
        const float bCos = eCos * mMountTrigs[i].cosVal - eSin * mMountTrigs[i].sinVal;
        const float bSin = eSin * mMountTrigs[i].cosVal + eCos * mMountTrigs[i].sinVal;
        mObsX += w * std::fabs(bCos);
        mObsY += w * std::fabs(bSin);
    }
}

void MclTracking::resample() {
    auto& particles = *mParticles;
    auto& newParticles = *mNewParticles;

    float totalWeight = 0.0f;
    for (int i = 0; i < mConfig.particleCount; i++) {
        totalWeight += particles[i].weight;
    }

    if (totalWeight < 1e-20f) {
        uniformReset();
        return;
    }

    // Scattering particles down an axis nothing is watching does not find the
    // robot, it just outvotes the cluster that had it right.
    const bool canPlace = mConfig.minAxisObservability <= 0.0f ||
                          (mObsX >= mConfig.minAxisObservability &&
                           mObsY >= mConfig.minAxisObservability);
    const float frac = (canPlace && mInjectFrac >= mConfig.injectResampleFloor)
                       ? mInjectFrac : 0.0f;
    const int injected =
        std::min(mConfig.particleCount - 1,
                 static_cast<int>(frac * mConfig.particleCount));
    const int drawn = mConfig.particleCount - injected;

    float step = totalWeight / static_cast<float>(drawn);

    std::uniform_real_distribution<float> starter(0.0f, step);
    float currPos = starter(mGen);

    float cumWeight = particles[0].weight;
    int index = 0;

    for (int m = 0; m < drawn; m++) {
        while (index < mConfig.particleCount - 1 && currPos > cumWeight) {
            index++;
            cumWeight += particles[index].weight;
        }

        Particle selected = particles[index];
        selected.pose.x     += nextNoise() * mConfig.distResampleVariance;
        selected.pose.y     += nextNoise() * mConfig.distResampleVariance;
        selected.pose.theta  = wrapRad(selected.pose.theta +
                                       nextNoise() * mConfig.thetaResampleVariance);
        selected.weight = 1.0f;
        selected.logWeight = 0.0f;

        newParticles[m] = selected;
        currPos += step;
    }

    if (injected > 0) {
        const float heading = mOdom.imuHeadingRad();
        const float reach = reachLimit();
        const bool local = mConfig.injectRadiusIn > 0.0f;
        const float half = local ? mConfig.injectRadiusIn : Field::FIELD_HALF_WALL;
        const float cx = local ? mRawEstimate.x : 0.0f;
        const float cy = local ? mRawEstimate.y : 0.0f;
        std::uniform_real_distribution<float> spread(-half, half);
        for (int m = drawn; m < mConfig.particleCount; m++) {
            newParticles[m].pose = {
                clamp(cx + spread(mGen), -reach, reach),
                clamp(cy + spread(mGen), -reach, reach),
                wrapRad(heading + nextNoise() * mConfig.thetaResampleVariance)};
            newParticles[m].weight = 1.0f;
            newParticles[m].logWeight = 0.0f;
        }
    }

    std::swap(mParticles, mNewParticles);
}

std::pair<Pose, float> MclTracking::getEstimate() const {
    std::lock_guard<pros::Mutex> lock(mEstimateMutex);
    return {mPublished.pose, mPublished.nEff};
}

MclTracking::Estimate MclTracking::estimate() const {
    std::lock_guard<pros::Mutex> lock(mEstimateMutex);
    return mPublished;
}

void MclTracking::publishEstimate(const Estimate& est) {
    std::lock_guard<pros::Mutex> lock(mEstimateMutex);
    mPublished = est;
}

MclTracking::Estimate MclTracking::computeEstimate() const {
    Estimate out;
    out.pose = mRawEstimate;

    float xSum = 0.0f, ySum = 0.0f;
    float sinSum = 0.0f, cosSum = 0.0f;
    float totalWeight = 0.0f;
    float weightSqrSum = 0.0f;

    const auto& particles = *mParticles;
    for (int i = 0; i < mConfig.particleCount; i++) {
        const auto& p = particles[i];
        xSum += p.pose.x * p.weight;
        ySum += p.pose.y * p.weight;
        sinSum += FastTrig::sin(p.pose.theta) * p.weight;
        cosSum += FastTrig::cos(p.pose.theta) * p.weight;
        totalWeight += p.weight;
        weightSqrSum += p.weight * p.weight;
    }

    if (totalWeight < 1e-20f) return out;

    out.pose = {
        xSum / totalWeight,
        ySum / totalWeight,
        std::atan2(sinSum, cosSum)
    };
    out.nEff = totalWeight * totalWeight / weightSqrSum;
    if (!(out.nEff > 0.0f)) return out;

    totalWeight = 0.0f;
    float sxx = 0.0f, sxy = 0.0f, syy = 0.0f, stt = 0.0f;
    for (int i = 0; i < mConfig.particleCount; i++) {
        const auto& p = particles[i];
        const float dx = p.pose.x - out.pose.x;
        const float dy = p.pose.y - out.pose.y;
        const float dt = angleDiffRad(out.pose.theta, p.pose.theta);
        sxx += p.weight * dx * dx;
        sxy += p.weight * dx * dy;
        syy += p.weight * dy * dy;
        stt += p.weight * dt * dt;
        totalWeight += p.weight;
    }
    if (totalWeight < 1e-20f) return out;

    const float inv = 1.0f / totalWeight;
    out.covXX = sxx * inv;
    out.covXY = sxy * inv;
    out.covYY = syy * inv;
    out.covTheta = stt * inv;
    return out;
}

Pose MclTracking::update() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (mDisableTimers[i] > 0.0f) {
            mDisableTimers[i] -= mConfig.updatePeriodMs;
            mDisabledSensors[i] = (mDisableTimers[i] > 0.0f);
        }
    }

    predict();

    // Spinning fast enough and the beams have swept onto different geometry
    // between predicting a pose and reading them. Every particle then looks
    // wrong, which collapses the weights and reads as a lost filter rather
    // than as a blind moment. Hold the update and let odometry carry it; the
    // weights are left alone so the collapse never enters the fast/slow
    // averages and never triggers injection.
    mSensorsHeld = mConfig.maxTurnRateRadS > 0.0f &&
                   mLatestTurnRate > mConfig.maxTurnRateRadS;
    if (!mSensorsHeld) {
        updateWeights();
    }

    const Estimate estimate = computeEstimate();

    float distSinceResample = distanceToPoint(mLastResamplePose, estimate.pose.x, estimate.pose.y);

    bool shouldResample = false;
    if (mSensorsHeld) {
        // Resampling on weights nothing refreshed only throws away spread.
        shouldResample = false;
    } else if (estimate.nEff < mConfig.resampleThreshold * 0.5f) {
        shouldResample = true;
    } else if (estimate.nEff < mConfig.resampleThreshold &&
               distSinceResample > 5.0f &&
               mLatestSpeed < 100.0f) {
        shouldResample = true;
    } else if (mInjectFrac >= mConfig.injectResampleFloor) {
        shouldResample = true;
    }

    if (shouldResample) {
        resample();
        mLastResamplePose = estimate.pose;
    }

    mRawEstimate = estimate.pose;

    // Pre-resample on purpose: this is what the filter acted on, and its
    // nEff and covariance describe the posterior rather than the roughening.
    publishEstimate(estimate);

    if (mConfig.autoSync) {
        syncToOdometry();
    }

    static constexpr int REGEN_COUNT = 3;
    for (int i = 0; i < REGEN_COUNT; i++) {
        regenerateNoise();
    }

    return estimate.pose;
}

void MclTracking::syncToOdometry() {
    Pose odomPose = mOdom.getPose();

    float newX = odomPose.x + mConfig.distSyncProp * (mRawEstimate.x - odomPose.x);
    float newY = odomPose.y + mConfig.distSyncProp * (mRawEstimate.y - odomPose.y);

    float delta = angleDiffRad(odomPose.theta, mRawEstimate.theta);
    float newTheta = odomPose.theta + mConfig.thetaSyncProp * delta;

    mOdom.syncPose({newX, newY, wrapRad(newTheta)});
}

void MclTracking::setDistanceSensors(const std::array<pros::Distance*, SENSOR_COUNT>& sensors) {
    mDistanceSensors = sensors;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        mDisabledSensors[i] = (sensors[i] == nullptr);
        mDisableTimers[i] = 0.0f;
    }
}

void MclTracking::setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts) {
    mSensorMounts = mounts;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        mMountTrigs[i] = {std::cos(mSensorMounts[i].theta),
                          std::sin(mSensorMounts[i].theta)};
    }
}

void MclTracking::enableSensor(int index) {
    if (index < 0 || index >= SENSOR_COUNT) return;
    mDisabledSensors[index] = false;
    mDisableTimers[index] = 0.0f;
}

void MclTracking::disableSensor(int index) {
    if (index < 0 || index >= SENSOR_COUNT) return;
    mDisabledSensors[index] = true;
    mDisableTimers[index] = 1e20f;
}

void MclTracking::disableSensorFor(int index, float durationMs) {
    if (index < 0 || index >= SENSOR_COUNT) return;
    mDisabledSensors[index] = true;
    mDisableTimers[index] = durationMs;
}

void MclTracking::setObstacles(const std::vector<LineObstacle>* lineObstacles,
                                const std::vector<CircleObstacle>* circleObstacles) {
    mLineObstacles = lineObstacles;
    mCircleObstacles = circleObstacles;
}

void MclTracking::setDrift(float verticalDrift, float horizontalDrift) {
    float updatesPerSec = 1000.0f / mConfig.updatePeriodMs;
    mVertDrift = verticalDrift / updatesPerSec;
    mHorizDrift = horizontalDrift / updatesPerSec;
}

Pose MclTracking::getRawEstimate() const {
    return mRawEstimate;
}

void MclTracking::setSeed(std::uint32_t seed) {
    mGen.seed(seed);
    mDist.reset();

    for (int i = 0; i < NOISE_POOL_SIZE; i++) {
        mNoisePool[i] = mDist(mGen);
    }
    mNoiseIdx = 0;
    mRegenIdx = 0;
}

int MclTracking::snapshotParticles(float* out, int maxParticles) const {
    if (out == nullptr || maxParticles <= 0) return 0;

    const int count = std::min(mConfig.particleCount, maxParticles);
    const auto& particles = *mParticles;
    for (int i = 0; i < count; i++) {
        const auto& p = particles[i];
        out[i * 4 + 0] = p.pose.x;
        out[i * 4 + 1] = p.pose.y;
        out[i * 4 + 2] = p.pose.theta;
        out[i * 4 + 3] = p.weight;
    }
    return count;
}
}
