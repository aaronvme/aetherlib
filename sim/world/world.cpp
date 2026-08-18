#include "world.hpp"
#include "field.hpp"

#include <cmath>

#include "../platform/sim_runtime.hpp"
#include "gamespecific/config.hpp"

namespace sim {
namespace {

constexpr double kMmPerInch = 25.4;

/// Wheel surface speed at the configured free RPM, inches/sec.
float maxWheelSpeedIn(const RobotParams& p) {
    return (p.maxWheelRpm / 60.0f) * (riku::PI * p.wheelDiameterIn);
}

/// Round to a device's reporting resolution.
double quantise(double v, double step) {
    if (step <= 0.0) return v;
    return std::round(v / step) * step;
}

}  // namespace

World& world() {
    static World w;
    return w;
}

// ============================================================================
// init / reset
// ============================================================================

void World::init() {
    if (mInitialised) return;
    mInitialised = true;

    // Take geometry from the robot's own drivetrain so the sim cannot drift
    // out of sync with the config that ships to the brain.
    mParams.wheelDiameterIn = riku::drivetrain.wheelDiameter;
    mParams.trackWidthIn    = riku::drivetrain.trackWidth;
    mParams.maxWheelRpm     = riku::drivetrain.rpm;

    field().loadDefault();
    setPhysicsFn([this](double dtMs) { this->step(dtMs); });
}

void World::teleport(const riku::Pose& pose) {
    mTruth.x = pose.x;
    mTruth.y = pose.y;
    mTruth.theta = pose.theta;
    // Encoders and IMU intentionally continue from where they were: a kidnap
    // must be invisible to dead reckoning, which is what MCL has to notice.
    updateDistanceSensors();
}

void World::resetTo(const riku::Pose& pose) {
    mTruth = TrueState{};
    mTruth.x = pose.x;
    mTruth.y = pose.y;
    mTruth.theta = pose.theta;

    mLeftCmdIn = mRightCmdIn = 0.0f;
    mMotorDegLeft = mMotorDegRight = 0.0;
    mDistAccum = mImuAccum = mEncAccum = 0.0;
    mPodVertIn = mPodHorizIn = 0.0;

    // The IMU reads a heading, not a pose, so seed it from the start heading in
    // VEX convention (0 = forward, clockwise positive).
    //
    // Scale by underReport for the same reason the per-tick deltas are: the bus
    // holds what the PHYSICAL sensor reports, and ScaledIMU multiplies by
    // 360/354.25 on the way out. Seeding the true angle here instead would make
    // the corrected heading read 1.6% high from the first tick -- a constant
    // offset that looks exactly like a real miscalibration.
    mImuRotationDeg = riku::stdRadToVexDeg(pose.theta) * mParams.imuUnderReport;

    for (int i = 1; i < kMaxSmartPorts; i++) {
        bus().motors[i].positionDeg = 0.0;
        bus().motors[i].velocityRpm = 0.0;
        bus().motors[i].commandedMv = 0.0;
        bus().rotations[i].positionCentideg = 0;
    }
    publishImu();
    // Pods must be on the bus before anything baselines against them. Left to
    // the next encoder tick, OdomTracking::reset() would sample the previous
    // run's travel and then see it jump to zero as a real delta.
    publishEncoders();
    updateDistanceSensors();
}

// ============================================================================
// step
// ============================================================================

void World::step(double dtMs) {
    const double dt = dtMs / 1000.0;
    if (dt <= 0.0) return;

    updateDrive(dt);

    // Devices refresh on their own clocks. Between refreshes the bus keeps the
    // previous sample, so library code that polls faster than the hardware sees
    // repeated values -- which is what happens on a real brain and is a genuine
    // source of estimator lag.
    mEncAccum += dtMs;
    if (mEncAccum >= mParams.encoderPeriodMs) {
        mEncAccum = std::fmod(mEncAccum, static_cast<double>(mParams.encoderPeriodMs));
        publishEncoders();
    }

    mImuAccum += dtMs;
    if (mImuAccum >= mParams.imuPeriodMs) {
        mImuAccum = std::fmod(mImuAccum, static_cast<double>(mParams.imuPeriodMs));
        publishImu();
    }

    mDistAccum += dtMs;
    if (mDistAccum >= mParams.distPeriodMs) {
        mDistAccum = std::fmod(mDistAccum, static_cast<double>(mParams.distPeriodMs));
        updateDistanceSensors();
    }
}

void World::updateDrive(double dtSec) {
    const float vMax = maxWheelSpeedIn(mParams);

    // Commanded voltage -> target wheel speed. Motors in a group all carry the
    // same command, so reading the first port of each side is sufficient.
    auto sideTargetIn = [&](const pros::MotorGroup* group, float scale) -> float {
        if (group == nullptr || group->ports().empty()) return 0.0f;
        const auto& m = bus().motors[group->ports()[0]];
        const double mv = m.reversed ? -m.commandedMv : m.commandedMv;
        return static_cast<float>(mv / 12000.0) * vMax * scale;
    };

    const float targetL = sideTargetIn(riku::drivetrain.leftMotors,  mParams.leftScale);
    const float targetR = sideTargetIn(riku::drivetrain.rightMotors, mParams.rightScale);

    // First-order lag toward the commanded speed. Not a physics model -- just
    // enough dynamics that PID and exit conditions face something to settle.
    if (mParams.responseTauMs > 1e-3f) {
        const float alpha = 1.0f - std::exp(-static_cast<float>(dtSec) * 1000.0f / mParams.responseTauMs);
        mLeftCmdIn  += (targetL - mLeftCmdIn)  * alpha;
        mRightCmdIn += (targetR - mRightCmdIn) * alpha;
    } else {
        mLeftCmdIn = targetL;
        mRightCmdIn = targetR;
    }

    mTruth.leftVelIn = mLeftCmdIn;
    mTruth.rightVelIn = mRightCmdIn;

    // Differential drive kinematics.
    const float vLin = 0.5f * (mLeftCmdIn + mRightCmdIn);
    const float vAng = (mRightCmdIn - mLeftCmdIn) / mParams.trackWidthIn;

    mTruth.vLinIn = vLin;
    mTruth.vAngRad = vAng;

    const double dTheta = vAng * dtSec;
    const double dDist  = vLin * dtSec;

    // Integrate about the arc midpoint -- matches how odometry reconstructs it,
    // so any disagreement is from sensor error rather than integration style.
    const double midTheta = mTruth.theta + dTheta * 0.5;
    const double nx = mTruth.x + dDist * std::cos(midTheta);
    const double ny = mTruth.y + dDist * std::sin(midTheta);

    // Walls and posts are solid: stop translation on contact but let the robot
    // keep rotating, otherwise it can wedge permanently.
    const float radius = 0.5f * std::hypot(mParams.robotLengthIn, mParams.robotWidthIn);
    if (!field().collides(static_cast<float>(nx), static_cast<float>(ny), radius * 0.5f,
                          mParams.collideObstacles)) {
        mTruth.x = static_cast<float>(nx);
        mTruth.y = static_cast<float>(ny);
        mTruth.colliding = false;
    } else {
        mTruth.colliding = true;
    }
    mTruth.theta = riku::wrapRad(static_cast<float>(mTruth.theta + dTheta));

    accumulateEncoders(mLeftCmdIn * dtSec, mRightCmdIn * dtSec);
    accumulateImu(dTheta, dtSec);

    // Dead wheels roll on the floor rather than being driven, so they see the
    // travel the drive wheels only claim to have made: no encoderSlip term.
    // Both pods sit at the tracking centre, so rotation contributes nothing.
    mPodVertIn += dDist;

}

// ============================================================================
// Encoders
// ============================================================================

void World::accumulateEncoders(double leftTravelIn, double rightTravelIn) {
    // Encoders only see the fraction of travel that was not slip.
    const double seen = 1.0 - mParams.encoderSlip;
    const double circumference = riku::PI * mParams.wheelDiameterIn;
    const double extRatio = riku::drivetrain.externalGearRatio();
    if (circumference <= 0.0 || extRatio <= 0.0) return;

    // Wheel revolutions -> motor revolutions -> motor shaft degrees, the
    // inverse of the conversion OdomTracking::currentVertDistance() applies.
    mMotorDegLeft  += (leftTravelIn  * seen / circumference) / extRatio * 360.0;
    mMotorDegRight += (rightTravelIn * seen / circumference) / extRatio * 360.0;
}

namespace {
/// Inches of pod travel -> rotation-sensor centidegrees.
std::int32_t podCentideg(double travelIn, double wheelDiamIn) {
    const double circumference = riku::PI * wheelDiamIn;
    if (circumference <= 0.0) return 0;
    return static_cast<std::int32_t>(travelIn / circumference * 36000.0);
}
constexpr double kPodWheelDiamIn = 2.75;
constexpr int kVertPodPort = 11;
constexpr int kHorizPodPort = 12;
}  // namespace

void World::publishEncoders() {
    // Always published, whether or not the library is configured to read them.
    // Enabling pods mid-run then has a live baseline instead of a jump.
    bus().rotations[kVertPodPort].positionCentideg =
        podCentideg(mPodVertIn, kPodWheelDiamIn);
    bus().rotations[kHorizPodPort].positionCentideg =
        podCentideg(mPodHorizIn, kPodWheelDiamIn);

    const double circumference = riku::PI * mParams.wheelDiameterIn;
    const double extRatio = riku::drivetrain.externalGearRatio();
    if (circumference <= 0.0 || extRatio <= 0.0) return;

    const double seen = 1.0 - mParams.encoderSlip;

    auto writeSide = [&](const pros::MotorGroup* group, double deg, double velIn) {
        if (group == nullptr) return;
        const double motorRpm = (velIn / circumference) / extRatio * 60.0;
        const double q = quantise(deg, mParams.encoderResolutionDeg);
        for (std::uint8_t port : group->ports()) {
            auto& m = bus().motors[port];
            // The bus stores the raw sensor value; pros::Motor applies the
            // reversal flag on read, exactly as the real device does.
            m.positionDeg = m.reversed ? -q : q;
            m.velocityRpm = m.reversed ? -motorRpm : motorRpm;
            m.currentMa = std::fabs(m.commandedMv) * 0.05;
        }
    };

    writeSide(riku::drivetrain.leftMotors,  mMotorDegLeft,  mLeftCmdIn * seen);
    writeSide(riku::drivetrain.rightMotors, mMotorDegRight, mRightCmdIn * seen);
}

// ============================================================================
// IMU
// ============================================================================

void World::accumulateImu(double dThetaRad, double dtSec) {
    // Standard-math CCW radians -> VEX clockwise-positive degrees.
    const double dVexDeg = -dThetaRad * riku::RAD_TO_DEG;

    mImuRotationDeg += dVexDeg * mParams.imuUnderReport;
    mImuRotationDeg += mParams.imuDriftDegPerSec * dtSec;
    if (mParams.imuNoiseDeg > 0.0f) {
        mImuRotationDeg += randNormal() * mParams.imuNoiseDeg * std::sqrt(dtSec);
    }
}

void World::publishImu() {
    // The V5 reports hundredths of a degree.
    const double q = quantise(mImuRotationDeg, 0.01);
    for (int i = 1; i < kMaxSmartPorts; i++) {
        if (bus().imus[i].present) bus().imus[i].rotationDeg = q;
    }
}

// ============================================================================
// Distance sensors
// ============================================================================

void World::updateDistanceSensors() {
    const float ct = std::cos(mTruth.theta);
    const float st = std::sin(mTruth.theta);
    const float maxRangeIn = mParams.distMaxRangeMm / static_cast<float>(kMmPerInch);

    const int rays = (mParams.distConeRays < 1) ? 1 : mParams.distConeRays;
    const float fov = mParams.distFovDeg * riku::DEG_TO_RAD;

    for (int i = 0; i < riku::MAX_DISTANCE_SENSORS; i++) {
        pros::Distance* dev = riku::distSensors[i];
        if (dev == nullptr) {
            mBeams[i].active = false;
            continue;
        }

        // Mount offsets straight from the robot config: x forward, y left,
        // theta pointing angle -- the same array handed to MCL.
        const riku::Pose& mount = riku::sensorMounts[i];

        const float sx = mTruth.x + (mount.x * ct - mount.y * st);
        const float sy = mTruth.y + (mount.x * st + mount.y * ct);
        const float axis = mTruth.theta + mount.theta;

        // Sweep the cone and keep the STRONGEST return, not the nearest.
        //
        // Nearest is wrong for a flat surface: a wall fills the whole cone, so
        // the closest ray in it is always the one at the cone edge, and taking
        // that reads short by d*(1 - cos(alpha)/cos(alpha - fov/2)) -- several
        // inches once the robot is off square. A real sensor times the return
        // with the most energy coming back, which for a wall is the on-axis one.
        //
        // Strength falls as 1/d^2, with incidence, and with angle off the axis.
        // Keeping the 1/d^2 term is what still lets a nearby 4 in goal post beat
        // the wall behind it from off-axis, which was the point of the sweep.
        const float apodSigma = std::max(mParams.distConeSigmaFrac, 0.02f);
        RayHit best;
        best.dist = maxRangeIn;
        bool any = false;
        float bestStrength = -1.0f;
        float axisRange = maxRangeIn;

        for (int r = 0; r < rays; r++) {
            const float frac = (rays == 1) ? 0.0f
                             : (static_cast<float>(r) / (rays - 1)) - 0.5f;
            const float a = axis + frac * fov;
            const float ca = std::cos(a), sa = std::sin(a);
            const RayHit h = field().cast(sx, sy, ca, sa, maxRangeIn);
            if (std::fabs(frac) < 1e-6f && h.hit) axisRange = h.dist;
            if (!h.hit || h.dist < 1e-3f) continue;

            const float cosInc = -(ca * h.nx + sa * h.ny);
            if (cosInc <= 0.0f) continue;

            const float apod = std::exp(-0.5f * (frac / apodSigma) * (frac / apodSigma));
            const float strength = apod * cosInc / (h.dist * h.dist);
            if (strength > bestStrength) {
                bestStrength = strength;
                best = h;
                best.dist = h.dist;
                any = true;
                mBeams[i].hitX = sx + ca * h.dist;
                mBeams[i].hitY = sy + sa * h.dist;
                mBeams[i].hitIsObstacle = h.isCircle;
            }
        }

        auto& beam = mBeams[i];
        beam.active = true;
        beam.originX = sx;
        beam.originY = sy;
        beam.dirDeg = axis * riku::RAD_TO_DEG;
        beam.fovDeg = mParams.distFovDeg;
        beam.rangeIn = any ? best.dist : maxRangeIn;
        // Drawn along the axis to whatever the axis ray hit, so the laser is
        // shown pointing where it really points. The timed return is hitX/hitY.
        beam.axisRangeIn = axisRange;
        beam.endX = sx + std::cos(axis) * axisRange;
        beam.endY = sy + std::sin(axis) * axisRange;
        if (!any) { beam.hitX = beam.endX; beam.hitY = beam.endY; beam.hitIsObstacle = false; }

        auto& slot = bus().distances[dev->get_port()];

        if (!any) {
            slot.distanceMm = 9999;      // the V5's "nothing there" value
            slot.confidence = 0;
            beam.inRange = false;
            beam.incidenceDeg = 0.0f;
            beam.confidence = 0;
            continue;
        }

        // Angle between the outgoing ray and the surface normal. 0 is head-on.
        const float dot = -(std::cos(axis) * best.nx + std::sin(axis) * best.ny);
        const float incidence = std::acos(riku::clamp(dot, -1.0f, 1.0f)) * riku::RAD_TO_DEG;
        beam.incidenceDeg = incidence;

        double mm = best.dist * kMmPerInch;
        // Noise grows on a slanted surface: the return spreads across the cone.
        const float slantFactor = 1.0f + 2.0f * (incidence / 90.0f) * (incidence / 90.0f);
        if (mParams.distNoiseMm > 0.0f) {
            mm += randNormal() * mParams.distNoiseMm * slantFactor;
        }

        if (incidence > mParams.distMaxIncidenceDeg ||
            mm >= mParams.distMaxRangeMm ||
            best.dist >= maxRangeIn - 1e-3f) {
            slot.distanceMm = 9999;
            slot.confidence = 0;
            beam.inRange = false;
            beam.confidence = 0;
            continue;
        }

        if (mm < mParams.distMinRangeMm) mm = mParams.distMinRangeMm;

        // Confidence degrades with both range and incidence, mirroring how the
        // real sensor reports a weaker return the less light comes back.
        const double rangeTerm = 1.0 - 0.6 * (mm / mParams.distMaxRangeMm);
        const double angleTerm = 1.0 - 0.7 * (incidence / mParams.distMaxIncidenceDeg);
        int conf = static_cast<int>(63.0 * rangeTerm * angleTerm);
        conf = static_cast<int>(riku::clamp(static_cast<float>(conf), 0.0f, 63.0f));

        slot.distanceMm = static_cast<std::int32_t>(mm);
        slot.confidence = conf;
        beam.confidence = conf;
        beam.inRange = true;
    }
}

}  // namespace sim
