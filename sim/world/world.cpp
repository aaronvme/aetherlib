#include "world.hpp"
#include "field.hpp"
#include "pieces.hpp"

#include <algorithm>
#include <cmath>

#include "../platform/sim_runtime.hpp"
#include "gamespecific/config.hpp"
#include "gamespecific/lift.hpp"

namespace sim {
namespace {

constexpr double kMmPerInch = 25.4;

/// Wheel surface speed at the configured free RPM, inches/sec.
float maxWheelSpeedIn(const RobotParams& p) {
    return (p.maxWheelRpm / 60.0f) * (chad::PI * p.wheelDiameterIn);
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
    mParams.wheelDiameterIn = chad::drivetrain.wheelDiameter;
    mParams.trackWidthIn    = chad::drivetrain.trackWidth;
    mParams.maxWheelRpm     = chad::drivetrain.rpm;

    // And the IMU's calibration, for the same reason. The under-report has to
    // be the inverse of whatever ScaledIMU is correcting by or the two do not
    // cancel, and the robot's measured 360/359 against this default's assumed
    // 360/354.25 left the heading reading 1.19 deg low from the first tick --
    // which is a 1.3 in cross-track error after 63 in of straight driving,
    // attributable to nothing the test could see.
    const double scale = chad::imu.scaleFactor();
    if (scale > 0.0) mParams.imuUnderReport = static_cast<float>(1.0 / scale);

    field().loadDefault();
    setPhysicsFn([this](double dtMs) { this->step(dtMs); });
}

void World::teleport(const chad::Pose& pose) {
    mTruth.x = pose.x;
    mTruth.y = pose.y;
    mTruth.theta = pose.theta;
    // Encoders and IMU intentionally continue from where they were: a kidnap
    // must be invisible to dead reckoning, which is what MCL has to notice.
    updateDistanceSensors();
    updateClawSensor();
}

void World::resetTo(const chad::Pose& pose) {
    mTruth = TrueState{};
    mTruth.x = pose.x;
    mTruth.y = pose.y;
    mTruth.theta = pose.theta;

    mLeftCmdIn = mRightCmdIn = 0.0f;
    mLiftDeg = mLiftVel = mPubLiftDeg = 0.0;
    mMotorDegLeft = mMotorDegRight = 0.0;
    mDistAccum = mImuAccum = mEncAccum = 0.0;
    mPodVertIn = mPodHorizIn = 0.0;

    // The IMU reads a heading, not a pose, so seed it from the start heading in
    // VEX convention (0 = forward, clockwise positive).
    //
    // Scale by underReport for the same reason the per-tick deltas are: the bus
    // holds what the PHYSICAL sensor reports, and ScaledIMU multiplies its own
    // scale factor back in on the way out. Seeding the true angle here instead
    // would make the corrected heading read high from the first tick -- a
    // constant offset that looks exactly like a real miscalibration.
    mImuRotationDeg = chad::stdRadToVexDeg(pose.theta) * mParams.imuUnderReport;

    for (int i = 1; i < kMaxSmartPorts; i++) {
        bus().motors[i].positionDeg = 0.0;
        bus().motors[i].velocityRpm = 0.0;
        bus().motors[i].commandedMv = 0.0;
        bus().rotations[i].positionCentideg = 0;
    }
    // The bus was just zeroed, so the delta trackers have to agree with it or
    // the next tick republishes the previous run's travel as a real delta.
    mPubMotorDegLeft = mPubMotorDegRight = 0.0;
    mPubPodVertCentideg = mPubPodHorizCentideg = 0;

    seedImu();
    // Pods must be on the bus before anything baselines against them. Left to
    // the next encoder tick, OdomTracking::reset() would sample the previous
    // run's travel and then see it jump to zero as a real delta.
    publishEncoders();
    updateDistanceSensors();
    updateClawSensor();
}

// ============================================================================
// step
// ============================================================================

void World::step(double dtMs) {
    const double dt = dtMs / 1000.0;
    if (dt <= 0.0) return;

    updateDrive(dt);
    updateLift(dt);

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
        updateClawSensor();
    }
}

// ============================================================================
// Lift
// ============================================================================

void World::updateLift(double dtSec) {
    const auto* group = &chad::liftMotor;
    if (group->ports().empty()) return;

    // The group carries one command; the first port is enough to read it.
    const auto& lead = bus().motors[group->ports()[0]];
    const double cmd = lead.commandedMv / 12000.0;   // -1 .. 1
    const double target = cmd * mParams.liftDegPerSec;

    if (mParams.liftResponseTauMs > 1e-3f) {
        const double alpha =
            1.0 - std::exp(-dtSec * 1000.0 / mParams.liftResponseTauMs);
        mLiftVel += (target - mLiftVel) * alpha;
    } else {
        mLiftVel = target;
    }

    mLiftDeg += mLiftVel * dtSec;

    // Hard stops, so a routine that drives into the bottom does not integrate
    // its way to a negative height and then report the wrong pin.
    const double top = liftPinDegrees(kLiftPinCount) + 120.0;
    if (mLiftDeg < 0.0) { mLiftDeg = 0.0; if (mLiftVel < 0.0) mLiftVel = 0.0; }
    if (mLiftDeg > top) { mLiftDeg = top; if (mLiftVel > 0.0) mLiftVel = 0.0; }

    const double dq = mLiftDeg - mPubLiftDeg;
    for (std::uint8_t port : group->ports()) {
        auto& m = bus().motors[port];
        m.positionDeg += m.reversed ? -dq : dq;
        m.velocityRpm = (m.reversed ? -mLiftVel : mLiftVel) / 6.0;
    }
    mPubLiftDeg = mLiftDeg;
}

// ============================================================================
// Claw
// ============================================================================

void World::updateClawSensor() {
    auto& slot = bus().distances[chad::claw_dist.get_port()];

    const chad::Pose mount{mParams.clawMountX, mParams.clawMountY,
                           mParams.clawMountDeg * chad::DEG_TO_RAD};
    pieces().setClawMount(mount);

    float cx = 0.0f, cy = 0.0f, cdir = 0.0f;
    pieces().clawPoint({mTruth.x, mTruth.y, mTruth.theta}, cx, cy, cdir);

    const float maxRangeIn = mParams.clawMaxRangeMm / static_cast<float>(kMmPerInch);
    const float ca = std::cos(cdir), sa = std::sin(cdir);
    // Everything, pins and cups included: this sensor is inches from what it
    // is looking at, and seeing a pin is the whole job.
    const RayHit hit = field().cast(cx, cy, ca, sa, maxRangeIn, Field::CAST_ALL);

    mClawBeam.active = true;
    mClawBeam.originX = cx;
    mClawBeam.originY = cy;
    mClawBeam.dirDeg = cdir * chad::RAD_TO_DEG;
    mClawBeam.fovDeg = 0.0f;

    if (!hit.hit || hit.dist >= maxRangeIn - 1e-3f) {
        slot.distanceMm = 9999;
        slot.confidence = 0;
        mClawBeam.inRange = false;
        mClawBeam.confidence = 0;
        mClawBeam.rangeIn = maxRangeIn;
        mClawBeam.endX = cx + ca * maxRangeIn;
        mClawBeam.endY = cy + sa * maxRangeIn;
        mClawBeam.hitX = mClawBeam.endX;
        mClawBeam.hitY = mClawBeam.endY;
        mClawBeam.axisRangeIn = maxRangeIn;
        return;
    }

    double mm = hit.dist * kMmPerInch;
    if (mParams.distNoiseMm > 0.0f) mm += randNormalAux() * mParams.distNoiseMm;
    if (mm < 0.0) mm = 0.0;

    slot.distanceMm = static_cast<std::int32_t>(mm);
    slot.confidence = 63;
    mClawBeam.inRange = true;
    mClawBeam.confidence = 63;
    mClawBeam.rangeIn = hit.dist;
    mClawBeam.axisRangeIn = hit.dist;
    mClawBeam.endX = cx + ca * hit.dist;
    mClawBeam.endY = cy + sa * hit.dist;
    mClawBeam.hitX = mClawBeam.endX;
    mClawBeam.hitY = mClawBeam.endY;
    mClawBeam.hitIsObstacle = hit.isCircle;
}

void World::updateDrive(double dtSec) {
    const float vMax = maxWheelSpeedIn(mParams);

    // Commanded voltage -> target wheel speed. Motors in a group all carry the
    // same command, so reading the first port of each side is sufficient.
    //
    // The reversal flag is deliberately NOT applied. A drivetrain's port signs
    // exist to cancel the mirrored mounting of one side, so a positive command
    // drives both sides forward -- which is why publishEncoders() writes
    // `reversed ? -q : q` and lets the shim negate it back on read. Applying it
    // here as well made the two paths disagree: with the real robot's ports
    // (-18, -19, -12 on the left) a straight-ahead command spun the sim in
    // place at 230 deg/s, and every motion test failed as a consequence.
    auto sideTargetIn = [&](const pros::MotorGroup* group, float scale) -> float {
        if (group == nullptr || group->ports().empty()) return 0.0f;
        const auto& m = bus().motors[group->ports()[0]];
        return static_cast<float>(m.commandedMv / 12000.0) * vMax * scale;
    };

    const float targetL = sideTargetIn(chad::drivetrain.leftMotors,  mParams.leftScale);
    const float targetR = sideTargetIn(chad::drivetrain.rightMotors, mParams.rightScale);

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

    // Walls and posts are solid, and the body is tested as the rectangle it is:
    // standing a circle in for it cannot be right at both angles -- a radius
    // that stops the robot square-on lets a corner through at 45 deg, and one
    // that stops the corner holds it a corner's-worth off every wall it faces.
    //
    // Which means rotation is no longer free. A circle's footprint does not
    // change when it turns, so the old rule -- block translation, always allow
    // rotation -- could not put the body anywhere it was not already. A
    // rectangle's does: parked square-on against a wall, a 10 degree turn
    // buries a corner 1.17 in into it, and then translation is blocked in EVERY
    // direction, including away. That is the permanent wedge the old comment
    // was guarding against, arrived at from the other side.
    //
    // So the two are accepted separately, most-preferred first, and whichever
    // combination is free wins. The body can then never overlap anything, and
    // there is always a way out: if turning is blocked the robot can still
    // reverse, and if reversing is blocked it can still turn.
    const float endTheta = static_cast<float>(mTruth.theta + dTheta);
    const float halfLen = 0.5f * mParams.robotLengthIn;
    const float halfWid = 0.5f * mParams.robotWidthIn;
    auto free = [&](double x, double y, float th) {
        return !field().collidesBody(static_cast<float>(x), static_cast<float>(y),
                                     std::cos(th), std::sin(th), halfLen, halfWid,
                                     mParams.collideObstacles);
    };

    // A pose that already overlaps can only have come from a teleport or a
    // reset -- the rules below cannot produce one. Driving out of it has to be
    // allowed or the robot is stuck where it was dropped.
    const bool stuckAlready = !free(mTruth.x, mTruth.y, static_cast<float>(mTruth.theta));

    bool moved = false, turned = false;
    if (stuckAlready || free(nx, ny, endTheta)) {
        moved = turned = true;
    } else if (free(nx, ny, static_cast<float>(mTruth.theta))) {
        moved = true;                   // translate, hold the heading
    } else if (free(mTruth.x, mTruth.y, endTheta)) {
        turned = true;                  // turn in place
    }

    if (moved) {
        mTruth.x = static_cast<float>(nx);
        mTruth.y = static_cast<float>(ny);
    }
    const double appliedTheta = turned ? dTheta : 0.0;
    mTruth.theta = chad::wrapRad(static_cast<float>(mTruth.theta + appliedTheta));
    mTruth.colliding = stuckAlready || !(moved && turned);

    // The encoders see the travel the wheels claim -- blocked or not, they spin
    // and slip. The IMU does not: a gyro reports the rotation the body actually
    // made, so a turn the walls refused is a turn it never saw.
    accumulateEncoders(mLeftCmdIn * dtSec, mRightCmdIn * dtSec);
    accumulateImu(appliedTheta, dtSec);

    // Dead wheels are unpowered and roll on the floor, so they see real travel
    // rather than what the drive wheels claim: no encoderSlip term, and nothing
    // at all while a wall is holding the robot still. Both pods sit at the
    // tracking centre, so rotation contributes nothing.
    if (moved) mPodVertIn += dDist;

    applyRearAligner(dtSec);
}

// ============================================================================
// Rear goal aligner
// ============================================================================

void World::applyRearAligner(double dtSec) {
    if (mParams.alignerWidthIn <= 0.0f || mParams.alignerDepthIn <= 0.0f) return;
    // Passive arms: they only do anything while the robot is being driven
    // backwards onto something.
    if (mTruth.vLinIn > -1.0f) return;

    const float ct = std::cos(mTruth.theta), st = std::sin(mTruth.theta);
    const float halfLen = 0.5f * mParams.robotLengthIn;
    const float halfWid = 0.5f * mParams.robotWidthIn;

    // The goal nearest the centreline that the mouth has actually caught.
    float bestLy = 0.0f;
    bool found = false;
    for (const auto& c : field().circles()) {
        const float dx = c.x - static_cast<float>(mTruth.x);
        const float dy = c.y - static_cast<float>(mTruth.y);
        const float lx =  dx * ct + dy * st;
        const float ly = -dx * st + dy * ct;

        // Behind the back face, within the arms' reach.
        const float back = -lx - halfLen;
        if (back < -1.0f || back > mParams.alignerDepthIn) continue;

        // The mouth narrows from the tips to the throat, so an arm only
        // catches a goal it could physically reach at that depth.
        const float t = back / mParams.alignerDepthIn;
        const float halfOpen = 0.5f * (mParams.alignerWidthIn +
            (mParams.alignerMouthIn - mParams.alignerWidthIn) * t);
        if (std::fabs(ly) > halfOpen) continue;

        if (!found || std::fabs(ly) < std::fabs(bestLy)) { bestLy = ly; found = true; }
    }
    if (!found || std::fabs(bestLy) < 1e-3f) return;

    // Slide toward centring it. Capped per tick so it reads as being funnelled
    // rather than snapping, and refused outright if the slide would put the
    // body into something.
    const float cap = mParams.alignerRateInS * static_cast<float>(dtSec);
    float slide = bestLy;
    if (slide >  cap) slide =  cap;
    if (slide < -cap) slide = -cap;

    const float nx = static_cast<float>(mTruth.x) + slide * -st;
    const float ny = static_cast<float>(mTruth.y) + slide *  ct;
    if (field().collidesBody(nx, ny, ct, st, halfLen, halfWid,
                             mParams.collideObstacles)) {
        return;
    }
    mTruth.x = nx;
    mTruth.y = ny;

    // The drive wheels did not turn to do this -- the robot was pushed across
    // by the arms -- so the encoders see none of it. The horizontal tracking
    // pod does: it is unpowered and rolls sideways, which is the whole reason
    // to have one.
    mPodHorizIn += slide;
}

// ============================================================================
// Encoders
// ============================================================================

void World::accumulateEncoders(double leftTravelIn, double rightTravelIn) {
    // Encoders only see the fraction of travel that was not slip.
    const double seen = 1.0 - mParams.encoderSlip;
    const double circumference = chad::PI * mParams.wheelDiameterIn;
    const double extRatio = chad::drivetrain.externalGearRatio();
    if (circumference <= 0.0 || extRatio <= 0.0) return;

    // Wheel revolutions -> motor revolutions -> motor shaft degrees, the
    // inverse of the conversion OdomTracking::currentVertDistance() applies.
    mMotorDegLeft  += (leftTravelIn  * seen / circumference) / extRatio * 360.0;
    mMotorDegRight += (rightTravelIn * seen / circumference) / extRatio * 360.0;
}

namespace {
/// Inches of pod travel -> rotation-sensor centidegrees.
std::int32_t podCentideg(double travelIn, double wheelDiamIn) {
    const double circumference = chad::PI * wheelDiamIn;
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
    const std::int32_t podVert = podCentideg(mPodVertIn, kPodWheelDiamIn);
    const std::int32_t podHoriz = podCentideg(mPodHorizIn, kPodWheelDiamIn);
    bus().rotations[kVertPodPort].positionCentideg += podVert - mPubPodVertCentideg;
    bus().rotations[kHorizPodPort].positionCentideg += podHoriz - mPubPodHorizCentideg;
    mPubPodVertCentideg = podVert;
    mPubPodHorizCentideg = podHoriz;

    const double circumference = chad::PI * mParams.wheelDiameterIn;
    const double extRatio = chad::drivetrain.externalGearRatio();
    if (circumference <= 0.0 || extRatio <= 0.0) return;

    const double seen = 1.0 - mParams.encoderSlip;

    auto writeSide = [&](const pros::MotorGroup* group, double deg, double velIn,
                         double& published) {
        if (group == nullptr) return;
        const double motorRpm = (velIn / circumference) / extRatio * 60.0;
        // Quantise the absolute travel, then publish the difference, so the
        // resolution step stays exact instead of accumulating rounding error.
        const double q = quantise(deg, mParams.encoderResolutionDeg);
        const double dq = q - published;
        for (std::uint8_t port : group->ports()) {
            auto& m = bus().motors[port];
            // The bus stores the raw sensor value; pros::Motor applies the
            // reversal flag on read, exactly as the real device does.
            m.positionDeg += m.reversed ? -dq : dq;
            m.velocityRpm = m.reversed ? -motorRpm : motorRpm;
            m.currentMa = std::fabs(m.commandedMv) * 0.05;
        }
        published = q;
    };

    writeSide(chad::drivetrain.leftMotors,  mMotorDegLeft,  mLeftCmdIn * seen,
              mPubMotorDegLeft);
    writeSide(chad::drivetrain.rightMotors, mMotorDegRight, mRightCmdIn * seen,
              mPubMotorDegRight);
}

// ============================================================================
// IMU
// ============================================================================

void World::accumulateImu(double dThetaRad, double dtSec) {
    // Standard-math CCW radians -> VEX clockwise-positive degrees.
    const double dVexDeg = -dThetaRad * chad::RAD_TO_DEG;

    mImuRotationDeg += dVexDeg * mParams.imuUnderReport;
    mImuRotationDeg += mParams.imuDriftDegPerSec * dtSec;
    if (mParams.imuNoiseDeg > 0.0f) {
        mImuRotationDeg += randNormal() * mParams.imuNoiseDeg * std::sqrt(dtSec);
    }
}

void World::publishImu() {
    // The V5 reports hundredths of a degree.
    const double q = quantise(mImuRotationDeg, 0.01);
    const double dq = q - mPubImuRotationDeg;
    for (int i = 1; i < kMaxSmartPorts; i++) {
        if (bus().imus[i].present) bus().imus[i].rotationDeg += dq;
    }
    mPubImuRotationDeg = q;
}

/// Write the IMU's absolute heading onto the bus, discarding whatever the
/// library had tared it to. Only a world reset may do this; the per-tick path
/// publishes deltas so a tare survives.
void World::seedImu() {
    const double q = quantise(mImuRotationDeg, 0.01);
    for (int i = 1; i < kMaxSmartPorts; i++) {
        if (bus().imus[i].present) bus().imus[i].rotationDeg = q;
    }
    mPubImuRotationDeg = q;
}

// ============================================================================
// Distance sensors
// ============================================================================

void World::updateDistanceSensors() {
    const unsigned kBeamLayers = mParams.distSeePieces
                                     ? Field::CAST_ALL : Field::CAST_BEAMS;
    const float ct = std::cos(mTruth.theta);
    const float st = std::sin(mTruth.theta);
    const float maxRangeIn = mParams.distMaxRangeMm / static_cast<float>(kMmPerInch);

    const int rays = (mParams.distConeRays < 1) ? 1 : mParams.distConeRays;
    const float fov = mParams.distFovDeg * chad::DEG_TO_RAD;

    for (int i = 0; i < chad::MAX_DISTANCE_SENSORS; i++) {
        pros::Distance* dev = chad::distSensors[i];
        if (dev == nullptr) {
            mBeams[i].active = false;
            continue;
        }

        // Mount offsets straight from the robot config: x forward, y left,
        // theta pointing angle -- the same array handed to MCL.
        const chad::Pose& mount = chad::sensorMounts[i];

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
            // Walls, goals, edited geometry and the other robots -- but not
            // loose pins and cups. See Field::CastLayer.
            const RayHit h = field().cast(sx, sy, ca, sa, maxRangeIn, kBeamLayers);
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
        beam.dirDeg = axis * chad::RAD_TO_DEG;
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
        const float incidence = std::acos(chad::clamp(dot, -1.0f, 1.0f)) * chad::RAD_TO_DEG;
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
        conf = static_cast<int>(chad::clamp(static_cast<float>(conf), 0.0f, 63.0f));

        slot.distanceMm = static_cast<std::int32_t>(mm);
        slot.confidence = conf;
        beam.confidence = conf;
        beam.inRange = true;
    }
}

}  // namespace sim

#ifdef __EMSCRIPTEN__
#include <emscripten.h>

/// Unplug (1) or reconnect (0) the distance sensor in RCL slot `slot`.
extern "C" EMSCRIPTEN_KEEPALIVE void sim_distance_fault(int slot, int faulted) {
    if (slot < 0 || slot >= chad::MAX_DISTANCE_SENSORS) return;
    const pros::Distance* dev = chad::distSensors[slot];
    if (dev != nullptr) sim::distanceFaulted(dev->get_port()) = (faulted != 0);
}
#endif
