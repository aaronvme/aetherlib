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

/// Inches of pod travel -> rotation-sensor centidegrees.
std::int32_t podCentideg(double travelIn, double wheelDiamIn) {
    const double circumference = chad::PI * wheelDiamIn;
    if (circumference <= 0.0) return 0;
    return static_cast<std::int32_t>(travelIn / circumference * 36000.0);
}
// The bridge's optional dead wheels, at the tracking centre.
constexpr double kPodWheelDiamIn = 2.75;
constexpr int kVertPodPort = 11;
constexpr int kHorizPodPort = 12;

// --- contact response -------------------------------------------------------
/// Into-surface speed below which a hit does not rebound.
constexpr double kBounceMinInS = 10.0;
/// Square-up stops this close to flush.
constexpr double kFlushTolRad = 0.05 * chad::DEG_TO_RAD;
/// Square-up yaw-rate ceiling.
constexpr double kSquareUpMaxRadS = 720.0 * chad::DEG_TO_RAD;
/// Bisection depth for the free part of a slide, approach or push-out.
constexpr int kBisect = 10;
constexpr int kMaxContacts = 8;
/// Gap an approach stops at, in.
constexpr double kSkinIn = 1e-3;

/// Closest approach of the body to one obstacle.
struct Contact {
    double gap{0.0};            // in
    double nx{0.0}, ny{0.0};    // unit normal, body -> obstacle, world
    double px{0.0}, py{0.0};    // body point nearest it, world
    double bx{0.0}, by{0.0};    // same point, body frame
    bool cornerOnFlat{false};   // a body corner against a segment's interior
};

/// Every wall, post and dynamic segment within reach of the body: exactly what
/// Field::collidesBody() tests. Closest pairs of a box and a segment are a box
/// corner on the segment or a segment endpoint on the box.
int findContacts(const Field& f, double x, double y, double th, float halfLen,
                 float halfWid, bool obstacles, double reach, Contact* out) {
    const double c = std::cos(th), s = std::sin(th);
    const double L = halfLen, W = halfWid;
    int n = 0;
    auto emit = [&](double gap, double nbx, double nby, double bx, double by, bool corner) {
        if (gap >= reach || n >= kMaxContacts) return;
        Contact& k = out[n++];
        k.gap = gap;
        k.nx = nbx * c - nby * s;
        k.ny = nbx * s + nby * c;
        k.bx = bx;
        k.by = by;
        k.px = x + bx * c - by * s;
        k.py = y + bx * s + by * c;
        k.cornerOnFlat = corner;
    };
    auto toBody = [&](double wx, double wy, double& lx, double& ly) {
        const double dx = wx - x, dy = wy - y;
        lx =  dx * c + dy * s;
        ly = -dx * s + dy * c;
    };
    auto segment = [&](const Segment& sg) {
        double ax, ay, bx, by;
        toBody(sg.x1, sg.y1, ax, ay);
        toBody(sg.x2, sg.y2, bx, by);
        const double ex = bx - ax, ey = by - ay;
        const double len2 = ex * ex + ey * ey;
        double best = 1e30, bnx = 0, bny = 0, bpx = 0, bpy = 0;
        bool corner = false;
        if (len2 > 1e-12) {
            const double len = std::sqrt(len2);
            for (int i = 0; i < 4; i++) {
                const double cx = (i & 1) ? L : -L, cy = (i & 2) ? W : -W;
                const double t = ((cx - ax) * ex + (cy - ay) * ey) / len2;
                if (t <= 1e-6 || t >= 1.0 - 1e-6) continue;
                double mx = -ey / len, my = ex / len;
                const double qx = ax + t * ex, qy = ay + t * ey;
                if (mx * qx + my * qy < 0.0) { mx = -mx; my = -my; }
                const double d = (qx - cx) * mx + (qy - cy) * my;
                if (d < best) { best = d; bnx = mx; bny = my; bpx = cx; bpy = cy; corner = true; }
            }
        }
        for (int i = 0; i < 2; i++) {
            const double qx = i ? bx : ax, qy = i ? by : ay;
            const double px = std::clamp(qx, -L, L), py = std::clamp(qy, -W, W);
            const double d = std::hypot(qx - px, qy - py);
            if (d >= best) continue;
            double mx, my;
            if (d > 1e-9) { mx = (qx - px) / d; my = (qy - py) / d; }
            else if (L - std::fabs(px) < W - std::fabs(py)) { mx = px > 0 ? 1 : -1; my = 0; }
            else { mx = 0; my = py > 0 ? 1 : -1; }
            best = d; bnx = mx; bny = my; bpx = px; bpy = py; corner = false;
        }
        if (best < 1e29) emit(std::max(0.0, best), bnx, bny, bpx, bpy, corner);
    };

    for (const auto& sg : f.segments()) {
        if (sg.kind != 0 && !obstacles) continue;
        segment(sg);
    }
    if (!obstacles) return n;
    for (const auto& sg : f.dynamic()) segment(sg);
    // Round: no flat to square up to, so never a square-up contact.
    for (const auto& ci : f.circles()) {
        double lx, ly;
        toBody(ci.x, ci.y, lx, ly);
        const double px = std::clamp(lx, -L, L), py = std::clamp(ly, -W, W);
        const double d = std::hypot(lx - px, ly - py);
        if (d < 1e-9) continue;   // swallowed: overlapping, not a contact
        emit(std::max(0.0, d - ci.r), (lx - px) / d, (ly - py) / d, px, py, false);
    }
    return n;
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

    // And its tracking wheels: ports, diameters and offsets, so odometry reads
    // the geometry it was configured with. ADI encoders have no sim device.
    auto addWheels = [this](const std::vector<chad::TrackingWheel*>& wheels,
                            bool horizontal) {
        for (const chad::TrackingWheel* w : wheels) {
            if (w == nullptr || w->rotation() == nullptr) continue;
            addPod({w->rotation()->get_port(), w->rotation()->get_reversed() != 0,
                    horizontal, w->wheelDiamIn(), w->gearRatio(), w->offsetIn()});
        }
    };
    addWheels(chad::odomSensors.vertWheelCollection, false);
    addWheels(chad::odomSensors.horizWheelCollection, true);
    addPod({kVertPodPort, false, false, kPodWheelDiamIn, 1.0, 0.0});
    addPod({kHorizPodPort, false, true, kPodWheelDiamIn, 1.0, 0.0});

    field().loadDefault();
    setPhysicsFn([this](double dtMs) { this->step(dtMs); });
}

void World::addPod(const Pod& pod) {
    if (pod.port <= 0 || pod.port >= kMaxSmartPorts) return;
    // The config's wheels go in first, so one sharing a port with the
    // bridge's pair keeps its own geometry.
    for (const Pod& p : mPods) {
        if (p.port == pod.port) return;
    }
    mPods.push_back(pod);
}

void World::teleport(const chad::Pose& pose) {
    mTruth.x = pose.x;
    mTruth.y = pose.y;
    mTruth.theta = pose.theta;
    mBounceVx = mBounceVy = 0.0;
    mInContact = false;
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
    mBounceVx = mBounceVy = 0.0;
    mInContact = false;
    mLiftDeg = mLiftVel = mPubLiftDeg = 0.0;
    mMotorDegLeft = mMotorDegRight = 0.0;
    mDistAccum = mImuAccum = mEncAccum = 0.0;
    mPodVertIn = mPodHorizIn = mPodThetaRad = 0.0;

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
    for (Pod& p : mPods) p.publishedCentideg = 0;

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
    // A rebound from a hard hit rides on top of the drive's step.
    const double midTheta = mTruth.theta + dTheta * 0.5;
    const double bdx = mBounceVx * dtSec, bdy = mBounceVy * dtSec;
    const double dx = dDist * std::cos(midTheta) + bdx;
    const double dy = dDist * std::sin(midTheta) + bdy;
    const double nx = mTruth.x + dx;
    const double ny = mTruth.y + dy;

    // Walls and posts are solid, and the body is tested as the rectangle it is:
    // standing a circle in for it cannot be right at both angles -- a radius
    // that stops the robot square-on lets a corner through at 45 deg, and one
    // that stops the corner holds it a corner's-worth off every wall it faces.
    // Rotation is therefore not free either: parked square-on against a wall,
    // a 10 degree turn buries a corner 1.17 in into it. Every pose applied below
    // is collision-tested, so the body never overlaps anything.
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

    const float theta0 = mTruth.theta;
    double sx = 0.0, sy = 0.0, rot = 0.0;
    bool clear = false;
    if (stuckAlready || free(nx, ny, endTheta)) {
        sx = dx;
        sy = dy;
        rot = dTheta;
        clear = true;
    } else if (free(nx, ny, theta0)) {
        sx = dx;                        // translate, the surface refuses the turn
        sy = dy;
    } else {
        resolveContact(dx, dy, dTheta, dtSec, sx, sy, rot);
    }
    mInContact = !clear;

    mTruth.x = static_cast<float>(mTruth.x + sx);
    mTruth.y = static_cast<float>(mTruth.y + sy);
    mTruth.theta = chad::wrapRad(static_cast<float>(mTruth.theta + rot));
    mTruth.colliding = stuckAlready || !clear;

    // The rebound dies as the drive pushes the body back up to wheel speed.
    if (mBounceVx != 0.0 || mBounceVy != 0.0) {
        const double keep = mParams.responseTauMs > 1e-3f
            ? std::exp(-dtSec * 1000.0 / mParams.responseTauMs) : 0.0;
        mBounceVx *= keep;
        mBounceVy *= keep;
        if (std::hypot(mBounceVx, mBounceVy) < 1e-3) mBounceVx = mBounceVy = 0.0;
    }

    // The encoders see the travel the wheels claim -- blocked or not, they spin
    // and slip. The IMU does not: a gyro reports the rotation the body actually
    // made, so a turn the walls refused is a turn it never saw, and a turn a
    // wall forced is one it did.
    accumulateEncoders(mLeftCmdIn * dtSec, mRightCmdIn * dtSec);
    accumulateImu(rot, dtSec);

    // Dead wheels are unpowered and roll on the floor, so they see real travel
    // rather than what the drive wheels claim: no encoderSlip term, nothing
    // while a wall holds the robot still, and the body-frame components of a
    // slide, pivot or rebound -- the lateral part on the horizontal pod. A pod
    // off the tracking centre also rolls with the rotation; publishEncoders()
    // adds that per pod.
    if (clear && bdx == 0.0 && bdy == 0.0) {
        mPodVertIn += dDist;            // the step is along the mid heading
    } else {
        const double h = theta0 + rot * 0.5;
        const double ch = std::cos(h), sh = std::sin(h);
        mPodVertIn  +=  sx * ch + sy * sh;
        mPodHorizIn += -sx * sh + sy * ch;
    }
    mPodThetaRad += rot;

    applyRearAligner(dtSec);
}

// ============================================================================
// Contact response
// ============================================================================
//
// A blocked step is split against the surface it drives deepest into: the part
// into it is lost, the part along it slides less Coulomb drag (wallFriction
// times the push, so a face squared to the wall stops), a corner on a flat
// pivots the body flush at squareUpRate per inch of push, and a hit faster
// than kBounceMinInS starts a rebound. Candidates go most-preferred first and
// every one is collision-tested; the last is the current pose, so one always
// succeeds. A turn the surface refuses walks the body off it instead.
void World::resolveContact(double dx, double dy, double dTheta, double dtSec,
                           double& sx, double& sy, double& rot) {
    sx = sy = rot = 0.0;
    const float halfLen = 0.5f * mParams.robotLengthIn;
    const float halfWid = 0.5f * mParams.robotWidthIn;
    auto free = [&](double x, double y, double th) {
        const float t = static_cast<float>(th);
        return !field().collidesBody(static_cast<float>(x), static_cast<float>(y),
                                     std::cos(t), std::sin(t), halfLen, halfWid,
                                     mParams.collideObstacles);
    };
    /// Largest fraction in [0, 1] of (vx, vy) that keeps the pose free.
    auto fit = [&](double x, double y, double th, double vx, double vy) {
        if (vx * vx + vy * vy < 1e-14) return 0.0;
        if (free(x + vx, y + vy, th)) return 1.0;
        double lo = 0.0, hi = 1.0;
        for (int i = 0; i < kBisect; i++) {
            const double m = 0.5 * (lo + hi);
            (free(x + vx * m, y + vy * m, th) ? lo : hi) = m;
        }
        return lo;
    };

    const double x0 = mTruth.x, y0 = mTruth.y, th0 = mTruth.theta;
    const double halfDiag = std::hypot(halfLen, halfWid);
    const double reach = std::hypot(dx, dy) + halfDiag * std::fabs(dTheta) + 0.05;
    Contact cs[kMaxContacts];
    const int n = findContacts(field(), x0, y0, th0, halfLen, halfWid,
                               mParams.collideObstacles, reach, cs);
    if (n == 0) return;

    int pi = 0;
    for (int i = 1; i < n; i++) {
        if (dx * cs[i].nx + dy * cs[i].ny - cs[i].gap >
            dx * cs[pi].nx + dy * cs[pi].ny - cs[pi].gap) pi = i;
    }
    const Contact& k = cs[pi];
    const double dn = dx * k.nx + dy * k.ny;
    const double into = std::max(0.0, dn);

    // Rebound: only on a fresh hit, never on a sustained push. The body's
    // velocity into the surface becomes -restitution * (excess over the
    // threshold); any rebound already headed into this surface is dropped.
    const double bIn = mBounceVx * k.nx + mBounceVy * k.ny;
    if (bIn > 0.0) {
        mBounceVx -= bIn * k.nx;
        mBounceVy -= bIn * k.ny;
    }
    const double vIn = into / dtSec;
    if (!mInContact && mParams.wallRestitution > 0.0f && vIn > kBounceMinInS) {
        const double kick = vIn + mParams.wallRestitution * (vIn - kBounceMinInS);
        mBounceVx -= kick * k.nx;
        mBounceVy -= kick * k.ny;
    }

    // Along-surface part less Coulomb drag; an away part is kept whole.
    const double tx = dx - dn * k.nx, ty = dy - dn * k.ny;
    const double tLen = std::hypot(tx, ty);
    const double slideFrac = std::clamp(static_cast<double>(mParams.wallSlideFrac), 0.0, 1.0);
    const double mu = std::max(0.0, static_cast<double>(mParams.wallFriction));
    const double keep = tLen > 1e-12
        ? std::max(0.0, tLen - mu * into) / tLen * slideFrac : 0.0;
    const double mx = tx * keep + (dn < 0.0 ? dn * k.nx : 0.0);
    const double my = ty * keep + (dn < 0.0 ? dn * k.ny : 0.0);

    // Square-up: the reaction at a corner turns the body about that corner
    // toward the adjacent face its torque favours, until that face is flush.
    double sq = 0.0;
    if (k.cornerOnFlat && into > 0.0 && mParams.squareUpRate > 0.0f) {
        const double c = std::cos(th0), s = std::sin(th0);
        const double nbx = k.nx * c + k.ny * s, nby = -k.nx * s + k.ny * c;
        const double phi = std::atan2(nby, nbx);
        const double eX = chad::wrapRad(static_cast<float>(phi - (k.bx > 0.0 ? 0.0 : chad::PI)));
        const double eY = chad::wrapRad(static_cast<float>(phi - (k.by > 0.0 ? 0.5 : -0.5) * chad::PI));
        const double torque = -(k.bx * nby - k.by * nbx);   // of -n at the corner
        const double e = torque >= 0.0 ? std::max(eX, eY) : std::min(eX, eY);
        if (std::fabs(e) > kFlushTolRad) {
            const double step = std::min({std::fabs(e),
                mParams.squareUpRate * chad::DEG_TO_RAD * into,
                kSquareUpMaxRadS * dtSec});
            sq = std::copysign(step, e);
        }
    }

    // Outward for a turn that buries a corner: away from everything touching.
    double ox = 0.0, oy = 0.0;
    for (int i = 0; i < n; i++) {
        if (cs[i].gap > 0.05) continue;
        ox -= cs[i].nx;
        oy -= cs[i].ny;
    }
    const double oLen = std::hypot(ox, oy);

    const double cand[4][2] = {{sq, dTheta}, {sq, 0.0}, {0.0, dTheta}, {0.0, 0.0}};
    for (int ci = 0; ci < 4; ci++) {
        const double q = cand[ci][0], r = cand[ci][1];
        bool dup = false;
        for (int j = 0; j < ci; j++) dup |= (cand[j][0] == q && cand[j][1] == r);
        if (dup) continue;

        double x = x0, y = y0;
        if (q != 0.0) {
            const double rx = x0 - k.px, ry = y0 - k.py;
            const double cq = std::cos(q), sq2 = std::sin(q);
            x = k.px + rx * cq - ry * sq2;
            y = k.py + rx * sq2 + ry * cq;
        }
        const double th = th0 + q + r;
        if (!free(x, y, th)) {
            if (r == 0.0 || oLen < 1e-9) continue;
            const double out = halfDiag * std::fabs(r) + 1e-3;
            const double ux = ox / oLen * out, uy = oy / oLen * out;
            if (!free(x + ux, y + uy, th)) continue;
            double lo = 0.0, hi = 1.0;
            for (int i = 0; i < kBisect; i++) {
                const double m = 0.5 * (lo + hi);
                (free(x + ux * m, y + uy * m, th) ? hi : lo) = m;
            }
            x += ux * hi;
            y += uy * hi;
        }
        const double s = fit(x, y, th, mx, my);
        x += mx * s;
        y += my * s;
        // Approach to a skin short of contact: closer, float rounding in the
        // collision test blocks the next tick's slide and pivot.
        const double a = std::max(0.0, fit(x, y, th, into * k.nx, into * k.ny) * into - kSkinIn);
        x += k.nx * a;
        y += k.ny * a;

        sx = x - x0;
        sy = y - y0;
        rot = q + r;
        return;
    }
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

void World::publishEncoders() {
    // Always published, whether or not the library is configured to read them.
    // Enabling pods mid-run then has a live baseline instead of a jump.
    for (Pod& p : mPods) {
        // One to the right moves forward as the robot turns left; one ahead
        // of the centre moves left.
        const double travelIn = (p.horizontal ? mPodHorizIn : mPodVertIn) +
                                p.offsetIn * mPodThetaRad;
        const std::int32_t c = podCentideg(travelIn * p.gearRatio, p.wheelDiamIn);
        // The bus holds the raw value and pros::Rotation applies the reversal
        // on read, so a reversed sensor stands for one mounted the way its
        // config says: it still reads up driving forward.
        const std::int32_t d = c - p.publishedCentideg;
        bus().rotations[p.port].positionCentideg += p.reversed ? -d : d;
        p.publishedCentideg = c;
    }

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
