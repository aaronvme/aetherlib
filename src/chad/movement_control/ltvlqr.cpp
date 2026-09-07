#include "chad/movement_control/ltvlqr.hpp"

#include <algorithm>
#include <cmath>

#include "chad/chassis.hpp"
#include "chad/drivetrain.hpp"

namespace chad {
namespace {
float maxLinearInPerSec(const Drivetrain& dt) {
    return dt.rpm / 60.0f * PI * dt.wheelDiameter;
}
}

LtvLqrFollower::LtvLqrFollower(Chassis& chassis, Drivetrain& drivetrain)
    : mChassis(chassis), mDrivetrain(drivetrain),
      mFf(FeedforwardGains{}, drivetrain.trackWidth * INCH_TO_METER) {}

namespace {
float sideVelInPerSec(const pros::MotorGroup* g, const Drivetrain& dt) {
    if (g == nullptr) return 0.0f;
    const double rpm = g->get_actual_velocity();
    if (!std::isfinite(rpm)) return 0.0f;
    return static_cast<float>(rpm) * dt.externalGearRatio() / 60.0f * PI * dt.wheelDiameter;
}
}

void LtvLqrFollower::followPath(const std::vector<Pose>& path,
                                const LtvLqrParams& params, float timeoutMs) {
    if (path.size() < 2 || mRunning) return;
    mPeakError = 0.0f;
    mFinalError = 0.0f;

    const float dt = params.updatePeriodMs / 1000.0f;
    const float vMaxIn = maxLinearInPerSec(mDrivetrain);

    if (!mSchedule.ready() || mBuiltDt != dt ||
        mBuiltWith.qx != params.weights.qx || mBuiltWith.qy != params.weights.qy ||
        mBuiltWith.qtheta != params.weights.qtheta ||
        mBuiltWith.rv != params.weights.rv || mBuiltWith.romega != params.weights.romega) {
        mSchedule.build(params.weights, dt, vMaxIn);
        mBuiltWith = params.weights;
        mBuiltDt = dt;
    }

    const std::size_t n = path.size();
    std::vector<float> cum(n, 0.0f), v(n, 0.0f);
    for (std::size_t i = 1; i < n; i++) {
        cum[i] = cum[i - 1] + distance(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
    }

    const float cruise = vMaxIn * (params.maxSpeed / 127.0f);
    std::fill(v.begin(), v.end(), cruise);
    if (params.useVelocityProfile) {
        for (std::size_t i = 1; i + 1 < n; i++) {
            const float seg = distance(path[i-1].x, path[i-1].y, path[i+1].x, path[i+1].y);
            if (seg <= 1e-3f) continue;
            const float curv = std::fabs(angleDiffRad(path[i-1].theta, path[i+1].theta)) / seg;
            if (curv > 1e-5f && params.maxLateralAccel > 0.0f) {
                v[i] = std::min(v[i], std::sqrt(params.maxLateralAccel / curv));
            }
        }
        v.front() = 0.0f;
        v.back() = 0.0f;
        for (std::size_t i = 1; i < n; i++) {
            const float ds = cum[i] - cum[i - 1];
            v[i] = std::min(v[i], std::sqrt(v[i-1]*v[i-1] + 2.0f*params.maxAccel*ds));
        }
        for (std::size_t i = n - 1; i > 0; i--) {
            const float ds = cum[i] - cum[i - 1];
            v[i-1] = std::min(v[i-1], std::sqrt(v[i]*v[i] + 2.0f*params.maxAccel*ds));
        }
    }

    const float minIn = vMaxIn * (params.minSpeed / 127.0f);
    const float outLimit = vMaxIn * (params.maxSpeed / 127.0f);
    float peakPos = 0.0f;
    {
        std::size_t pk = 0;
        for (std::size_t i = 1; i < n; i++) if (v[i] > v[pk]) pk = i;
        peakPos = cum[pk];
    }
    if (params.useVelocityController) {
        mFf.setGains(params.ffGains);
        mFf.setTrackWidth(mDrivetrain.trackWidth * INCH_TO_METER);

        mFf.setMaxVoltage(12.0f * params.maxSpeed / 127.0f);
        mFf.reset();
    }
    mSlewLeft.setRates(params.slewRisePerSec, params.slewFallPerSec);
    mSlewRight.setRates(params.slewRisePerSec, params.slewFallPerSec);
    mSlewLeft.reset(0.0f);
    mSlewRight.reset(0.0f);
    mRunning = true;
    mCancel = false;
    const uint32_t start = pros::millis();

    uint32_t prevTick = pros::millis();
    while (!mCancel) {
        if (timeoutMs > 0.0f && pros::millis() - start > timeoutMs) break;
        const uint32_t tickStart = pros::millis();
        float tickDt = (tickStart - prevTick) / 1000.0f;
        if (tickDt <= 0.0f || tickDt > 0.1f) tickDt = dt;
        prevTick = tickStart;

        const Pose pose = mChassis.getPose();

        std::size_t best = 0;
        float bestD = 1e18f;
        for (std::size_t i = 0; i < n; i++) {
            const float ddx = path[i].x - pose.x, ddy = path[i].y - pose.y;
            const float d = ddx * ddx + ddy * ddy;
            if (d < bestD) { bestD = d; best = i; }
        }
        const float cross = std::sqrt(bestD);
        mPeakError = std::max(mPeakError, cross);

        const float toEnd = distance(path.back().x, path.back().y, pose.x, pose.y);
        const bool headingOk =
            angleDistRad(pose.theta, path.back().theta) < degToRad(params.headingTolerance);
        const float measuredSpeed =
            0.5f * (sideVelInPerSec(mDrivetrain.leftMotors, mDrivetrain) +
                    sideVelInPerSec(mDrivetrain.rightMotors, mDrivetrain));

        const bool stalled = std::fabs(measuredSpeed) < params.stallSpeed;
        if (toEnd < params.targetTolerance && (headingOk || stalled)) break;

        const Pose& ref = path[best];

        float vRef = (cum[best] < peakPos) ? std::max(v[best], minIn) : v[best];

        float omegaRef = 0.0f;
        if (best + 1 < n) {
            const float ds = cum[best + 1] - cum[best];
            if (ds > 1e-4f) {
                omegaRef = angleDiffRad(ref.theta, path[best + 1].theta) / ds * vRef;
            }
        }

        const float dx = ref.x - pose.x, dy = ref.y - pose.y;
        const float c = std::cos(pose.theta), s = std::sin(pose.theta);
        const float ex =  dx * c + dy * s;
        const float ey = -dx * s + dy * c;
        const float eth = angleDiffRad(pose.theta, ref.theta);

        const LqrGain g = mSchedule.at(vRef);
        const float dv = g.k[0] * ex + g.k[1] * ey + g.k[2] * eth;
        const float dw = g.k[3] * ex + g.k[4] * ey + g.k[5] * eth;

        float vCmd = vRef + dv;
        float wCmd = omegaRef + dw;

        float left  = vCmd - wCmd * mDrivetrain.trackWidth * 0.5f;
        float right = vCmd + wCmd * mDrivetrain.trackWidth * 0.5f;
        const float peak = std::max(std::fabs(left), std::fabs(right));
        if (peak > outLimit) {
            const float sc = outLimit / peak;
            left *= sc; right *= sc;
            vCmd *= sc; wCmd *= sc;
        }

        float vl, vr;
        if (params.useVelocityController) {
            const DrivetrainVoltages out = mFf.update(
                vCmd * INCH_TO_METER, wCmd,
                sideVelInPerSec(mDrivetrain.leftMotors, mDrivetrain) * INCH_TO_METER,
                sideVelInPerSec(mDrivetrain.rightMotors, mDrivetrain) * INCH_TO_METER,
                tickDt);
            vl = out.left;
            vr = out.right;
        } else {
            const float k = 12.0f / vMaxIn;
            vl = left * k;
            vr = right * k;
        }
        if (params.slewRisePerSec > 0.0f || params.slewFallPerSec > 0.0f) {
            vl = mSlewLeft.update(vl, tickDt);
            vr = mSlewRight.update(vr, tickDt);
        }
        const DrivetrainVoltages out{vl, vr};
        mDrivetrain.setVoltage(out.leftPower(), out.rightPower());

        const int32_t elapsed = pros::millis() - tickStart;
        const int32_t rest = static_cast<int32_t>(params.updatePeriodMs) - elapsed;
        pros::delay(rest > 1 ? rest : 1);
    }

    mDrivetrain.setVoltage(0.0f, 0.0f);
    const Pose end = mChassis.getPose();
    mFinalError = distance(path.back().x, path.back().y, end.x, end.y);
    mRunning = false;
}
}
