#include "chad/util/characterize.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

#include "pros/rtos.hpp"

namespace chad::tuning {
namespace {
constexpr float kVoltsPerPower = 12.0f / 127.0f;
}

Characterizer::Characterizer(Chassis& chassis, Drivetrain& drivetrain,
                             const CharacterizeConfig& config)
    : mChassis(chassis), mDrivetrain(drivetrain), mConfig(config) {}

void Characterizer::report(const char* stage, const char* detail) const {
    if (mConfig.onProgress) mConfig.onProgress(stage, detail);
}

void Characterizer::drive(float volts, bool spin) {
    const float power = volts / kVoltsPerPower;
    mDrivetrain.setVoltage(power, spin ? -power : power);
}
namespace {
float wheelSpeed(Drivetrain& dt, bool spin, float dir) {
    const float l = dt.leftVelocityInPerSec();
    const float r = dt.rightVelocityInPerSec();
    return spin ? 0.5f * (std::fabs(l) + std::fabs(r)) : dir * 0.5f * (l + r);
}
}

FeedforwardFit Characterizer::feedforward(bool spinInPlace) {
    FeedforwardFit fit;
    char msg[96];

    const float dt = mConfig.sampleMs / 1000.0f;
    const int holdN = static_cast<int>(mConfig.holdMs / mConfig.sampleMs);
    const int dwellN = static_cast<int>(mConfig.dwellMs / mConfig.sampleMs);
    if (dt <= 0.0f || holdN < 2 || dwellN <= holdN) {
        report("feedforward", "dwellMs must exceed holdMs, both over sampleMs");
        return fit;
    }
    const float degPerIn = 360.0f /
        (mDrivetrain.externalGearRatio() * PI * mDrivetrain.wheelDiameter);

    const float vLo = breakawayVolts(spinInPlace) + 0.3f;
    report("feedforward", spinInPlace ? "staircase (spin)" : "staircase (straight)");

    std::vector<VelocityPoint> levels;
    std::vector<float> hold;
    const float startDeg = mDrivetrain.averagePositionDeg();
    int skipped = 0;

    for (int i = 0; i < mConfig.staircaseLevels; i++) {
        const float frac =
            mConfig.staircaseLevels > 1
                ? static_cast<float>(i) /
                      static_cast<float>(mConfig.staircaseLevels - 1)
                : 0.0f;
        const float v = vLo + frac * (mConfig.maxVolts - vLo);
        if (v <= 0.0f) continue;

        const float net = (mDrivetrain.averagePositionDeg() - startDeg) / degPerIn;
        const float dir = spinInPlace ? 1.0f : (net > 0.0f ? -1.0f : 1.0f);
        const float legStart = mDrivetrain.averagePositionDeg();

        hold.clear();
        bool ranOutOfRoom = false;
        for (int k = 0; k < dwellN; k++) {
            drive(v * dir, spinInPlace);
            pros::delay(static_cast<uint32_t>(mConfig.sampleMs));
            if (k >= dwellN - holdN) {
                hold.push_back(wheelSpeed(mDrivetrain, spinInPlace, dir));
            }
            if (!spinInPlace) {
                const float travel =
                    std::fabs(mDrivetrain.averagePositionDeg() - legStart) / degPerIn;
                if (travel > mConfig.maxTravelIn) { ranOutOfRoom = true; break; }
            }
        }
        mDrivetrain.setVoltage(0.0f, 0.0f);
        pros::delay(250);

        if (ranOutOfRoom || static_cast<int>(hold.size()) < holdN) {
            skipped++;
            continue;
        }
        const float mean = meanOverWindow(hold.data(), static_cast<int>(hold.size()));
        if (mean <= mConfig.minSpeedInPerSec) { skipped++; continue; }
        levels.push_back({mean, v});
    }

    const VelocityFit vel =
        fitVelocity(levels.data(), static_cast<int>(levels.size()), mConfig.minR2);
    if (vel.levels < 4) {
        std::snprintf(msg, sizeof(msg), "only %d steady levels of %d -- more room?",
                      vel.levels, mConfig.staircaseLevels);
        report("feedforward", msg);
        return fit;
    }

    report("feedforward", "steps");
    const int slopeN = static_cast<int>(mConfig.slopeWindowMs / mConfig.sampleMs);
    const int stepN = static_cast<int>(mConfig.stepMs / mConfig.sampleMs);
    std::vector<StepSample> steps;
    std::vector<float> trace;

    for (int rep = 0; rep < mConfig.stepReps && rep < MAX_STEP_REPS; rep++) {
        const float net = (mDrivetrain.averagePositionDeg() - startDeg) / degPerIn;
        const float dir = spinInPlace ? 1.0f : (net > 0.0f ? -1.0f : 1.0f);
        trace.clear();
        for (int k = 0; k < stepN; k++) {
            drive(mConfig.stepVolts * dir, spinInPlace);
            pros::delay(static_cast<uint32_t>(mConfig.sampleMs));
            trace.push_back(wheelSpeed(mDrivetrain, spinInPlace, dir));
        }
        mDrivetrain.setVoltage(0.0f, 0.0f);
        pros::delay(400);

        int first = -1;
        for (int k = 0; k < static_cast<int>(trace.size()); k++) {
            if (trace[k] > mConfig.minSpeedInPerSec) { first = k; break; }
        }
        if (first < 0 || first + slopeN > static_cast<int>(trace.size())) continue;
        const float* w = trace.data() + first;
        steps.push_back({meanOverWindow(w, slopeN),
                         slopeOverWindow(w, slopeN, dt), mConfig.stepVolts});
    }

    const AccelFit acc = fitAccel(steps.data(), static_cast<int>(steps.size()),
                                  vel.kS, vel.kV, mConfig.maxKaSpread);

    fit.kV = vel.kV / INCH_TO_METER;
    fit.kA = acc.kA / INCH_TO_METER;
    fit.kS = vel.kS;
    fit.r2 = vel.r2;
    fit.kaSpread = acc.spread;
    fit.speedSpanInPerSec = vel.speedSpan;
    fit.samples = vel.levels;
    fit.stepReps = acc.reps;
    fit.valid = vel.valid;
    fit.kaValid = vel.valid && acc.valid;

    const float rated = mDrivetrain.rpm / 60.0f * PI * mDrivetrain.wheelDiameter;
    fit.freeSpeedRatio = freeSpeedRatio(vel.kS, vel.kV, rated);
    fit.drivetrainSuspect = fit.freeSpeedRatio < mConfig.freeSpeedLo ||
                            fit.freeSpeedRatio > mConfig.freeSpeedHi;

    std::snprintf(msg, sizeof(msg), "kV %.4f kS %.3f r2 %.3f lev %d/%d", fit.kV,
                  fit.kS, fit.r2, vel.levels, vel.levels + skipped);
    report("feedforward", msg);
    if (fit.kaValid) {
        std::snprintf(msg, sizeof(msg), "kA %.4f spread %.2f over %d reps", fit.kA,
                      acc.spread, acc.reps);
    } else {
        std::snprintf(msg, sizeof(msg), "kA UNFIT (%.4f spread %.2f, %d reps)",
                      fit.kA, acc.spread, acc.reps);
    }
    report("feedforward", msg);
    if (fit.drivetrainSuspect) {
        std::snprintf(msg, sizeof(msg), "free speed %.0f%% of rated -- check motors",
                      100.0f * fit.freeSpeedRatio);
        report("feedforward", msg);
    }
    return fit;
}

float Characterizer::trackWidth() {
    char msg[96];
    report("trackWidth", "spinning");

    const float l0 = mDrivetrain.leftPositionIn();
    const float r0 = mDrivetrain.rightPositionIn();
    float prevHeading = mChassis.odom().imuHeadingRad();
    double turned = 0.0;
    const double target = 2.0 * PI * mConfig.spinRotations;

    const uint32_t start = pros::millis();
    while (turned < target && pros::millis() - start < 30000u) {
        drive(4.0f, true);
        pros::delay(static_cast<uint32_t>(mConfig.sampleMs));
        const float h = mChassis.odom().imuHeadingRad();
        turned += std::fabs(angleDiffRad(prevHeading, h));
        prevHeading = h;
    }
    mDrivetrain.setVoltage(0.0f, 0.0f);
    pros::delay(500);

    const float dl = mDrivetrain.leftPositionIn() - l0;
    const float dr = mDrivetrain.rightPositionIn() - r0;
    if (turned < 1.0) {
        report("trackWidth", "robot did not turn -- check the IMU");
        return mDrivetrain.trackWidth;
    }
    const float width = std::fabs(dr - dl) / static_cast<float>(turned);

    std::snprintf(msg, sizeof(msg), "%.3f in over %.1f rev (was %.3f)", width,
                  turned / (2.0 * PI), mDrivetrain.trackWidth);
    report("trackWidth", msg);
    return width;
}

float Characterizer::measureScale(float trueDistanceIn, float commandDistanceIn) {
    char msg[96];
    report("scale", "driving");

    const float before = mDrivetrain.averagePositionDeg();
    MoveDistanceParams params;
    mChassis.moveDistance(commandDistanceIn, 5000, params);
    const float after = mDrivetrain.averagePositionDeg();
    pros::delay(300);

    const float reported = (after - before) / 360.0f *
                           mDrivetrain.externalGearRatio() * PI *
                           mDrivetrain.wheelDiameter;
    if (std::fabs(reported) < 1e-3f || trueDistanceIn <= 0.0f) {
        report("scale", "no travel recorded");
        return 1.0f;
    }
    const float factor = trueDistanceIn / reported;
    std::snprintf(msg, sizeof(msg), "reported %.2f, true %.2f, factor %.4f",
                  reported, trueDistanceIn, factor);
    report("scale", msg);
    return factor;
}

NoiseFit Characterizer::localizationNoise() {
    NoiseFit out;
    char msg[96];
    report("noise", "parked, sampling");

    mDrivetrain.setVoltage(0.0f, 0.0f);
    std::vector<Pose> poses;
    const int n = static_cast<int>(mConfig.parkSeconds * 1000.0f / mConfig.sampleMs);
    for (int i = 0; i < n; i++) {
        poses.push_back(mChassis.getPose());
        pros::delay(static_cast<uint32_t>(mConfig.sampleMs));
    }
    if (poses.size() < 8) return out;

    double mx = 0, my = 0, mt = 0;
    for (const Pose& p : poses) { mx += p.x; my += p.y; mt += p.theta; }
    mx /= poses.size(); my /= poses.size(); mt /= poses.size();

    double var = 0, vart = 0;
    for (const Pose& p : poses) {
        const double dx = p.x - mx, dy = p.y - my;
        const double d2 = dx * dx + dy * dy;
        var += d2;
        out.peakIn = std::max(out.peakIn, static_cast<float>(std::sqrt(d2)));
        const double dt = angleDiffRad(static_cast<float>(mt), p.theta);
        vart += dt * dt;
    }
    out.samples = static_cast<int>(poses.size());
    out.sigmaIn = static_cast<float>(std::sqrt(var / poses.size()));
    out.sigmaThetaDeg = static_cast<float>(std::sqrt(vart / poses.size())) *
                        180.0f / PI;

    out.suggestTolerance = std::max(1.0f, out.peakIn * 1.5f);
    out.suggestBearingHold = std::max(1.0f, out.sigmaIn * 3.0f);

    std::snprintf(msg, sizeof(msg), "sigma %.3f in / %.2f deg, peak %.3f in",
                  out.sigmaIn, out.sigmaThetaDeg, out.peakIn);
    report("noise", msg);
    return out;
}

OdomDriftFit Characterizer::measureOdomDrift(float legIn, int reps) {
    OdomDriftFit fit;
    char msg[96];
    report("odomDrift", "out and back");

    double along = 0.0, lateral = 0.0, theta = 0.0, driven = 0.0;
    for (int i = 0; i < reps; i++) {
        const Pose start = mChassis.getPose();
        mChassis.moveDistance(legIn, 6000);
        mChassis.moveDistance(-legIn, 6000);
        pros::delay(300);
        const Pose end = mChassis.getPose();

        const float dx = end.x - start.x;
        const float dy = end.y - start.y;
        const float c = std::cos(start.theta), sn = std::sin(start.theta);
        along += std::fabs(dx * c + dy * sn);
        lateral += std::fabs(-dx * sn + dy * c);
        theta += std::fabs(angleDiffRad(start.theta, end.theta));
        driven += 2.0 * legIn;
        fit.reps++;
    }
    if (driven < 1.0) {
        report("odomDrift", "no travel recorded");
        return fit;
    }
    fit.distanceIn = static_cast<float>(driven);

    fit.alongPerIn = sigmaFromMeanAbs(static_cast<float>(along / driven));
    fit.lateralPerIn = sigmaFromMeanAbs(static_cast<float>(lateral / driven));

    fit.thetaPerIn = sigmaFromMeanAbs(static_cast<float>(theta / driven));
    fit.valid = true;

    std::snprintf(msg, sizeof(msg), "along %.4f/in  lateral %.4f/in over %.0f in",
                  fit.alongPerIn, fit.lateralPerIn, fit.distanceIn);
    report("odomDrift", msg);
    return fit;
}

RangeNoiseFit Characterizer::measureRangeNoise(
    const std::array<pros::Distance*, MAX_DISTANCE_SENSORS>& sensors) {
    RangeNoiseFit fit;
    char msg[96];
    report("rangeNoise", "parked, sampling beams");

    mDrivetrain.setVoltage(0.0f, 0.0f);
    const int n = static_cast<int>(mConfig.parkSeconds * 1000.0f / mConfig.sampleMs);
    double sum[MAX_DISTANCE_SENSORS]{}, sumSq[MAX_DISTANCE_SENSORS]{};

    for (int k = 0; k < n; k++) {
        for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
            if (sensors[i] == nullptr) continue;
            const double mm = sensors[i]->get_distance();

            if (!std::isfinite(mm) || mm <= 0.0 || mm >= 9000.0) continue;
            const double in = mm / 25.4;
            sum[i] += in;
            sumSq[i] += in * in;
            fit.samples[i]++;
        }
        pros::delay(static_cast<uint32_t>(mConfig.sampleMs));
    }
    for (int i = 0; i < MAX_DISTANCE_SENSORS; i++) {
        if (fit.samples[i] < 8) continue;
        const double m = sum[i] / fit.samples[i];
        const double var = sumSq[i] / fit.samples[i] - m * m;
        fit.meanIn[i] = static_cast<float>(m);
        fit.sigmaIn[i] = static_cast<float>(std::sqrt(var > 0.0 ? var : 0.0));
        fit.worstSigmaIn = std::max(fit.worstSigmaIn, fit.sigmaIn[i]);
        std::snprintf(msg, sizeof(msg), "beam %d: mean %.2f in, sigma %.3f in (n=%d)",
                      i, fit.meanIn[i], fit.sigmaIn[i], fit.samples[i]);
        report("rangeNoise", msg);
    }
    return fit;
}

float Characterizer::breakawayVolts(bool spinInPlace) {
    char msg[96];

    report("breakaway", "waiting for rest");
    mDrivetrain.setVoltage(0.0f, 0.0f);
    for (int i = 0; i < 300; ++i) {
        pros::delay(10);
        if (std::fabs(mDrivetrain.leftVelocityInPerSec()) < 0.5f &&
            std::fabs(mDrivetrain.rightVelocityInPerSec()) < 0.5f) {
            break;
        }
    }

    report("breakaway", "ramping from rest");
    const float dt = mConfig.sampleMs / 1000.0f;
    float volts = 0.0f;
    for (float v = 0.0f; v <= mConfig.maxVolts; v += mConfig.breakawayVoltsPerSec * dt) {
        drive(v, spinInPlace);
        pros::delay(static_cast<uint32_t>(mConfig.sampleMs));
        const float l = mDrivetrain.leftVelocityInPerSec();
        const float r = mDrivetrain.rightVelocityInPerSec();
        const float speed = spinInPlace ? 0.5f * (std::fabs(l) + std::fabs(r))
                                        : std::fabs(0.5f * (l + r));

        if (v > 0.0f && speed > 1.0f) { volts = v; break; }
    }
    mDrivetrain.setVoltage(0.0f, 0.0f);

    std::snprintf(msg, sizeof(msg), "%.3f V", volts);
    report("breakaway", msg);
    return volts;
}
}
