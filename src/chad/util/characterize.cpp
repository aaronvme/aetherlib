#include "chad/util/characterize.hpp"

#include "chad/util/characterize_core.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

#include "pros/rtos.hpp"

namespace chad::tuning {

namespace {

constexpr float kVoltsPerPower = 12.0f / 127.0f;

}  // namespace

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

FeedforwardFit Characterizer::feedforward(bool spinInPlace) {
    FeedforwardFit fit;
    char msg[96];

    report("feedforward", spinInPlace ? "ramping (spin)" : "ramping (straight)");

    const float dt = mConfig.sampleMs / 1000.0f;
    FeedforwardAccumulator acc;
    RampSampler sampler(mConfig.accelSpanTicks, dt);
    float dir = 1.0f;
    float legStart = mDrivetrain.averagePositionDeg();
    const float degPerIn = 360.0f /
        (mDrivetrain.externalGearRatio() * PI * mDrivetrain.wheelDiameter);

    float v = 0.0f;
    while (v <= mConfig.maxVolts) {
        drive(v * dir, spinInPlace);
        pros::delay(static_cast<uint32_t>(mConfig.sampleMs));
        const float l = mDrivetrain.leftVelocityInPerSec();
        const float r = mDrivetrain.rightVelocityInPerSec();
        // Spinning, the sides oppose: magnitudes give wheel speed, which is what
        // the fit needs. The chassis speed is 0.
        const float speed = spinInPlace ? 0.5f * (std::fabs(l) + std::fabs(r))
                                        : dir * 0.5f * (l + r);

        RampSampler::Sample sample;
        if (sampler.push(speed, v, sample) &&
            sample.speed > mConfig.minSpeedInPerSec) {
            acc.add(sample.speed, sample.accel, sample.volts);
        }

        if (!spinInPlace) {
            const float travelIn =
                std::fabs(mDrivetrain.averagePositionDeg() - legStart) / degPerIn;
            if (travelIn > mConfig.maxTravelIn) {
                mDrivetrain.setVoltage(0.0f, 0.0f);
                pros::delay(350);
                dir = -dir;
                legStart = mDrivetrain.averagePositionDeg();
                // A step in every quantity being fitted.
                sampler.reset();
            }
        }
        v += (v < mConfig.rampSwitchVolts ? mConfig.rampVoltsPerSec
                                          : mConfig.rampVoltsPerSecFast) * dt;
    }
    drive(0.0f, spinInPlace);
    pros::delay(400);

    FeedforwardFitStats st;
    if (!acc.solve(st)) {
        report("feedforward", "not enough motion to fit -- raise maxVolts");
        return fit;
    }
    // Per in/s to per m/s divides by INCH_TO_METER; same for in/s^2.
    fit.kV = st.kV / INCH_TO_METER;
    fit.kA = st.kA / INCH_TO_METER;
    fit.kS = st.kS;
    fit.r2 = st.r2;
    fit.kaT = st.kaT;
    fit.samples = st.samples;
    fit.valid = fit.kV > 0.0f && fit.r2 >= mConfig.minR2;
    fit.kaValid = fit.valid && fit.kA > 0.0f && fit.kaT >= mConfig.kaMinT;

    std::snprintf(msg, sizeof(msg), "kV %.3f kS %.3f kA %.4f%s r2 %.3f n=%d",
                  fit.kV, fit.kS, fit.kA, fit.kaValid ? "" : " (kA UNFIT)",
                  fit.r2, fit.samples);
    report("feedforward", msg);
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

    // In-place rotation moves the sides opposite by theta * trackWidth / 2, so
    // their difference over the same theta is the track width.
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

    // Tolerance has to clear the worst excursion or settle never fires. The hold
    // radius measured out at about three sigma.
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

        // The robot is physically back where it began, so anything odometry
        // reports here is its own error. Split it along and across the heading
        // it started on: the two have different causes and MCL models them
        // separately.
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
    fit.alongPerIn = static_cast<float>(along / driven);
    fit.lateralPerIn = static_cast<float>(lateral / driven);
    // Straight legs turn very little, so a heading residual here is drift rather
    // than a scale error on rotation. Reported per radian only if it turned.
    fit.thetaPerRad = static_cast<float>(theta / driven);
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
            // Sentinel and error returns are not noise, they are absence.
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

    // From rest means from rest: a coasting drivetrain reads as motion at 0 V.
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
    for (float v = 0.0f; v <= mConfig.maxVolts; v += mConfig.rampVoltsPerSec * dt) {
        drive(v, spinInPlace);
        pros::delay(static_cast<uint32_t>(mConfig.sampleMs));
        const float l = mDrivetrain.leftVelocityInPerSec();
        const float r = mDrivetrain.rightVelocityInPerSec();
        const float speed = spinInPlace ? 0.5f * (std::fabs(l) + std::fabs(r))
                                        : std::fabs(0.5f * (l + r));
        // Not minSpeedInPerSec -- stiction is the thing being measured.
        if (v > 0.0f && speed > 1.0f) { volts = v; break; }
    }
    mDrivetrain.setVoltage(0.0f, 0.0f);

    std::snprintf(msg, sizeof(msg), "%.3f V", volts);
    report("breakaway", msg);
    return volts;
}

}  // namespace chad::tuning
