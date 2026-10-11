#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "chad/api.hpp"
#include "chad/util/mount_offset.hpp"
#include "chad/util/sensor_calibrator.hpp"

namespace chad {
using namespace autons;

namespace {
constexpr float kSensorSpanXIn = 1.75f;   // front sensor face -> back face
constexpr float kSensorSpanYIn = 11.25f;   // left sensor face -> right face

// The robot starts with its front bumper flat on the wall and backs off this
// far before sweeping.
constexpr float kBackOffIn = 15.0f;
// Turning centre to the face of the front bumper that touches the wall, along
// the robot. With it, the wall distance comes from the back-off itself and the
// spans above are only a cross-check; 0 leaves it to the spans.
constexpr float kFrontToCentreIn = 0.0f;

constexpr int kFront = 0, kLeft = 1, kBack = 2, kRight = 3;

// D = (interceptA + interceptB + meanCos * span) / 2
bool solveWallDistance(const CalibrationResult* r, int a, int b, float spanIn,
                       float& distOut) {
    if (spanIn <= 0.0f) return false;
    if (!r[a].valid || !r[b].valid) return false;
    const float meanCos = 0.5f * (r[a].meanCosTheta + r[b].meanCosTheta);
    if (!(meanCos > 0.1f)) return false;
    distOut = 0.5f * (r[a].interceptIn + r[b].interceptIn + meanCos * spanIn);
    return true;
}
}

void autonCalibrateMounts() {
    Report out;
    out("mounts: sensor y offsets");
    if (!headingOk(out)) return;
    waitForStart(out, "front bumper flat on a wall");
    out("backs off %.0f in, turns a full circle", kBackOffIn);
    setFilters(false);

    // Flush on the wall the robot is square to it by construction, so this
    // heading is the wall's normal and every sweep is measured from it --
    // not from wherever the back-off happens to leave the heading.
    const Pose atWall = chassis.getPose();
    const float base = stdRadToVexDeg(atWall.theta);
    chassis.moveDistance(-kBackOffIn, 3000, {.maxSpeed = 50.0f});
    pros::delay(300);
    const Pose offWall = chassis.getPose();
    const float backedIn = distance(atWall.x, atWall.y, offWall.x, offWall.y);
    out("backed off %.2f in", backedIn);

    std::vector<CalibrationConfig> cfgs;
    std::vector<int> slots;
    for (int i = 0; i < MAX_DISTANCE_SENSORS; ++i) {
        if (distSensors[i] == nullptr) continue;
        CalibrationConfig c;
        c.name = std::to_string(i);
        c.sensor = distSensors[i];
        c.baseHeadingDeg =
            std::fmod(base + radToDeg(sensorMounts[i].theta) + 360.0f, 360.0f);
        cfgs.push_back(c);
        slots.push_back(i);
    }
    if (cfgs.empty()) {
        out.result("no distance sensors configured");
        return;
    }

    SensorCalibrator::Config scfg;
    scfg.sweepAnglesDeg = {-30.0f, -15.0f, 0.0f, 15.0f, 30.0f};
    // The default second is not enough for the 90 deg hop between sensors.
    // The reading cap only has to reject a no-return; a wall that is further
    // than expected should still be fitted, and the residual will say if it
    // was not a wall.
    scfg.turnTimeoutMs = 2500.0f;
    scfg.maxValidReadingMm = 2600.0f;
    // waitUntilStill() covers the settling this delay used to guess at.
    scfg.postTurnDelayMs = 200;
    scfg.verbose = false;
    SensorCalibrator cal(chassis, scfg);

    out("%d sensors, ~%d s", static_cast<int>(cfgs.size()),
        static_cast<int>(cfgs.size() * 9));
    const auto results = cal.calibrateAll(cfgs);

    // The intercepts are only comparable across sensors if they came from the
    // same spin at the same wall, so gather them before anything is applied.
    CalibrationResult bySlot[MAX_DISTANCE_SENSORS];
    for (size_t k = 0; k < results.size(); ++k) bySlot[slots[k]] = results[k];

    float wallDistX = 0.0f;
    const bool haveX = solveWallDistance(bySlot, kFront, kBack, kSensorSpanXIn,
                                         wallDistX);
    float wallDistY = 0.0f;
    const bool haveY = solveWallDistance(bySlot, kLeft, kRight, kSensorSpanYIn,
                                         wallDistY);

    // Bumper on the wall plus a measured back-off is a tape measurement the
    // robot took itself, so it wins over the spans when it is set up.
    const bool haveDrive = kFrontToCentreIn > 0.0f;
    float wallDist = 0.0f;
    bool haveDist = true;
    if (haveDrive)  wallDist = kFrontToCentreIn + backedIn;
    else if (haveX) wallDist = wallDistX;
    else if (haveY) wallDist = wallDistY;
    else            haveDist = false;

    int applied = 0;
    for (size_t k = 0; k < results.size(); ++k) {
        const CalibrationResult& r = results[k];
        const int i = slots[k];
        if (!r.valid || r.samplesUsed < 3) {
            out.result("s%d FAILED n=%d", i, r.samplesUsed);
            continue;
        }

        out.result("s%d across %+.2f r%.2f", i, r.offsetIn, r.confidence);
        if (haveDist && r.meanCosTheta > 0.1f) {
            const float along = (wallDist - r.interceptIn) / r.meanCosTheta;
            tuning::setMountFromBeam(sensorMounts[i].theta, along, r.offsetIn,
                                     &sensorMounts[i].x, &sensorMounts[i].y);
        } else {
            tuning::applyMountOffset(sensorMounts[i].theta, r.offsetIn,
                                     &sensorMounts[i].x, &sensorMounts[i].y);
        }
        out.result("s%d -> x%+.2f y%+.2f", i, sensorMounts[i].x,
                   sensorMounts[i].y);
        applied++;
    }

    if (applied == 0) {
        out.result("nothing solved. is the wall");
        out.result("flat and in range at +-30?");
        return;
    }
    if (!haveDist) {
        out.result("along NOT measured: set");
        out.result("kFrontToCentreIn, rerun");
    } else {
        out.result("wall %.2f in from centre (%s)", wallDist,
                   haveDrive ? "back-off" : "spans");
        // Each check is against whatever wallDist came from, so a big number
        // points at that span (or kFrontToCentreIn) being mismeasured.
        if (haveX && (haveDrive || haveY)) {
            out.result("front/back span %+.2f", wallDistX - wallDist);
        }
        if (haveY && (haveDrive || haveX)) {
            out.result("left/right span %+.2f", wallDistY - wallDist);
        }
    }
    out.result("copy BOTH x and y into");
    out.result("sensorMounts in config.cpp.");

    out.result("reboot for MCL to use them");
    out.result("r under ~0.05 is a good fit");
}
}
