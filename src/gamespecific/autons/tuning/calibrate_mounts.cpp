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
constexpr float kSensorSpanXIn = 2.0f;   // front sensor face -> back face
constexpr float kSensorSpanYIn = 9.5f;   // left sensor face -> right face

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
    waitForStart(out, "square to a wall, 20-40 in");
    out("it turns a full circle");
    setFilters(false);

    const float base = stdRadToVexDeg(chassis.getPose().theta);

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
    scfg.verbose = false;
    SensorCalibrator cal(chassis, scfg);

    out("%d sensors, ~%d s", static_cast<int>(cfgs.size()),
        static_cast<int>(cfgs.size() * 9));
    const auto results = cal.calibrateAll(cfgs);

    // The intercepts are only comparable across sensors if they came from the
    // same spin at the same wall, so gather them before anything is applied.
    CalibrationResult bySlot[MAX_DISTANCE_SENSORS];
    for (size_t k = 0; k < results.size(); ++k) bySlot[slots[k]] = results[k];

    float wallDist = 0.0f;
    bool haveDist = solveWallDistance(bySlot, kFront, kBack, kSensorSpanXIn,
                                      wallDist);
    float wallDistY = 0.0f;
    const bool haveY = solveWallDistance(bySlot, kLeft, kRight, kSensorSpanYIn,
                                         wallDistY);
    if (!haveDist && haveY) {
        wallDist = wallDistY;
        haveDist = true;
    }

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
        out.result("kSensorSpanXIn, rerun");
    } else {
        out.result("wall %.2f in from centre", wallDist);
        if (haveY && kSensorSpanXIn > 0.0f) {
            out.result("x/y spans disagree %+.2f", wallDist - wallDistY);
        }
    }
    out.result("copy BOTH x and y into");
    out.result("sensorMounts in config.cpp.");

    out.result("reboot for MCL to use them");
    out.result("r under ~0.05 is a good fit");
}
}
