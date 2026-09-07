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

void autonCalibrateMounts() {
    Report out;
    out("mounts: sensor y offsets");
    if (!headingOk(out)) return;
    out("square to a wall, 20-40 in");
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
    scfg.verbose = false;
    SensorCalibrator cal(chassis, scfg);

    out("%d sensors, ~%d s", static_cast<int>(cfgs.size()),
        static_cast<int>(cfgs.size() * 9));
    const auto results = cal.calibrateAll(cfgs);

    int applied = 0;
    for (size_t k = 0; k < results.size(); ++k) {
        const CalibrationResult& r = results[k];
        const int i = slots[k];
        if (!r.valid || r.samplesUsed < 3) {
            out.result("s%d FAILED n=%d", i, r.samplesUsed);
            continue;
        }

        out.result("s%d across %+.2f r%.2f", i, r.offsetIn, r.confidence);
        tuning::applyMountOffset(sensorMounts[i].theta, r.offsetIn,
                                 &sensorMounts[i].x, &sensorMounts[i].y);
        out.result("s%d -> x%+.2f y%+.2f", i, sensorMounts[i].x,
                   sensorMounts[i].y);
        applied++;
    }

    if (applied == 0) {
        out.result("nothing solved. is the wall");
        out.result("flat and in range at +-30?");
        return;
    }
    out.result("copy the y values into");
    out.result("sensorMounts in config.cpp.");

    out.result("reboot for MCL to use them");
    out.result("r under ~0.05 is a good fit");
}
}
