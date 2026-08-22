#include "chad/util/tuning.hpp"

#include <memory>

#include "chad/util/logger.hpp"

namespace chad::tuning {

namespace {

// shared_ptr because the callback captures it.
std::shared_ptr<Logger> gChassisLog;
std::shared_ptr<Logger> gJamLog;
std::shared_ptr<Logger> gFeedforwardLog;

std::shared_ptr<Logger> open(std::string name, std::vector<std::string> columns) {
    auto log = std::make_shared<Logger>(std::move(name), std::move(columns));
    log->start();
    return log;
}

}  // namespace

void logChassis(Chassis& chassis, bool enabled) {
    if (!enabled) {
        chassis.setTelemetry(nullptr);
        gChassisLog.reset();
        return;
    }
    gChassisLog = open("chas", {"type", "x", "y", "thetaDeg", "targetX", "targetY",
                                "latOut", "angOut", "settled"});
    auto log = gChassisLog;
    chassis.setTelemetry([log](const ChassisSample& s) {
        log->row({static_cast<float>(s.type), s.x, s.y, s.thetaDeg,
                  s.targetX, s.targetY, s.lateralOut, s.angularOut,
                  s.settled ? 1.0f : 0.0f});
    });
}

void logJamDetector(JamDetector& jam, bool enabled) {
    if (!enabled) {
        jam.setTelemetry(nullptr);
        gJamLog.reset();
        return;
    }
    gJamLog = open("jam", {"eff0", "temp0", "cmd0", "eff1", "temp1", "cmd1",
                           "eff2", "temp2", "cmd2", "state", "jams"});
    auto log = gJamLog;
    jam.setTelemetry([log](const JamSample& s) {
        log->row({s.efficiency[0], s.temperature[0], s.command[0],
                  s.efficiency[1], s.temperature[1], s.command[1],
                  s.efficiency[2], s.temperature[2], s.command[2],
                  static_cast<float>(s.state),
                  static_cast<float>(s.jamCount)});
    });
}

void logFeedforward(FeedforwardController& controller, bool enabled) {
    if (!enabled) {
        controller.setTelemetry(nullptr);
        gFeedforwardLog.reset();
        return;
    }
    gFeedforwardLog = open("ff", {"vTarget", "wTarget", "vLeftTarget", "vRightTarget",
                                  "vLeftMeas", "vRightMeas", "aLinear", "aAngular",
                                  "voltLeft", "voltRight", "dt"});
    auto log = gFeedforwardLog;
    controller.setTelemetry([log](const FeedforwardSample& s) {
        log->row({s.vTarget, s.wTarget, s.vLeftTarget, s.vRightTarget,
                  s.vLeftMeasured, s.vRightMeasured, s.aLinear, s.aAngular,
                  s.voltLeft, s.voltRight, s.dt});
    });
}

}  // namespace chad::tuning
