#include "gamespecific/tuning.hpp"

#include <memory>

#include "gamespecific/lift.hpp"
#include "riku/util/logger.hpp"

namespace {
std::shared_ptr<riku::Logger> gLiftLog;
}  // namespace

void logLift(bool enabled) {
    if (!enabled) {
        liftSetTelemetry(nullptr);
        gLiftLog.reset();
        return;
    }
    gLiftLog = std::make_shared<riku::Logger>(
        "lift", std::vector<std::string>{"target", "position", "error", "pid", "ff", "power"});
    gLiftLog->start();

    auto log = gLiftLog;
    liftSetTelemetry([log](const LiftSample& s) {
        log->row({s.target, s.position, s.error, s.pid, s.feedforward, s.power});
    });
}
