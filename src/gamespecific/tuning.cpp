#include "gamespecific/tuning.hpp"

#include <memory>

#include "gamespecific/lift.hpp"
#include "chad/util/logger.hpp"

namespace {
std::shared_ptr<chad::Logger> gLiftLog;
}  // namespace

void logLift(bool enabled) {
    if (!enabled) {
        liftSetTelemetry(nullptr);
        gLiftLog.reset();
        return;
    }
    gLiftLog = std::make_shared<chad::Logger>(
        "lift", std::vector<std::string>{"target", "position", "error", "pid", "ff", "power"});
    gLiftLog->start();

    auto log = gLiftLog;
    liftSetTelemetry([log](const LiftSample& s) {
        log->row({s.target, s.position, s.error, s.pid, s.feedforward, s.power});
    });
}
