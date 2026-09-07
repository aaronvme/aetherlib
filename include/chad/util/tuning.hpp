#pragma once

#include "chad/chassis.hpp"
#include "chad/movement_control/feedforward.hpp"
#include "chad/util/jam_detector.hpp"

namespace chad::tuning {
void logChassis(Chassis& chassis, bool enabled);

void logJamDetector(JamDetector& jam, bool enabled);

void logFeedforward(FeedforwardController& controller, bool enabled);
}
