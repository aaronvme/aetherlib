#include "gamespecific/autons.hpp"
#include "gamespecific/lift.hpp"
#include "gamespecific/autons/tuning/support.hpp"
#include "gamespecific/claw.hpp"
#include "chad/api.hpp"
#include "pros/rtos.hpp"

namespace chad{
void autonBlue3PinFar() { reflect(autonRed3PinFar); }
}