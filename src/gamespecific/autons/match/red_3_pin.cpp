#include "gamespecific/autons.hpp"

#include "chad/api.hpp"

namespace chad {
void autonRed3Pin() {
    chassis.setPose({65.0f, 0.0f, 90.0f});
    chassis.moveToPoint(69.0f,0.0f,500, {}, false);
    // chassis.turnToHeading(180.0f, 1200);
    // chassis.moveDistance(12.0f, 1000);
}
}
