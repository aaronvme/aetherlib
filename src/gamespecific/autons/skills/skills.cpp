#include "gamespecific/autons.hpp"

#include "chad/api.hpp"

namespace chad {

void autonSkills() {
    chassis.setPose({0.0f, 0.0f, 0.0f});
    chassis.moveDistance(30.0f, 2000);
    chassis.turnToHeading(90.0f, 1200);
    chassis.moveToPoint(24.0f, 24.0f, 2500);
}

}  // namespace chad
