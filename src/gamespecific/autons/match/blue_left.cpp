#include "gamespecific/autons.hpp"

#include "chad/api.hpp"

namespace chad {

void autonBlueLeft() {
    chassis.setPose({0.0f, 0.0f, 0.0f});
    chassis.moveDistance(12.0f, 1500);
    chassis.turnToHeading(270.0f, 1200);
    chassis.moveDistance(18.0f, 1000);
}

}  // namespace chad
