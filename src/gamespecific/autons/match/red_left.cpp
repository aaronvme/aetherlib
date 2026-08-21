#include "gamespecific/autons.hpp"

#include "riku/api.hpp"

namespace riku {

void autonRedLeft() {
    chassis.setPose({0.0f, 0.0f, 0.0f});
    chassis.moveDistance(18.0f, 1500);
    chassis.turnToHeading(180.0f, 1200);
    chassis.moveDistance(12.0f, 1000);
}

}  // namespace riku
