#include "gamespecific/autons.hpp"

#include "riku/api.hpp"

namespace riku {

void autonExample() {
    chassis.setPose({0.0f, 0.0f, 0.0f});

    chassis.moveDistance(24.0f, 2000,
                         {.forwards = true,
                          .maxSpeed = 100.0f,
                          .minSpeed = 20.0f,
                          .targetTolerance = 1.0f,
                          .earlyExitRange = 0.0f},
                         false);

    chassis.moveToPoint(36.0f, 24.0f, 3000,
                        {.forwards = true,
                         .maxSpeed = 110.0f,
                         .minSpeed = 15.0f,
                         .targetTolerance = 1.5f,
                         .earlyExitRange = 0.0f,
                         .lead = 4.0f},
                        false);

    chassis.turnToHeading(90.0f, 1500,
                          {.direction = TurnDirection::SHORTEST,
                           .maxSpeed = 90.0f,
                           .minSpeed = 10.0f,
                           .targetTolerance = 2.0f},
                          false);

    chassis.turnToPoint(0.0f, 0.0f, 1500,
                        {.direction = TurnDirection::SHORTEST,
                         .maxSpeed = 90.0f,
                         .minSpeed = 10.0f,
                         .targetTolerance = 3.0f,
                         .offset = 0.0f},
                        false);

    chassis.moveArc(24.0f, 24.0f, 20.0f, 3000,
                    {.forwards = true,
                     .maxSpeed = 90.0f,
                     .minSpeed = 10.0f,
                     .targetTolerance = 1.5f,
                     .lead = 3.0f},
                    false);

    chassis.moveBoomerang(48.0f, 24.0f, 180.0f, 3500,
                          {.forwards = true,
                           .maxSpeed = 90.0f,
                           .minSpeed = 10.0f,
                           .targetTolerance = 2.0f,
                           .headingTolerance = 3.0f,
                           .lead = 0.6f,
                           .leadDecay = 0.5f},
                          false);

    std::vector<Pose> path = {
        {0.0f, 0.0f, 0.0f},
        {24.0f, 0.0f, 0.0f},
        {48.0f, 24.0f, static_cast<float>(M_PI) / 2.0f},
        {48.0f, 48.0f, static_cast<float>(M_PI) / 2.0f}
    };

    chassis.moveRAMSETE(path, 5000,
                        {.b = 2.0f,
                         .zeta = 0.7f,
                         .maxSpeed = 100.0f,
                         .minSpeed = 10.0f,
                         .targetTolerance = 1.5f,
                         .headingTolerance = 4.0f,
                         .lookaheadDist = 12.0f,
                         .useVelocityProfile = true,
                         .maxAccel = 40.0f},
                        false);

    chassis.swingToHeading(270.0f, SwingSide::Left, 1500,
                           {.direction = TurnDirection::SHORTEST,
                            .maxSpeed = 90.0f,
                            .minSpeed = 10.0f,
                            .targetTolerance = 2.0f},
                           false);

    chassis.waitUntilSettled();

}

}  // namespace riku
