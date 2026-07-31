#include "GameSpecific/autons.hpp"
#include <vector>
#include "GameSpecific/lift.hpp"
#include "AaronLib/AaronLib_API.hpp"
#include <iostream>

namespace AARONLIB {

void autonExample() {
    // Start in the lower-left field corner, facing +X.
    // (Background odometry is started automatically by the Chassis constructor.)
    chassis.setPose({0.0f, 0.0f, 0.0f});

    // 1) Straight drive a fixed distance (forward)
    chassis.moveDistance(24.0f, 2000,
                         {.forwards = true,
                          .maxSpeed = 100.0f,
                          .minSpeed = 20.0f,
                          .targetTolerance = 1.0f,
                          .earlyExitRange = 0.0f},
                         false);

    // 2) Drive to an absolute field point
    chassis.moveToPoint(36.0f, 24.0f, 3000,
                        {.forwards = true,
                         .maxSpeed = 110.0f,
                         .minSpeed = 15.0f,
                         .targetTolerance = 1.5f,
                         .earlyExitRange = 0.0f,
                         .lead = 4.0f},
                        false);

    // 3) Rotate to an absolute heading (VEX degrees, clockwise positive)
    chassis.turnToHeading(90.0f, 1500,
                          {.direction = TurnDirection::SHORTEST,
                           .maxSpeed = 90.0f,
                           .minSpeed = 10.0f,
                           .targetTolerance = 2.0f},
                          false);

    // 4) Turn to face a target point without driving to it
    chassis.turnToPoint(0.0f, 0.0f, 1500,
                        {.direction = TurnDirection::SHORTEST,
                         .maxSpeed = 90.0f,
                         .minSpeed = 10.0f,
                         .targetTolerance = 3.0f,
                         .offset = 0.0f},
                        false);

    // 5) Drive along an arc to a target coordinate
    chassis.moveArc(24.0f, 24.0f, 20.0f, 3000,
                    {.forwards = true,
                     .maxSpeed = 90.0f,
                     .minSpeed = 10.0f,
                     .targetTolerance = 1.5f,
                     .lead = 3.0f},
                    false);

    // 6) Boomerang approach: drive to a point and end with a heading
    chassis.moveBoomerang(48.0f, 24.0f, 180.0f, 3500,
                          {.forwards = true,
                           .maxSpeed = 90.0f,
                           .minSpeed = 10.0f,
                           .targetTolerance = 2.0f,
                           .headingTolerance = 3.0f,
                           .lead = 0.6f,
                           .leadDecay = 0.5f},
                          false);

    // 7) Follow a path with RAMSETE
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

    // 8) Swing turn: rotate using only one side of the drivetrain
    chassis.swingToHeading(270.0f, SwingSide::Left, 1500,
                           {.direction = TurnDirection::SHORTEST,
                            .maxSpeed = 90.0f,
                            .minSpeed = 10.0f,
                            .targetTolerance = 2.0f},
                           false);

    chassis.waitUntilSettled();


}

void autonSkills() {
    chassis.setPose({0.0f, 0.0f, 0.0f});
    chassis.moveDistance(30.0f, 2000);
    chassis.turnToHeading(90.0f, 1200);
    chassis.moveToPoint(24.0f, 24.0f, 2500);
}

void autonRedLeft() {
    chassis.setPose({0.0f, 0.0f, 0.0f});
    chassis.moveDistance(18.0f, 1500);
    chassis.turnToHeading(180.0f, 1200);
    chassis.moveDistance(12.0f, 1000);
}

void autonBlueLeft() {
    chassis.setPose({0.0f, 0.0f, 0.0f});
    chassis.moveDistance(12.0f, 1500);
    chassis.turnToHeading(270.0f, 1200);
    chassis.moveDistance(18.0f, 1000);
}


}  // namespace AARONLIB