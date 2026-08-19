#include "gamespecific/autons.hpp"
#include <vector>
#include "gamespecific/lift.hpp"
#include "riku/api.hpp"
#include "riku/util/characterize.hpp"
#include "riku/util/autotune.hpp"
#include <iostream>

namespace riku {

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

// ============================================================================
// Characterisation
// ============================================================================
//
// Each phase prints a block to paste into config.cpp. Order matters; see main.cpp.

namespace {

tuning::Characterizer makeCharacterizer() {
    tuning::CharacterizeConfig cfg;
    cfg.onProgress = [](const char* stage, const char* detail) {
        printf("  [%s] %s\n", stage, detail);
    };
    return tuning::Characterizer(chassis, drivetrain, cfg);
}

}  // namespace

void autonCharacterize1Drive() {
    printf("\n=== phase 1: track width and feedforward ===\n");
    printf("Spins ~6 turns, then shuttles forward and back over %.0f in.\n",
           tuning::CharacterizeConfig{}.maxTravelIn);
    printf("Needs a clear lane that long, both directions.\n");
    auto ch = makeCharacterizer();

    const float widthWas = drivetrain.trackWidth;
    const float width = ch.trackWidth();
    const auto ff = ch.feedforward(false);
    const auto turn = ch.feedforward(true);
    drivetrain.setVoltage(0.0f, 0.0f);

    printf("\n--- paste into config.cpp ---\n");
    printf("drivetrain.trackWidth = %.3ff;   // was %.3f\n", width, widthWas);
    printf("// RAMSETEParams::ffGains\n");
    printf(".kV = %.4ff, .kaStraight = %.4ff, .ksStraight = %.4ff, .ksTurn = %.4ff\n",
           ff.kV, ff.kA, ff.kS, turn.kS);

    // A bad line fit poisons every phase after this, so it is a stop condition
    // rather than a warning.
    if (!ff.valid || ff.r2 < 0.9f) {
        printf("\nSTOP: r2 = %.3f, want >= 0.9.\n", ff.r2);
        printf("Lower CharacterizeConfig::rampVoltsPerSec and rerun.\n");
    } else {
        printf("\nr2 = %.3f, ok. Write these in, then run phase 2.\n", ff.r2);
    }
}

void autonCharacterize2Pid() {
    printf("\n=== phase 2: PID ===\n");
    printf("Assumes phase 1's trackWidth is already in config.cpp.\n");

    tuning::PidAutoTuner tuner(chassis);
    const PIDGains angular = tuner.tuneAngular();
    const PIDGains lateral = tuner.tuneLateral();

    printf("\n--- paste into config.cpp ---\n");
    printf("chassisConfig.angularGains = {%.3ff, %.4ff, %.3ff};\n",
           angular.kP, angular.kI, angular.kD);
    printf("chassisConfig.lateralGains = {%.3ff, %.4ff, %.3ff};\n",
           lateral.kP, lateral.kI, lateral.kD);
}

void autonCharacterize3Odom() {
    printf("\n=== phase 3: wheel scale and odometry drift ===\n");
    printf("Needs phase 2's PID. Localization must be OFF.\n");
    auto ch = makeCharacterizer();

    // Drives 24 in so you can measure what it actually travelled. Put the tape
    // reading here and rerun to get the scale factor; 0 skips the correction.
    constexpr float kTapeMeasuredIn = 0.0f;

    if (kTapeMeasuredIn > 0.0f) {
        const float was = drivetrain.wheelDiameter;
        const float factor = ch.measureScale(kTapeMeasuredIn, 24.0f);
        printf("drivetrain.wheelDiameter = %.4ff;   // was %.4f\n",
               was * factor, was);
    } else {
        const float before = drivetrain.averagePositionDeg();
        chassis.moveDistance(24.0f, 5000);
        const float after = drivetrain.averagePositionDeg();
        const float reported = (after - before) / 360.0f *
                               drivetrain.externalGearRatio() * PI *
                               drivetrain.wheelDiameter;
        printf("  odometry says it drove %.2f in.\n", reported);
        printf("  Measure the real distance, set kTapeMeasuredIn, rerun.\n");
    }

    const auto drift = ch.measureOdomDrift(36.0f, 3);
    printf("\n--- paste into config.cpp ---\n");
    printf("chassisConfig.mclConfig.imeVariance        = %.4ff;\n", drift.alongPerIn);
    printf("chassisConfig.mclConfig.horizConstantNoise = %.4ff;\n", drift.lateralPerIn);
    printf("chassisConfig.ekfConfig.vertNoiseProp      = %.4ff;\n", drift.alongPerIn);
    printf("chassisConfig.ekfConfig.slipNoiseProp      = %.4ff;\n", drift.lateralPerIn);
    if (drift.alongPerIn > 0.5f) {
        printf("\nalong-track drift is implausibly large. It probably did not\n");
        printf("return to the same spot -- check the PID from phase 2.\n");
    }
}

void autonCharacterize4Filters() {
    printf("\n=== phase 4: sensor noise and localization noise ===\n");
    printf("Park square to a wall, 12-24 in away, before running.\n");
    auto ch = makeCharacterizer();

    const auto beams = ch.measureRangeNoise(distSensors);
    printf("\n--- paste into config.cpp ---\n");
    printf("chassisConfig.mclConfig.rangeSigmaCloseIn = %.4ff;\n", beams.worstSigmaIn);
    printf("chassisConfig.ekfConfig.rangeSigmaBase    = %.4ff;\n", beams.worstSigmaIn);

    // Measures the filter as configured, so it only means anything once the
    // values above and phase 3's are in and MCL + EKF are enabled.
    const auto noise = ch.localizationNoise();
    printf("\n// with MCL + EKF enabled:\n");
    printf("targetTolerance  >= %.2ff\n", noise.suggestTolerance);
    printf("bearingHoldDist   = %.2ff\n", noise.suggestBearingHold);
    if (noise.sigmaIn > 1.0f) {
        printf("\nsigma is over 1 in. Check the sensor mounts in config.cpp --\n");
        printf("mount error shows up here as noise.\n");
    }
}

}  // namespace riku
