#include "gamespecific/autons.hpp"
#include "gamespecific/autons/tuning/support.hpp"

#include "chad/api.hpp"
#include "chad/util/characterize.hpp"

namespace chad {

using namespace autons;

void autonCharacterize1Drive() {
    Report out;
    out("phase 1: track width + feedforward");
    if (!headingOk(out)) return;
    out("spins, then steps ~60 in");
    setFilters(false);
    auto ch = makeCharacterizer(out);

    const float widthWas = drivetrain.trackWidth;
    const float width = ch.trackWidth();

    const auto str = ch.feedforward(false);
    const auto turn = ch.feedforward(true);
    drivetrain.setVoltage(0.0f, 0.0f);

    if (!str.valid) {
        out.result("STOP r2=%.3f want>=0.9", str.r2);
        out.result("lev %d span %.0f in/s", str.samples,
                   str.speedSpanInPerSec);
        out.result("need room for 4+ levels");
        return;
    }

    drivetrain.trackWidth = width;
    ramseteFf.kV = str.kV;
    ramseteFf.ksStraight = str.kS;
    ramseteFf.ksTurn = turn.kS;

    // An unfit kA is worse than none: it feeds the velocity loop a voltage
    // proportional to commanded acceleration with no basis, and the old ramp
    // fit could return it negative. Zero just leaves kV and kS doing the work.
    ramseteFf.kaStraight = str.kaValid ? str.kA : 0.0f;
    ramseteFf.kaTurn =
        turn.kaValid ? turn.kA * width * INCH_TO_METER * 0.5f : 0.0f;

    constexpr float kLoopSpeedup = 3.0f;
    const float tau = str.kV > 0.0f ? str.kA / str.kV : 0.0f;
    ramseteFf.kP = (kLoopSpeedup - 1.0f) * str.kV;

    out.result("copy to config.cpp:");
    out.result("trackWidth = %.3ff (was %.3f)", width, widthWas);
    out.result("kV=%.4ff kaStraight=%.4ff", str.kV, ramseteFf.kaStraight);
    out.result("ksStraight=%.4ff ksTurn=%.4ff", str.kS, turn.kS);
    out.result("kaTurn=%.4ff kP=%.2ff", ramseteFf.kaTurn, ramseteFf.kP);
    out.result("r2=%.3f lev=%d free=%.0f%%", str.r2, str.samples,
               100.0f * str.freeSpeedRatio);
    if (str.kaValid) {
        out.result("tau=%.3fs", tau);
    } else {
        out.result("kA UNFIT->0 spr%.2f n%d", str.kaSpread, str.stepReps);
    }

    // The one fault a straight-line fit cannot see: a drivetrain that is
    // simply weaker than configured still fits a clean line.
    if (str.drivetrainSuspect) {
        out.result("CHECK MOTORS/GEARING");
        out.result("else rpm=%.0ff (is %.0f)",
                   drivetrain.rpm * str.freeSpeedRatio, drivetrain.rpm);
    }
}

}  // namespace chad
