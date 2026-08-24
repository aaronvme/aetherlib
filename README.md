# chad

A C++ robotics library for VEX V5RC built on the PROS kernel, combining the
strongest parts of [FBLib](https://github.com/jiazegao/FBLib), the 1516X
codebase and [LemLib](https://github.com/LemLib/LemLib) into one library.

**Target:** VEX V5 Brain (ARM Cortex-A9, PROS kernel 4.2.2)
**Language:** gnu++26
**Namespace:** `chad`

## Layout

```
include/chad/
  util/               geometry (SE(2), pose exponential), kinematics, PID, filters,
                      settle conditions, fast trig, IMU scaling, jam detection
  tracking/           odometry, MCL, RCL, EKF, latency-compensated pose fusion
  movement_control/   point-to-point, turns, arcs, boomerang, RAMSETE, splines,
                      trajectory generation, profiles, feedforward, LQR, MPC
  auton_selector/     LVGL 9 touch selector
  drivetrain.hpp      drive base + drive curves + voltage output type
  chassis.hpp         top-level drivetrain + motion API
include/gamespecific/ robot config, controls, autons, lift
include/Eigen/        vendored Eigen (used by the LTV follower)
test/host/            unit tests for the PROS-free maths, run on the desktop
docs/research/        what other VEX libraries and industry actually ship, with
                      sources, for judging this one against them
```

## Usage

```cpp
#include "chad/api.hpp"
using namespace chad;
```

Every public class and function is listed in [API.md](API.md).

Two conventions to know before writing anything: `Chassis` methods take **VEX
degrees** (0 = forward, clockwise positive) while `Pose::theta` is **standard
math radians** (0 = +X, counter-clockwise) — convert with `vexToStdRad()` /
`stdRadToVexDeg()`. And everything is in **inches** except the feedforward
controller and LTV follower, which work in metres.

## Building

The PROS toolchain ships with the VS Code extension and is not on the default
PATH:

```sh
export PATH="$APPDATA/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr/bin:$PATH"
make quick
```

Two builds matter. `make quick` produces the brain image; `test/host/run.ps1`
builds and runs the unit tests for every translation unit that does not include
`pros/`, on the desktop with MSVC. Keeping the maths PROS-free is what makes the
second one possible, so a controller, filter, spline or solver should never
reach for hardware directly.

```powershell
.\test\host\run.ps1              # whole suite
.\test\host\run.ps1 trajectory   # only matching cases
```

`-Tests` narrows what gets *compiled* rather than what gets run, and `-OutDir`
moves the object files, so two runs can proceed at once without fighting over
one build directory:

```powershell
.\test\host\run.ps1 -Tests mcl -OutDir $env:TEMP\mcl
```

PROS-dependent sources cannot be host-compiled, so they are checked against
the real ARM toolchain instead. `test/syntax-check.sh` writes nothing to
`bin/` or `.d/`, which makes it safe to run during a build and safe to run
several at once — unlike `make`:

```sh
./test/syntax-check.sh src/chad/chassis.cpp   # no arguments checks every source
```

## Path following

`followTrajectory` is the strongest option: generate a trajectory once with
`generateTrajectory` and the constraints from `Drivetrain::model()`, and the
reference carries its own speed, acceleration and curvature, already inside the
grip and wheel-speed limits the robot actually has. Declaring the wheel type
(`WheelType::Omni` / `Traction`) in the robot config is what sets those limits.

`moveRAMSETE` profiles a bare geometric path as it drives it, which is still the
right tool when the path comes from somewhere that cannot be parameterized
up front. Set `RAMSETEParams::useVelocityController` to close the loop on
measured wheel speed instead of treating voltage as speed; it measured faster
with better heading at every setting tried.

### Motion chaining

`MoveToPointParams::earlyExitRange` and `MoveDistanceParams::earlyExitRange`
end a motion early and hand the drivetrain over still moving, so the next queued
motion inherits the speed instead of accelerating from rest. `minSpeed` floors
the output while a motion is still trying, which is what stops a tuned loop
creeping out its timeout over the last inch. With the queue empty the drive
stops, so a chained motion cannot leave voltage on.

The feedforward gains behind both followers were fitted against the simulator,
so refit `kV`/`ksStraight`/`kP` on hardware before relying on them
(`chad/util/tuning.hpp` logs the CSV for it).

### Deprecated

`LtvLqrFollower` (`movement_control/ltvlqr.hpp`) and `LTVPathFollower`
(`movement_control/ltv.hpp`) are kept for completeness and are not recommended
for competition use. Both solve an LQR over the same pose-error plant that
`moveRAMSETE` now covers with a solved gain, and both duplicate the velocity
profile and settle logic that live in `Chassis`. `LtvLqrFollower` ends 2-3
degrees closer to the final tangent than RAMSETE; a `turnToHeading` afterwards is
the cheaper way to get that. The reusable pieces are `movement_control/lqr.hpp`
(Riccati solve plus a speed-scheduled gain table) and
`movement_control/drivetrain_plant.hpp`.

`util/slew.hpp` is disabled by default and measured worse than leaving it off on
every path tried. Shape acceleration with `RAMSETEParams::maxAccel` and
`maxLateralAccel` instead -- in the profile, in real units, ahead of the
controller.

## Localization

There are no odometry pods on this robot: the pose comes from the motor encoders
and the IMU, corrected against the field walls by the V5 distance sensors. MCL
runs a particle filter over the walls, and the EKF fuses it with odometry and
writes the result back into the odometry pose a fraction at a time, so nothing
downstream ever differentiates a step.

Two things to do before racing on it:

- **Measure `ekf.rangeLatencyMs`.** It ships at 30 ms, which is an estimate, not
  a measurement. A distance reading describes where the robot *was*, so the
  error it leaves is `speed * latency` — a bias, not a noise, which means more
  beams will not average it away. At 60 in/s, 50 ms of uncorrected latency is
  three inches. The procedure is in the notes at the top of `config.cpp`.
- **Watch the reject counters on a clean field.** The range gate is +/-3.17 in
  on the shipped config. If `rangeRejectStreak()` climbs with no robot in the
  way, the gate is too tight for this drivetrain's real odometry error — raise
  `vertBiasProp` / `slipBiasProp` in `config.cpp`, not in the header.
  `rangeBlindStreak()` climbing instead means beams are being deadzoned as
  obstacles rather than gated.

## Tuning logs

Subsystems emit telemetry through a callback; `chad/util/tuning.hpp` and
`gamespecific/tuning.hpp` attach hooks that write CSV to the microSD card.
The subsystems themselves know nothing about logging. All off by default, and
no-ops without a card.

```cpp
chad::tuning::logChassis(chassis, true);                          // chas<n>.csv
chad::tuning::logJamDetector(jam, true);                          // jam<n>.csv
chad::tuning::logFeedforward(ltv.velocityController(), true);     // ff<n>.csv
logLift(true);                                                    // lift<n>.csv
```

Each run picks the first free index, so files are never overwritten. `t_ms` is
prepended to every row, and the flush task fsyncs every 50 ms, so a power-off
loses at most that much. `Logger::droppedRows()` is non-zero if logging
outpaced the writer.

To graph one:

```sh
python tools/plot_log.py lift00.csv                  # every column vs time
python tools/plot_log.py lift00.csv --cols error pid
python tools/plot_log.py chas00.csv --xy             # path in field coords
```

You can also attach a raw callback with no SD card involved, e.g. to print to
the terminal or drive a screen widget:

```cpp
chassis.setTelemetry([](const chad::ChassisSample& s) { ... });
```

## License

GPL-3.0-or-later — see [LICENSE](LICENSE).

chad incorporates code from several projects; see
[ATTRIBUTION.md](ATTRIBUTION.md) for per-source licensing.