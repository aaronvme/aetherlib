# riku

A C++ robotics library for VEX V5RC built on the PROS kernel, combining the
strongest parts of [FBLib](https://github.com/jiazegao/FBLib), the 1516X
codebase and [LemLib](https://github.com/LemLib/LemLib) into one library.

**Target:** VEX V5 Brain (ARM Cortex-A9, PROS kernel 4.2.2)
**Language:** gnu++26
**Namespace:** `riku`

## Layout

```
include/riku/
  util/               math, Pose, PID, fast trig, IMU scaling, jam detection, warnings
  tracking/           odometry, MCL, RCL localization
  movement_control/   point-to-point, turns, arcs, boomerang, RAMSETE, profiles,
                      feedforward, LTV path following
  auton_selector/     LVGL 9 touch selector
  drivetrain.hpp      drive base + drive curves + voltage output type
  chassis.hpp         top-level drivetrain + motion API
include/gamespecific/ robot config, controls, autons, lift
include/Eigen/        vendored Eigen (used by the LTV follower)
```

## Usage

```cpp
#include "riku/api.hpp"
using namespace riku;
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
make clean && make
```

## Tuning logs

Subsystems emit telemetry through a callback; `riku/util/tuning.hpp` and
`gamespecific/tuning.hpp` attach hooks that write CSV to the microSD card.
The subsystems themselves know nothing about logging. All off by default, and
no-ops without a card.

```cpp
riku::tuning::logChassis(chassis, true);                          // chas<n>.csv
riku::tuning::logJamDetector(jam, true);                          // jam<n>.csv
riku::tuning::logFeedforward(ltv.velocityController(), true);     // ff<n>.csv
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
chassis.setTelemetry([](const riku::ChassisSample& s) { ... });
```

## License

GPL-3.0-or-later — see [LICENSE](LICENSE).

riku incorporates code from several projects; see
[ATTRIBUTION.md](ATTRIBUTION.md) for per-source licensing.