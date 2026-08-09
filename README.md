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

## Building

The PROS toolchain ships with the VS Code extension and is not on the default
PATH:

```sh
export PATH="$APPDATA/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr/bin:$PATH"
make clean && make
```

## Tuning logs

Every tunable subsystem can dump CSV to the microSD card for offline fitting.
All are off by default and no-op without a card inserted.

```cpp
chassis.setMotionLogging(true);   // /usd/chas<n>.csv  lateral + angular PID
liftSetLogging(true);             // /usd/lift<n>.csv  lift kP / kG
jam.setLogging(true);             // /usd/jam<n>.csv   efficiency band
ltv.setLogging(true);             // /usd/ff<n>.csv    kV / kA / kS fitting
```

Each run picks the first free index, so files are never overwritten. `t_ms` is
prepended to every row. `Logger::droppedRows()` is non-zero if the flush task
could not keep up.

## License

GPL-3.0-or-later — see [LICENSE](LICENSE).

riku incorporates code from several projects; see
[ATTRIBUTION.md](ATTRIBUTION.md) for per-source licensing. Note that FBLib
declares no license upstream, so its rights status must be confirmed before
publishing this repository.
