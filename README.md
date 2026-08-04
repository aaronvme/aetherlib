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
  Util/               math, Pose, PID, fast trig, IMU scaling, jam detection, warnings
  Tracking/           odometry, MCL, RCL localization
  Movement_Control/   point-to-point, turns, arcs, boomerang, RAMSETE, profiles,
                      feedforward, LTV path following
  Auton_Selector/     LVGL 9 touch selector
  Drivetrain.hpp      drive base + drive curves + voltage output type
  Chassis.hpp         top-level drivetrain + motion API
include/GameSpecific/ robot config, controls, autons, lift
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

## License

GPL-3.0-or-later — see [LICENSE](LICENSE).

riku incorporates code from several projects; see
[ATTRIBUTION.md](ATTRIBUTION.md) for per-source licensing. Note that FBLib
declares no license upstream, so its rights status must be confirmed before
publishing this repository.
