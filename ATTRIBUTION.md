# Attribution

chad is assembled from three existing VEX V5 codebases. This file records
where each part came from and under what terms.

chad as a whole is distributed under **GPL-3.0-or-later** (see `LICENSE`),
because it incorporates GPL-3.0-or-later code from 1516X. Individual files
keep the SPDX headers and copyright notices of their origin.

## Sources

### 1516X — California High Robotics
- Upstream: the 1516X competition codebase
- License: **GPL-3.0-or-later**, `Copyright 2026 California High Robotics, Team 1516X`
- Used for: the LTV/LQR path follower and related path-following support code.
  Their `VoltageController` was also ported initially but has since been
  removed: FBLib's `FeedforwardController` is a corrected reimplementation of
  the same controller, so keeping both meant shipping two of one thing.
- Original per-file `Copyright` and `SPDX-License-Identifier` headers are retained
  unmodified on every file derived from this source.

### FBLib — https://github.com/jiazegao/FBLib
- Used for: `Tracking/` (odometry, MCL, RCL), `Movement_Control/`, `Util/`,
  `Chassis`, `Auton_Selector/`, and the `GameSpecific/` starting point.
- License: **none declared upstream** — FBLib ships no `LICENSE` file and no
  per-file copyright headers. Permission to use it here was granted directly
  by FBLib's author. That permission is the basis for this project's use; it
  does not place a license on FBLib itself, so anyone redistributing this
  repository should seek their own.
- Files ported from FBLib are noted as such in their header comment. The FBLib
  namespace `FBLIB` is translated to `chad` and paths `FBLib/` to `chad/`;
  the code is otherwise line-for-line comparable to upstream.

### LemLib — https://github.com/LemLib/LemLib
- License: **MIT**
- Used for: `ExitCondition` (debounced settle check) ported to source form.
- `ExpoDriveCurve` was deliberately **not** ported — FBLib's `DriveCurve` already
  covers it and is wired into `ChassisConfig`.
- Only available locally as a prebuilt `LemLib.a` plus headers, so this piece is
  ported rather than linked. Ported files carry a LemLib MIT notice.

## Toolchain dependencies (unmodified, not vendored into chad)

- **PROS kernel** 4.2.2 — MPL-2.0, Copyright (c) Purdue University ACM SIGBots.
- **LVGL** (`liblvgl` 9.2.0) — MIT.
- **Eigen** — MPL-2.0. Vendored unmodified under `include/Eigen/` (374 files),
  copied from the 1516X source tree for the LTV follower. Per-file MPL-2.0
  notices are intact; note that 1516X's copy does not include Eigen's
  `COPYING.*` files, so those are absent here too.
