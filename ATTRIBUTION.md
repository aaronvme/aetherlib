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

### WPILib (allwpilib) — https://github.com/wpilibsuite/allwpilib
- License: **BSD-3-Clause**
- Used for: the design of `movement_control/spline.*`,
  `movement_control/trajectory.*`, `movement_control/trajectory_constraint.hpp`
  and `movement_control/trajectory_generator.*`. The Hermite spline basis,
  the clamped-cubic tridiagonal tangent solve, recursive spline
  parameterization, the two-pass (forward/backward) time parameterization and
  the constraint interface all follow `frc::Spline`, `frc::SplineHelper`,
  `frc::SplineParameterizer`, `frc::Trajectory`,
  `frc::TrajectoryParameterizer` and `frc::TrajectoryConstraint`. WPILib's
  spline parameterizer and time parameterizer are themselves derived from
  Team 254's 2018 code (MIT) and from Sprunk, *Planning Motion Trajectories
  for Mobile Robots Using Splines* (2008).
- Also used for: the design of `util/geometry.hpp` (`Rotation2d` holding its own
  cosine and sine, `Translation2d`, `Twist2d`, `Transform2d`, and the `Pose`
  exponential/logarithm), `util/kinematics.hpp` (`ChassisSpeeds` and its
  second-order `discretize`, `DifferentialDriveKinematics`, wheel-speed
  desaturation), `util/filters.hpp`, and the rewind-correct-replay scheme in
  `tracking/pose_estimator.hpp`, which follows
  `frc::DifferentialDrivePoseEstimator` and `frc::TimeInterpolatableBuffer`.
  The pose-exponential derivation is Veness, *Controls Engineering in the FIRST
  Robotics Competition*.
- No WPILib code is copied: the algorithms were re-derived and rewritten in
  chad's style, in inches/seconds/radians, with `float` arithmetic, explicit
  failure returns instead of exceptions and fixed-size scratch buffers. Where
  float32 forced a different formulation it is noted in the code — the pose
  exponential uses half-angle identities rather than WPILib's `(1 - cos)/theta`,
  which loses most of its significance at small angles in single precision.

### Published algorithms implemented from the literature
- `movement_control/riccati.hpp` solves the discrete algebraic Riccati equation
  by structure-preserving doubling, after Chu, Fan, Lin and Wang,
  *Structure-preserving algorithms for periodic discrete-time algebraic Riccati
  equations*. No third-party code involved.
- `movement_control/mpc.*` follows the standard condensed box-constrained MPC
  formulation with a fast-gradient solve: Nesterov acceleration with the
  gradient-based adaptive restart of O'Donoghue and Candes, *Adaptive restart
  for accelerated gradient schemes*, and the real-time iteration scheme of
  Diehl et al. for the fixed-budget mode.
- `movement_control/qp.hpp` is an operator-splitting (ADMM) QP solver for
  general linear inequality constraints, following Stellato, Banjac, Goulart,
  Bemporad and Boyd, *OSQP: an operator splitting solver for quadratic
  programs*: the same split, over-relaxation and primal/dual residual pair.
  Adapted for a fixed-size embedded target — one factorization per horizon
  rebuild, rho held constant for its lifetime so the iteration never
  refactorizes, and no allocation in the solve path. No OSQP code involved.
- `tracking/mcl_core.*` implements the particle filter of Thrun, Burgard and
  Fox, *Probabilistic Robotics*: the low-variance sampler (Table 4.4), the beam
  range-finder mixture model (Table 6.1) and the sampling motion model
  (Table 5.6), plus the augmented-MCL random injection of Table 8.2 (kept off by
  default). Weights are accumulated in the log domain, which the book leaves as
  an implementation concern. No third-party code involved.

  One deliberate divergence from Table 6.1: `p_short` decays from the expected
  range toward the sensor rather than outward from the sensor, so it is at its
  maximum at the expected range where the book's is small. That keeps the
  mixture a function of the residual in sigmas, so it can be tabulated once
  instead of costing a logarithm per particle per beam. The cost is a standoff
  bias, bounded by the width of the converged cloud — 0.229 sigma against a flat
  prior, but 0.002 to 0.038 inches on real field geometry, against a
  noise-driven mean error of 0.13 inches.
- `tracking/mcl_core.hpp` generates random numbers with PCG-XSH-RR 32/64, after
  M. E. O'Neill, *PCG: a family of better random number generators* — chosen
  over mt19937 for its 16 bytes of state. Constants are the published ones; the
  implementation is written here.
- `tracking/ekf_core.*` uses the Joseph-form covariance update and a
  Mahalanobis (normalized innovation squared) gate on measurements, both
  standard results in the Kalman filtering literature — see Bar-Shalom, Li and
  Kirubarajan, *Estimation with Applications to Tracking and Navigation*.

## Toolchain dependencies (unmodified, not vendored into chad)

- **PROS kernel** 4.2.2 — MPL-2.0, Copyright (c) Purdue University ACM SIGBots.
- **LVGL** (`liblvgl` 9.2.0) — MIT.
- **Eigen** — MPL-2.0. Vendored unmodified under `include/Eigen/` (374 files),
  copied from the 1516X source tree for the LTV follower. Per-file MPL-2.0
  notices are intact; note that 1516X's copy does not include Eigen's
  `COPYING.*` files, so those are absent here too.
