# What the VEX community actually ships (researched 2026-08-22)

Reference material for the blind side-by-side comparison. Every claim here has a
URL; do not extend it with speculation.

## The libraries teams actually use

| Library | Platform | Motion control | Localization |
|---|---|---|---|
| **LemLib** (most used PROS template) | PROS, diff-drive only | PID, boomerang `moveToPose`, pure pursuit path following, motion chaining, arcade/tank/curvature driver control | odometry from any mix of motor encoders, rotation sensors, optical shaft encoders, IMU |
| **EZ-Template** | PROS | plug-and-play PID drive functions | odometry |
| **JAR-Template** | VEXcode Pro | PID, "custom motion algorithms" | odometry |
| **OkapiLib** | PROS | the "comprehensive" end of the scale LemLib positions against | odometry |

LemLib explicitly describes its goal as balancing EZ-Template's ease of use with
OkapiLib's functionality. It supports **differential drivetrains only**.

Sources:
- https://github.com/LemLib/LemLib/blob/master/README.md
- https://lemlib.readthedocs.io/en/stable/tutorials/8_motion_chaining.html
- https://github.com/JacksonAreaRobotics/JAR-Template
- https://www.vexforum.com/t/ez-template-vs-arms-vs-lemlib/114691

## Ceiling of VEX practice, per subsystem

**Path following.** Pure pursuit is the top of what shipping VEX libraries do.
No shipping VEX library found doing RAMSETE, LTV unicycle, or LQR-based
following, and none doing constrained time-optimal trajectory generation.

**Motion chaining** (LemLib, the closest thing to a standard):
- `minSpeed` — floor on output that "overrides everything (slew, PID,
  maxSlipSpeed), but it does not override maxSpeed".
- `earlyExitRange` — error threshold at which the motion ends early; **units
  differ by motion type**, degrees for angular and inches for positional.
- Exit is either automatic on `error < earlyExitRange`, or manual via
  `cancelMotion()` / `cancelAllMotions()`.
- Documented warning: without one of those exits "the motion will just run until
  the timeout runs out", because a high `minSpeed` stops it ever settling. High
  `minSpeed` in a 2D motion also costs heading accuracy.

So: `earlyExitRange` + `minSpeed` is the community-standard chaining API, and
chad matches that surface. The documented failure mode — a chained motion that
cannot settle and therefore burns its whole timeout at speed — is exactly the
gap that stall detection closes. No VEX library found does stall detection.

**MPC.** Searching for MPC on a V5 drivetrain returns *no* VEX implementation,
forum thread, or writeup — only automotive/railway/UAV academic papers. Treat
this as the finding it is: there is no VEX peer for chad's MPC, so it can only be
judged against the industry standard (OSQP, acados, WPILib's LTV controllers),
never against VEX practice. Do not claim a VEX comparison that does not exist.

Traction wheels, from the forum, are chosen precisely because they slip less and
make autonomous more repeatable — at the cost of turning "violent jittering" when
a team runs all-traction. That is the physical justification for the higher grip
limit and for wheel-slip-aware constraints being worth having.

Sources:
- https://www.vexforum.com/t/should-i-use-traction-wheels-for-high-stakes/127046
- https://www.vexforum.com/t/traction-wheel-problems/105734

## The best documented VEX MCL (aadishv.dev), for the localization comparison

Concrete numbers from the writeup — this is the bar, and the author is candid
about its limits:

- **100 particles**, all initialised at the start point, spreading naturally.
- **Noise: uniform** in [-1, 1) scaled by `XY_NOISE = 0.1 in`, `THETA_NOISE = 2 deg`.
  Uniform, not Gaussian. Advice is to tune it to expected odometry drift.
- **Sensor model: one distance sensor assumed at robot centre.** No mount offset,
  no yaw offset. Expected wall distance from a cos/sine projection rather than a
  ray cast — the author states this is "not as accurate as raycasting".
- **Weighting: Gaussian** `exp(-0.5 (x/s)^2) / (s sqrt(2pi))`, `GAUSSIAN_STDEV = 1`,
  and the beam likelihoods are **summed, not multiplied** — the author says
  outright that theory calls for the product but summing was "easier to tune".
- **No outlier rejection**, no max-range branch, no log-domain accumulation.
- **Resampling: Stochastic Universal Resampling** — evenly spaced offsets on the
  cumulative-weight wheel with one random start. This is a genuine low-variance
  scheme, so it is *not* a differentiator; matching it is table stakes.
- **No effective-sample-size tracking and no adaptive resampling.**
- Written in Python; the author warns the VEX MicroPython interpreter is "far too
  slow to run efficient MCL" and says the code is **"not suitable for
  competition use"**.
- Odometry is folded straight into the particles; no separate fusion stage.

Sources:
- https://www.aadishv.dev/mcl
- https://github.com/u-k-g/monte-carlo-localization

### What this means for judging chad's MCL

Table stakes (community already here): low-variance resampling, Gaussian
likelihood, odometry-driven motion update.

Genuinely above the community bar, and therefore what to check chad actually
does correctly: true ray casting against the field rectangle; multiple sensors
with mount offset **and** yaw; log-domain product weighting; a max-range branch
and an outlier mixture term; ESS-triggered adaptive resampling; body-frame
Gaussian motion noise; a circular mean for heading; and running fast enough in
C++ to be competition-viable at all.
