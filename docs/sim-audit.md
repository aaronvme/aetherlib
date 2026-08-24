# What the first working sim rebuild found

The simulator had not been rebuilt in days and had rotted silently. Everything
here was measured on 2026-08-23 after making it build again, on a machine with
no robot. Numbers are reproducible; each entry says how.

## Blockers fixed before anything could be measured

**`build.sh` listed a source that had become a directory.** It named
`src/gamespecific/autons.cpp`, which is now `src/gamespecific/autons/`. Sources
are discovered with `find` now, for both `src/gamespecific` and `sim/bridge`, so
adding a routine or a bridge translation unit cannot silently stop being built.

**The drive model was inverted, so the robot spun in place instead of driving.**
`World::updateDrive` applied each motor's `reversed` flag to the *command*.
`config.cpp`'s left ports are all negative (`{-18, -19, -12}`), so equal
voltages drove the left side backwards: at 6 V a side, ground truth held station
while the wheels read L −31.5 / R +31.5 in/s. Introduced by commit `a3599ea`,
which configured the real robot's ports, and invisible until now because nothing
had rebuilt the sim since.

The port sign *is* the mirrored mounting, so the reversal and the mounting
cancel and a side's forward command is the logical one. Fixed by making the bus
hold each motor's physical command (`move_voltage` applies the reversal, matching
`positionDeg`, which always did) and having the world undo it per motor and
average. Averaging rather than reading one port means a side wired with mixed
signs now shows up as motors fighting instead of as a clean drive.

**Consequence: every drive-dependent number in `sim/README.md` predates
`a3599ea` and is stale.** Re-measured after the fix, the qualitative claims all
hold and the figures do not — the robot's measured wheel is 2.6738 in, not the
3.25 in those runs assumed:

| | README | measured now |
|---|---|---|
| RAMSETE, closest approach to path end | 0.29 in | 0.9 in |
| LTV end error at kV 6.2 | 0.42 in | 1.32 in |
| LTV end error at kV 0.9 | 9.1 in short | 11.85 in short |

## MCL published a health metric that could not move

`MclTracking` ran `mCore.step()`, discarded the estimate it computed, and called
`mCore.estimate()` again — on the cloud `step()` had just resampled, whose
weights `resample()` flattens to 1.0. So published `nEff` was **a constant equal
to the particle count**: one distinct value, 1024.0, over 800 samples spanning
healthy tracking, a 31 in kidnap and fully blinded sensors. The published
covariance was the post-resample roughening spread, not the evidence-weighted
spread the header promises.

Fixed by publishing the estimate `step()` already computed
(`MclCore::stepEstimate()`). Healthy tracking now reads 16-69 of 1024, collapsing
to 10.5 while re-acquiring after a kidnap. Pinned by two host tests.

**ESS is high when the filter is lost, not low.** It measures weight *disparity*.
A cloud straddling the truth gives 115.7 of 256; a cloud displaced 38 in gives
256.0, because every particle is equally wrong and log-sum-exp with max
subtraction flattens the weights. So `EkfCore::updateMcl`'s relocation gate,
`mclNeffFrac >= mclMinNeffFrac` (0.20), read a constant 1.0 before the fix — it
was vacuous — and with honest values it is **backwards**, since healthy is
0.016-0.067 and lost is ~1.0.

## MCL recovers from a kidnap less than half the time

Ten seeds, a 24 in kidnap, robot driving (a stationary robot cannot relocate, and
`sim/test/autotune.mjs`'s kidnap trial does not drive — which is why it reports
0/4 and should not be read as the recovery rate):

| | recovered to 3 in | when it did |
|---|---|---|
| MCL raw estimate | 3-4 of 10 | 315-370 ms |
| fused pose, MCL+EKF | 3-4 of 10 | ~970 ms |

The fused pose recovers exactly when MCL does and never otherwise, so recovery
runs through odometry writeback, not through the EKF's reject-streak relocation.
`randomInjectFraction()` is 0.000 throughout: augmented-MCL injection is off, a
deliberate round-2 decision taken on the reasoning that recovery no longer needed
it once the search width was decoupled. **A sub-50% recovery rate does not
support that reasoning, and it is the round-2 judgement most in need of
revisiting.** `distSearchVariance`, the parameter that decoupling turns on, has
no `sim_set_param` knob, so the mechanism cannot currently be swept.

## The tightened EKF range gate does not starve itself

Previously flagged as answerable only on a robot. Sweeping the world's
`encoderSlip` over a 78.7 in straight leg, EKF-only:

| encoder slip | dead reckoning | EKF | beams used | ticks with 0 beams |
|---|---|---|---|---|
| 0.00 | 1.64 in (2.08%) | 0.25 in | 2.23 | 0% |
| 0.08 | 6.49 in (8.25%) | 0.26 in | 2.23 | 0% |
| 0.25 | 19.73 in (25.1%) | 1.50 in | 2.23 | 0% |

The gate holds because it is applied against the *EKF's* predicted range, and the
EKF's own error stays at 1.50 in — well inside the ~±3.17 in gate — however badly
raw odometry drifts. This shows the gate is not structurally too tight; it does
not settle whether it survives real field-geometry error, which the sim's exact
walls do not model.

## Dead reckoning was 2.08% off, with every noise knob at zero — a stale IMU calibration

1.305 in over 63 in and 1.64 in over 78.7 in: the same constant to three figures.
I read that constant as a multiplicative scale error between the world's travel
model and what odometry reconstructs. **That was wrong, and the decomposition is
what settled it:** the along-track error is −0.02% and 100% of the error is
*cross-track*. 2.08% is `2*sin(dtheta/2)` for a fixed heading bias of 1.19 deg,
which also grows linearly with distance and so looks identical to a scale error
end-to-end.

`RobotParams::imuUnderReport` was hard-coded to 354.25/360 while `config.cpp`
declares `ScaledIMU(13, 360.0, 359.0)` — a net heading scale of 0.9867688, so at
the reset heading of 90 the library read 88.81 **while standing still, with
`err.dist` at 0.000**. `OdomCore::step` takes theta from the absolute IMU heading
and integrates travel along the previous heading, so the bias steers every inch
driven. Fixed by deriving the under-report as `1.0 / chad::imu.scaleFactor()`, so
the two cannot drift apart again; the knob still dials in a deliberate mismatch.
Now 0.000 in and 0.00 deg.

`quantise()` uses `std::round` on the absolute total, so it cannot accumulate a
bias in either direction — eliminated as a candidate.

## The grip limit is unreachable on this drivetrain

Found independently by two agents, which is why it is worth stating flatly.
`maxWheelSpeed = 63.00 in/s` and `b/2 = 5.959 in`, so grip can bind only when
`vmax^2 - 4*g*(b/2) >= 0`, i.e. below `g = 166.5 in/s^2`. `WheelType::Traction`
defaults to 200, so it **never** binds: zero grip-bound states on every path
tested, and no real root when solving for the crossover. The per-side wheel-speed
budget is tighter at every curvature — the motors cannot spin fast enough to
slide this base out of any turn it can execute. Consequence: choosing Traction vs
Mixed vs Omni currently changes nothing about a generated trajectory on this
robot. Dropping grip to the Omni default of 60 does produce 19 grip-bound states
and a 12% longer path, so the centripetal code is live; it is the wheel-type
choice that makes it inert.

`RAMSETEParams::maxLateralAccel = 60` is **not** an inconsistency with that 200 —
ramsete.hpp says explicitly that it bounds tracking authority rather than grip.
The real asymmetry is that a generated trajectory has no corner-speed cap beyond
the wheel budget, so moving a path from `moveRAMSETE` to `followTrajectory` buys
faster cornering and less tracking authority with nothing in the API saying so.

## Smaller things

- `sim_get_param` returns NaN for every `ekf*`, `mcl*`, `rcl*` and `ramsete*`
  key that `sim_set_param` accepts — write-only, so a tuning UI cannot show the
  value it is about to change.
- `sim/README.md`'s "Determinism caveat" is stale: `MclTracking::setSeed()`
  exists and `MclCore::kDefaultSeed` makes runs reproducible by default.
- All four registered autons now run to completion (2.7-8.5 s). Every one calls
  `chassis.setPose({0,0,0})`, so their odometry frame is decoupled from ground
  truth — comparing `odom` against `true` during an auton without accounting for
  that reports a spectacular error that does not exist.
- LTV's `kV` default of 6.2 is derived from a 3.25 in wheel; the robot's 2.6738 in
  wheel implies 7.5. Swept **after** the IMU fix: 1.27 in at kV 4, 0.57 in at 6.2,
  0.45 in at 7.5, **0.37 in at 9.0**, 0.69 in at 12 — so 6.2 is too low and the
  physical derivation is the better guide. Swept *before* the fix the same sweep
  put the optimum at 6.2 and made 7.5 look worse: a clean example of a fixture
  fault producing a confident wrong tuning answer. Settle the final value on
  hardware; the sim has no load model.

## Trajectory-stack defects the sim exposed

The trajectory stack had never run outside host unit tests. Driving it through the
world found six defects; they are being fixed under their own review.

1. **`WheelVoltageConstraint` emits states demanding more than 12 V.** Its window
   assumes `a_wheel = a * c` with `c = 1 +/- k*b/2`. The true demand is
   `d/dt(v*c) = a*c + v^2 * (b/2) * dk/ds`, and `dk/ds` is nonzero everywhere on a
   spline. On a 6.7 in-radius path at `kA = 0.8` the constraint reports exactly
   12.000 V while the reconstructed per-wheel demand reaches **14.79 V**. It
   survived its host test because `recheck()` validates the profile by calling
   `accelerationLimits()` — the same rigid-ratio model it was meant to check.
   This is the same bug class review round 1 missed, and for the same reason.
2. **`generateTrajectory` fails on a 48 in straight line when sampling is
   refined** — `maxDx = 0.5` works, `0.25` returns `ProfileDidNotConverge`.
   Refining sampling cannot make a feasible path infeasible. `(v^2 - v0^2)/(2*ds)`
   cancels in float32 below `ACCEL_CONVERGE_EPS = 1e-3`, and the forward pass then
   takes a branch that is a no-op when no constraint is active, so the fixed point
   cannot move and burns all 32 iterations.
3. **The voltage constraint is a silent no-op with the shipped gains** —
   `kaStraight = 0` makes `maxWheelAccelAt` return infinity. Adding it to a
   constraint list today buys nothing and says nothing. Defect 1 goes live the
   moment `kaStraight` is measured.
4. **`TrajectoryFollowParams::holdAtEnd` cannot correct anything.** Past the end
   the reference is parked at zero velocity, so `k = 2*zeta*sqrt(wRef^2 + b*vRef^2)`
   is zero and both outputs are identically zero. Enabling it spends 0.52 s longer,
   ends slightly worse, and downgrades the outcome from `arrived` to `stalled`.
5. **`holdAtEnd = false` reports `arrived` regardless of the tolerance** — with
   hold off, only the clock is consulted.
6. **`trajectory_follow.hpp` overclaims** that the shaping constraints are already
   respected. Without `DrivetrainConstraints` the same waypoints demand 118.7 in/s
   of a 63 in/s drivetrain.

Also: **quintic auto-tangents can cusp.** Waypoint headings of 45/0/90/0 degrees —
nothing a human would flag — give `|k| = 4.95 1/in`, a 0.20 in radius, which
unconstrained demands 1922 in/s and 300 V.

## Model limits worth knowing, deliberately not changed

- **Motor velocity is exact.** `velocityRpm` is unquantised and unfiltered while
  `positionDeg` is quantised to 0.1 deg, so the velocity is more precise than the
  encoder it is nominally derived from. A real V5 reports coarse, filtered
  velocity. This matters most to the MPC/TVLQR work, which is velocity-feedback
  heavy: it sees a signal it will not get. Measurable on the robot by logging
  `get_actual_velocity()` at 10 ms through a voltage ramp.
- **`JamDetector` can never fire here.** `currentMa` is a function of the command
  alone, so a stalled motor draws what a free one does, and
  `get_efficiency()` returns a constant 100.0 against a 7-20% band. No deadband,
  no stiction, no battery sag.
- **`responseTauMs = 70` implies 900 in/s^2, i.e. 2.33 g** on a full-scale step.
  Traction rubber on foam tiles at mu ~ 1 caps that near 386 in/s^2, so tau = 163 ms
  is where the model stops claiming super-traction acceleration. Unchanged because
  every gain in `config.cpp` was fitted at 70 ms; measure mu first.
- **The collision proxy is 5.303 in**, which is neither the inscribed (7.5) nor the
  circumscribed (10.6) radius of the footprint, and smaller than both — a 15x15
  robot squares to a wall with its corners 2.2 in inside it. `robotLengthIn` and
  `robotWidthIn` are unmeasured placeholders, so fixing the formula while its
  inputs are guesses only moves an arbitrary number.
- **Cone aliasing past 55 in.** 7 rays over 25 deg is 4.167 deg spacing, and a 2 in
  post subtends less than that beyond 55.0 in, so between there and the 78.7 in
  ceiling a post can fall entirely between two rays. `distConeRays = 11` removes
  it at 1.6x the raycast cost, but it changes which readings MCL discards and
  MCL/EKF were tuned against 7.
- **Device phase is locked to the control loop**, so the staleness pattern repeats
  exactly every run. A real brain beats a ~10 ms task against a ~10 ms IMU. Adding
  per-device phase would expose the aliasing at the cost of exact replay.

## Constrained MPC: reachable at last, and it fits the tick

The constrained surface added last round was unreachable from the sim. Worse, the
suite's closed-loop section had **never run the MPC at all** — `Mode` is
`{ScheduledLqr=0, RiccatiPerTick=1, MpcConverged=2, MpcRti=3}` and the cases
labelled "RTI" and "full" were modes 1 and 0, so `CondensedMpc` was only ever
exercised through the static checkers. With a corridor configured, mode 1 reports
`admmTicks 0, constraintRows 0`; mode 3 reports `232/232` and 30 rows. Two more
fixture faults sat behind it: `sim_mpc_bench_json` was cwrapped with 4 arguments
for a 5-argument function, so every "fits a control tick" number was a *single*
tick dominated by the clock read; and `sim_mpc_set` silently ignored unknown
keys, so any constraint knob would have been dropped without a word.

**Timing against a 10 ms V5 tick** (667 MHz Cortex-A9, scalar VFPv3, ~6.67 Mflop
per tick as a generous ceiling): every *feasible* constrained configuration sits
at or under **2.4% of the budget** — corridor+rate at horizon 20, fully
converged, is ~0.24 ms of estimated A9 time. The only configuration that
threatens it is an **infeasible** one, which burns the full 400-iteration cap
every tick at ~3.9 ms. **The cost of a constrained MPC on a V5 is decided by
feasibility, not by horizon or row count.** `sizeof(CondensedMpc)` is 63.0 KiB
and `sizeof(MpcPathFollower)` 64.4 KiB, all preallocated — fine on a V5, but
neither may ever be a local.

**Library defects found.** `AdmmQp`'s stall heuristic reported `primalInfeasible`
on demonstrably feasible problems, and did so *more* often the longer it ran:
0/400 ticks at 16 iterations, 137/400 at 80, 296/400 at 160, while the plan sat
exactly on the corridor boundary. Cause: with `tol.primal == 0` — what the RTI
path uses to burn a fixed budget — the converged branch can never fire, so a
residual that has bottomed out at the float32 floor looks exactly like one stuck
above tolerance. Fixed by gating the heuristic on a positive primal tolerance;
above zero it cannot misfire, because the converged branch would already have
broken out. Pinned by a host test that reproduces the floor with a tilted active
row (an axis-aligned box row reaches exact zero and converges, which is why it
does not reproduce it) and goes red without the fix.

Also documented rather than changed: an infeasible corridor combined with a rate
limit destabilises the loop (peak cross-track 0.514 m against 0.0575 m
unconstrained) and **no boolean diagnostic fires** — only `worstPrimalResidual`;
and `setReferenceDeltas` cannot affect the committed input, since `solve()` hard
-clamps the head against the rate anchor, so the deltas shape the plan only.
`addStateConstraint`'s row-budget refusal is unreachable by construction
(`2N + 2N + 4N = 160 = MPC_MAX_QP_ROWS` at N=20) — not a bug, but the arithmetic
is now pinned so raising one bound without the other fails loudly.

Measured benefit, corridor 0.02 m at 1 iteration in the world: gentle-S peak
cross-track 1.56 -> 0.84 in, hairpin 1.23 -> 0.87 in. At one iteration a corridor
acts as a heavy penalty rather than a constraint, exactly as the header says.

## Motion outcomes, stalls and chaining are observable now

None of `SettleMonitor`, `MotionOutcome`, `lastMotionArrived`, `EncoderAverage` or
`JamStateMachine` appeared anywhere in sim/. They do now, along with the EKF's
gates and reject streaks, MCL's generation/resample/mismatch, and per-sensor
`EncoderAverage`/`WheelReading` counts.

Two more world-model faults fell out of it. **A pinned robot fabricated wheel
travel**: held against the perimeter it reported 51.6 in/s indefinitely, so
`moveDistance(40, 2500)` from x=+40 returned **`Arrived` with the odometry pose at
x=79.3 — 9 in outside a 70.2 in wall**. A wall stall was therefore unreachable and
every motion that grazed a post reported a false arrival. Now the encoders and the
reported wheel speed follow the achieved motion, and rotation survives. Two test
poses had to move as a result, both of which had been driving into geometry
unknowingly. **And `sim_state_json` could overflow its buffer** rather than
truncate: `n += snprintf(buf + n, sizeof(buf) - n, ...)` underflows the size
argument to an enormous `size_t` once `n` passes the end, and eight sensors of
detail were already within 2x of the 8 KB bound.

Findings reported and not changed: a boomerang reaches `Arrived` without ever
passing through `inTolerance`, so "time to settle" is unavailable for
boomerang/path motions; odometry's outage hold is capped by the 24 in jump guard,
which at full speed is ~380 ms of blackout before the catch-up is rejected
(measured exact recovery at 300/500/700 ms, 28.7 in lost at 900 ms); and
`MclCore` has no cumulative resample counter, only `resampledLast()`.

## The trajectory stack: three generator defects fixed, and one follower defect

The generator's wheel-voltage constraint dropped the `v^2 (b/2) dk/ds` term, so a
wheel accelerates at `a*c + v^2 (b/2) dk/ds` and the constraint bounded only the
first half. `TrajectoryConstraint` now takes `curvatureRate` with **no default** —
deliberately, so no implementation can quietly assume the term away again. The
true wheel demand now peaks at 11.85-11.98 V with **0 of 754 wheel-states over
budget**, where it previously demanded 14.8 V of a 12 V battery, and it survives
being differenced on a uniform 10 ms grid (worst 11.93 V over 254 samples).

Note what this does to the audit's own numbers: the *rigid* model now reads
**above** 12 V (14.07 V at kA=0.8) and that is correct. It drops the dk/ds term,
so wherever dk/ds works against a wheel the rigid figure overstates the real
demand. Three suite assertions had been guarding on the rigid figure sitting at
exactly 12 V — the premise the fix invalidates — and now guard on the true demand
reaching the rail, which is what makes "never over 12 V" non-vacuous.

**`kA = 0` is now refused rather than ignored.** `maxWheelAccelAt()` returned
infinity for `kA <= 0`, so the constraint narrowed nothing and warned nobody;
generation now returns `TrajectoryError::UnusableConstraint` with an empty
trajectory. **`config.cpp` ships `kaStraight = 0`, so adding
`WheelVoltageConstraint` today fails loudly instead of silently protecting
nothing** — kA has to be measured before that constraint is usable.

**`holdAtEnd` could not correct anything.** Every term of the RAMSETE law is
multiplied by the reference velocity, and a trajectory ends at rest, so
`k = 2 zeta sqrt(w^2 + b v^2)` and both outputs were identically zero: holding
delayed the exit and downgraded the outcome, nothing more. Measured 0.839 in and
`stalled` with the hold against 0.786 in and `arrived` without it. The follower
now hands over to the same settle PIDs every other motion uses once the reference
is spent, projecting error onto the final heading axis via the existing
`straightDemand` primitive — so longitudinal and heading error are corrected and
perpendicular error is deliberately left alone, since closing that from rest
needs a turn-drive-turn nobody asked for. Now **0.238 in and `arrived`**, against
0.786 in unheld.

This one is sim-verified only: it lives in `chassis.cpp`, which pulls in PROS and
is not host-testable.

## A gap in the verification harness itself

`test/syntax-check.sh` covers the 56 firmware sources and **not** the wasm bridge,
so a constraint-interface change compiled clean for ARM while
`sim/bridge/bridge_traj.cpp` was still calling the old three-argument signature.
The ARM check passing is not evidence the simulator builds. Separately,
`errorName()` in that bridge had never been taught `UnusableConstraint` or
`CuspedPath`, so both rendered as `"unknown"` — a switch over an enum that grew.

## Kidnap recovery: the mechanism was dead code in the shipped configuration

Recovery from a 24 in kidnap ran at 14/19. The cause was not tuning. `roughening()`
ramps the cloud's spread from `distResampleVariance` up to `distSearchVariance`
when the estimate stops fitting -- and **`config.cpp` ships 1.5 for the first
while the second also defaulted to 1.5, so the ramp's amplitude was identically
zero.** Proof: changing `searchMismatchFull` or `injectResampleFloor` moved the
outcome by nothing at all, to three decimals. The mechanism that round 2 cited to
justify leaving augmented-MCL injection off did not exist at runtime.

The header comment above that field claimed 0.15 was "shipped here". It is the
*default*; the robot runs 1.5. That gap is the whole defect, and it is now called
out in the header and gated by `test/check-defaults.py`.

**The ramp was also driven by the wrong quantity.** `likelihoodMismatch()` is the
augmented-MCL fast/slow gap on the cloud's *mean* likelihood, which falls when the
cloud merely spreads -- so a width driven by it widens because it widened.
Measured p90 during healthy driving: **0.36, against a `searchMismatchFull` of
0.35** -- saturated when it should have been off. Forcing the old ramp on with a
6 in width produced 17.7-61 in excursions from a filter that was never lost.

It now runs on `MclCore::fitSigmas()`, the RMS range residual of the published
estimate in sigmas, which is absolute rather than relative:

| | tracking | 2.5 in out | 10 in out | 24 in kidnap |
|---|---|---|---|---|
| `fitSigmas` | 0.25 | 0.26 | 0.70 | **6.47** |

Two robustness properties, both tested: the **median** of the per-beam residuals,
not the mean, because one beam on something unmapped is tens of sigmas and a mean
of four would declare a healthy filter lost (disabling the deadzone model costs
12.9 in with the median against 30 in with the mean); and a **per-beam cap** so
one tick cannot cross the ramp floor, since a blocker across a sensor is 800
sigmas when a close reading's sigma is 0.068 in.

| 24 in kidnap, 4 poses x 8 directions | before | after |
|---|---|---|
| MCL raw recovered | 14/19 @ 484 ms | **18/19 @ 491 ms** |
| fused, 2.5 s budget | 13/19 @ 834 ms | 15/19 @ 740 ms |
| fused, 5 s budget | — | 18/19 @ 1264 ms |
| steady MCL error | 1.48 in (worst 7.1) | 1.52 in (worst 8.3) |
| steady fused error | 0.395 in | 0.393 in |

## The EKF relocation gate was a green light at the worst moment

`mclMinNeffFrac` required `nEff/N >= 0.20` as a "healthy cloud" test. ESS is
**high when the cloud is lost** -- every particle equally wrong flattens the
weights -- so the gate passed while lost and failed while tracking. And it fired:
one seed relocated at ~150 ms onto an MCL estimate **11.6 in from truth**, writing
a 2.64 in sigma. Worse, `nEff/N` is **non-monotone** in displacement (0.46-0.61
converged, 0.008 at 10 in out, 0.86 at 24 in, 0.001 uniform), so **no threshold in
either direction separates a good fix from a bad one.**

Replaced with evidence rather than self-report: `compareRangeFit()` scores the
beams under two hypotheses -- the filter's own state, and MCL's position with the
filter's heading -- on exactly the beams both can predict, and relocates only when
MCL's chi-square is `mclRelocEvidenceRatio` (12) times smaller *and* the filter's
own exceeds one per beam. `mclMinNeffFrac` is gone; `nEff` stays as a diagnostic.

## Injection stays off, now on measurement rather than reasoning

`maxRandomFrac = 0.10` recovers 19/19 at 388 ms but takes steady MCL error to
**7.44 in with a 126 in worst case**. The stated reason is real: posts are
registered as deadzone circles, so an injected particle aiming a beam at one
scores `deadzoneLikelihood` (0.06/in) instead of the ~1.3e-4/in a wrong hypothesis
deserves -- **460x better than being wrong**, which is how a hypothesis 60 in from
truth survives. The search width buys the same recovery for 1.52 in.

## That tuning option, now settled: half of it applied

The 19-trial set could not resolve this. It was resolved on **648 paired trials**
-- 8 starts x 16 directions x 8 seed bases -- every cell reporting raw MCL, fused
pose and recovery, paired on the trial so McNemar applies:

| `distResampleVariance` / `distSearchVariance` | raw rec | fused rec | steady raw (worst) | fused (worst) | fused p vs shipped |
|---|---|---|---|---|---|
| 1.5 / 6 (was shipped) | 628/648 | 515/648 | 1.53 (7.7) | 0.393 (1.12) | 0.0000 worse |
| **1.5 / 12 (now shipped)** | **637/648** | **550/648** | 1.56 (7.2) | 0.398 (1.12) | -- |
| 1.0 / 12 | 626/648 | 533/648 | 1.29 (5.0) | 0.397 (1.02) | 0.0030 worse |
| 0.6 / 12 | 634/648 | 537/648 | 1.06 (3.2) | 0.432 (1.03) | 0.024 worse |
| 0.15 / 12 | 622/648 | 528/648 | 0.65 (1.7) | 0.446 (1.06) | 0.0013 worse |

So **half of the candidate is right and half is wrong**: `distSearchVariance` goes
to 12, and `distResampleVariance` stays at 1.5 because every lower value tried
loses fused recoveries at p <= 0.024. The raw-MCL gain from a tighter cloud is
real and large -- 2.4x on the mean, 4.2x on the worst case -- and it is not what
the robot drives on.

Raising `ekfMclSigmaFloor` to 3.6 does recover the tighter cloud's fused *mean*
(0.466 -> 0.363 in), because a 1.2 in floor makes the EKF trust a converged cloud
more than its own range fusion deserves. It does not recover the fused *rate*, and
at the shipped 1.5 roughening a higher floor is worse on both. 1.2 stays.

### And a cost that only showed up outside this file

`distSearchVariance` was also the `setPose()` acquisition envelope. Raising it to
12 widened the cloud at the start of every run, which the trajectory suite caught
as a landing error going 0.79 -> 1.76 in and the MPC suite as a corridor
excursion. Acquisition is now `acquireVariance`, still 6 in.

Same file, `obstacleRangeTol` (8 in) looks loose -- 3 in gives better steady error
on every axis in the sim -- but 8 in is what forgives error in the real post
positions, and choosing needs the field.

## Two claims of mine the agent corrected

I briefed that the relocation gate was "backwards". It was worse than backwards:
non-monotone, so no threshold works. And I asserted that "a stationary robot cannot
relocate -- it gets no new information". Four absolute ranges pin a position given
the heading: measured **14/28 parked** against 18/19 driving. Harder, not
impossible. `recovery.mjs` asserts the difference rather than an impossibility.

## Two harness faults that had been making earlier numbers lies

`sim_reset` does **not** restore the filter configs or the world parameters, so a
probe silently ran trial N with trial N-1's leftovers -- a test that blinded the
beams then blinded every trial after it, and three assertions went vacuous.
`recovery.mjs` now writes a full baseline before every trial and checks it back
through `sim_get_param`, so a `config.cpp` move fails loudly. Separately, **the
first trial after `sim_init` is not comparable to any later one** (identical inputs
land ground truth 0.28 in elsewhere, the filter tasks not existing yet), so a
warm-up trial is discarded.

## Independent review round: what two critics found

Both returned FAIL. The findings that changed conclusions rather than code:

**Every reference path in the trajectory and MPC suites was illegal.** Both suites set
`collideObstacles = 0` in their setup and then asserted `collideTicks() === 0` labelled
"never touched field geometry" -- with the posts off, `Field::collides` returns after the
perimeter test, so it could not fail. Behind it, all four trajectory paths ran inside a
post's collision envelope (2.0 in post + 5.303 in proxy), `wide arc` passing **1.50 in from
a post centre**. Paths moved to genuinely clear space with the obstacles ON. `world.mjs`'s
pure-spin pose was also sitting on the centre post, so "a pure spin does not translate"
passed because contact forced the translation to zero, not because the kinematics are
symmetric.

**`world.mjs` could not detect a wrong track width.** `b = 2v/omega` is an identity against
how `world.cpp` computes omega, so a **15% error passed all 59 checks**. Now closed against
the drivetrain's own value through a different bridge entry point: measured 11.9163 in
against a declared 11.9180 in.

**The MPC timing claim was the wrong statistic.** `flopsPerTick` was `meanIters x
flopsPerIter`, and a loop misses its deadline on its worst tick. Box ADMM at horizon 20,
converged, has a worst tick of **2.69 ms (27% of budget)** against a 0.07 ms mean;
corridor+rate reaches **4.22 ms (42%)**. The old assertion was also unreachable -- the worst
value the bench could produce was 2.43e6 flop against a 6.67e6 ceiling. Both are now
reported and the budget test is on the worst.

**The wheel-voltage guarantee does not hold exactly.** The audit paired the start-of-interval
wheel speed with the interval acceleration, missing the peak at the interval's end. Corrected,
kA=0.4 still holds with 0 states over, kA=0.8 lands 20 of 754 over at 12.045 V and kA=2.0
lands 97 over at 12.106 V. This is the **generator** understating, not the audit
overstating: with piecewise-constant acceleration v(t) is linear on each interval, so the
demand is monotone there and the max of the two endpoints is exact. It is under 1% of the
rail against 14.79 V before the constraint existed, and `config.cpp` ships `kaStraight = 0`,
which is refused outright.

**The kidnap improvement is not statistically significant.** 14/19 -> 18/19 is McNemar
p = 0.0625 one-sided. The 95-98% rate itself is real -- it held over 5x the trials
(91/93 at 8 starts x 16 directions) and across four seed bases -- but 19 trials cannot
establish the improvement. Expanding the sweep is nearly free and has not been done.

**Two vacuous checks with named mutations.** The route-DSL opcode check sorted by value and
then asserted `OP[order[i]] === i`, true for any bijection: swapping `DRIVE` and `SWING`
passed. And `smoke.mjs`'s RAMSETE overshoot checks assert a `min` over the run, which an
overshoot passes straight through -- still open.

## Library defects the second critic reproduced

**A stack-buffer-overflow.** `MclCore::updateFit` wrote past an 8-element stack array with
no bound on the caller-supplied beam count, confirmed under AddressSanitizer. Both sibling
loops in the same file clamp it and one says so in a comment. Fixed.

**A relocation that went 96 inches the wrong way.** `rangesPreferMcl` scored the two
hypotheses with different noise scales, because `rangeSigma` divides by `cos(incidence)` and
incidence belongs to the hypothesis, not the reading. At the incidence limit the sigmas
differ 2.45x -- 6x in chi-square against an evidence ratio of 4 -- so geometry alone could
decide. Comparing raw chi-squares is only a likelihood ratio when the noise scale is shared.
Fixed by scoring both at normal incidence. The test asserting this property only passed
because every relocation test used axis-aligned mounts.

**`holdAtEnd` reported `Stalled` on a perpendicular residual.** `atTarget` tested Euclidean
distance, but the hold closes only the along-axis projection, so a perpendicular error left
both PIDs at zero and the arrival test structurally unreachable -- while the same physical
outcome with the flag OFF reported `Arrived`. Fixed by judging arrival on the error the hold
can actually close, keeping the full error as the settle error.

**The ADMM infeasibility gate was fixed at the wrong boundary.** Gating on `tol.primal > 0`
fixed exactly one value; the real boundary is the float32 residual floor, and it still
misfired at 1e-9, 1e-8 and 1e-7. Now gated on `8*eps*||Cz||`. OSQP's relative primal term
was tried first, as the critic suggested, and rejected on measurement: it moves the solution
enough to miss an active-set reference by 3.4e-4 against a 3e-4 tolerance.

**`v*|v|` versus `v*v` had no test at all.** A deliberate, load-bearing sign convention in
the dk/ds term: nothing in the suite distinguished them, because the only reversed-path test
carries grip and kinematics, neither of which reads the sign of speed. `v*v` -- which reads
as the obvious form, and which an earlier brief of mine asserted was correct -- demands
**16.4 to 23.5 V of a 12 V rail** on a reversed path with up to 171 states over. One line of
coverage added; it goes red at all three kA values.

**The `fitSigmas` per-beam cap guarantees less than it claimed.** One outlier tick cannot
cross the ramp floor only from below 1.33 sigmas; above that it can. Stated conditionally
with the derivation rather than changed, because bounding the per-tick rise instead -- the
suggested alternative -- makes the level asymptote below the floor and the ramp never fire.

## Still open

- **No penetration recovery in the world model.** `updateDrive` gates on the proposed pose
  only, so a robot whose current pose already collides is permanently immobilised: teleport
  into geometry and every direction is rejected forever, reporting zero wheel speed.
  `sim_teleport` is the kidnap primitive, and `recovery.mjs` is safe only by two accidents.
- **`Field::clear()` does not clear the collision perimeter.** `field.cpp` tests against a
  file-scope `kHalf = 70.2`, not against the loaded segments, so with no geometry at all the
  robot still stops at x = 64.85. The header claims geometry is data-driven and replaceable
  at runtime; for collision it is not.
- **`worstPrimalResidual`'s units.** `mpc_follower.hpp` promises metres; a +/-0.002 m corridor
  at one RTI iteration reports 390.26 with `relaxedRows = 0`, which cannot be a distance.
  Unexplained.
- **`smoke.mjs`'s two RAMSETE overshoot checks** assert a `min` over the run (see above).
- **RCL's gating knobs are unexposed** -- `angleTolerance`, `maxWallInnovation`,
  `confidenceThreshold`, `maxSyncDist`, `recoverySyncProp`, `displaceCycles`,
  `noReturnWallLimit`. The EKF's equivalents all have them.

## Needs a robot

The MCL/EKF changes above alter a tuned localization stack. Re-verify kidnap
recovery, and the range gate against real field geometry, before trusting either.
