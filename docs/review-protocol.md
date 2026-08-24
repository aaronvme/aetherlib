# Review protocol

How a module in this library gets signed off. Written down because the first two
rounds proved that a reviewer with no protocol produces agreement, and agreement
is worthless — round 1 passed 217 tests over a state that demanded 22 V from a
12 V battery.

## The two roles

A **builder** owns a disjoint set of files and changes them. A **critic** owns
nothing, changes nothing, and tries to break what the builder produced. They are
never the same agent, because an author cannot find the bug they just failed to
imagine.

Builders run concurrently only on disjoint file sets, and only with private
verification directories — `run.ps1 -Tests <own> -OutDir <own>` and
`test/syntax-check.sh`, never `make`, which has one shared build directory and
will corrupt itself if two builds overlap.

## What the critic is asked to do

1. **Reproduce before reporting.** A finding is a failing test, a hand-worked
   counterexample, or a line-cited derivation. "This looks fragile" is not a
   finding. If it cannot be demonstrated, it is a question, and it goes in a
   separate list from the defects.
2. **Read the reference implementation, not a memory of it.** WPILib, OSQP, and
   the Probabilistic Robotics algorithms are all obtainable. Compare against the
   real source.
3. **Attack the tests, not just the code.** The highest-value finding available
   is a test that cannot fail. Round 1's worst bug survived because its test
   took the *union* of two constraint windows, which is never tighter than
   either one. For every test, ask: what mutation of the code would this catch?
   If the answer is none, the test is a liability — it buys false confidence.
4. **Assert against physics, not against the code under test.** Reconstruct the
   wheel voltages a trajectory state demands and compare them to the battery.
   Differentiate a Jacobian numerically. Integrate an ODE 400k steps and compare
   the closed form. A test that re-derives the implementation's own formula
   proves only that the formula was typed twice.
5. **Judge embedded reality.** Cortex-A9, float32, no watchdog, ~100 Hz control
   loop, and a battery that browns out. An unbounded `while(true)`, a heap
   allocation in a control tick, a convergence tolerance below float epsilon, or
   an unguarded divide is a field-loss defect, not a style note.
6. **Say which is better, blind.** Put this library's implementation and the
   comparison implementation side by side with the names stripped, and state
   plainly which one is better and why. `docs/research/` holds what other VEX
   libraries and industry actually ship, with sources. Where no peer exists —
   there is no published VEX MPC — say so instead of inventing a comparison.
7. **Return a verdict.** `PASS` or `FAIL`, and `FAIL` if any defect is
   reproduced. Ranked most severe first. No credit for effort.

## What the critic must not do

Do not accept a claim because the builder's report asserts it. Do not accept a
passing suite as evidence of correctness — establish that the suite could have
failed. Do not soften a verdict to be agreeable; the user asked for a harsh
critic precisely because a polite one is useless. Do not report style
preferences as defects, and do not pad a report to look thorough.

## The loop terminates when

Every reproduced defect is fixed, each with a regression test that fails against
the old code, and a critic that has read the reference implementations returns
`PASS` on its own terms. Not when the builder says it is done.

## Standing constraints on this repo

- Nothing is committed. The user reviews the working tree.
- Comments are minimal: only what stops a competent reader getting it wrong.
- Numerics stay PROS-free, so they can be host-tested. Hardware access lives in
  a thin adapter over a testable core — `util/pid.hpp` is the pattern.
- There is no simulator and no robot in the loop here. Every behaviour change to
  a tuned constant or control law must be reported as something to re-verify on
  the robot, and an untestable refactor of a tuned controller is a bad trade.
