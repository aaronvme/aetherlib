#!/usr/bin/env bash
# Build chad-sim and run its whole test suite.
#
#   ./sim/test/run.sh             rebuild, then test
#   ./sim/test/run.sh --no-build  test whatever is already in sim/web/
#
# The rebuild is the default because the suites otherwise pass against a
# stale wasm. test/verify.sh used to call this with an --out flag that was
# silently ignored, so its "simulator builds" gate never built anything --
# and sim/build.sh had been broken for several commits underneath it. Hence
# the unknown-flag check below: quietly accepting a flag is how that hid.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

BUILD=1
while [ $# -gt 0 ]; do
    case "$1" in
        --build)    BUILD=1 ;;
        --no-build) BUILD=0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

if [ "$BUILD" = 1 ]; then
    ./sim/build.sh
    echo
fi

if [ ! -f sim/web/chad_sim.mjs ]; then
    echo "no build found -- run ./sim/build.sh first" >&2
    exit 1
fi

fail=0
echo "--- route DSL ---"
node sim/test/dsl.mjs       || fail=1
echo "--- auton source ---"
node sim/test/auton_cpp.mjs || fail=1
echo "--- front end boots ---"
node sim/test/app_boot.mjs  || fail=1
echo "--- simulator ---"
node sim/test/smoke.mjs || fail=1
# Every motion the route editor can emit, ground truth against what the command
# asked for. This is the gate the sim tuning in sim/bridge/bridge.cpp exists to
# pass: without it, a plausible change to a gain or a tolerance quietly moves
# where a route lands and nothing else notices.
# Odometry lying is the failure mode that loses a match: the filter relocates
# onto a mirror of itself and every motion after it aims at the wrong place.
# The claw, the lift and the things they move. The last case runs a compiled
# routine unmodified and checks a pin ends up in a goal.
echo "--- game pieces ---"
node sim/test/pieces.mjs || fail=1
# One half turn per flip, one way only; R2 held in driver spins both.
echo "--- side toggles ---"
node sim/test/sidetoggle.mjs || fail=1
# A beam reset off a field axis, or onto a post-lattice alias, must not move
# the pose; a square-on one still corrects a real offset.
echo "--- beam reset ---"
node sim/test/beam_reset.mjs || fail=1
# RCL owns the pose now. The two checks that matter are the rejections: a beam
# ranging an alliance partner, and one ranging a post, must not move it.
echo "--- RCL ---"
node sim/test/rcl.mjs || fail=1
# Automatic corrections are rate limited: a recovery glides, it never jumps.
echo "--- RCL pose-jump guard ---"
node sim/test/rcl_guard.mjs || fail=1
# Per-sensor trust: a game element a few inches in front of the wall, inside
# the innovation gate, must not drag the pose; the sensor drops out and returns.
echo "--- RCL reading trust ---"
node sim/test/rcl_trust.mjs || fail=1
# All-omni drive: an angled push on a wall or post slides along it, and the
# tracking pods see the slide. Frictionless: contact params at 0.
echo "--- wall glide ---"
node sim/test/glide.mjs || fail=1
# Default contact: a corner hit squares up, the slide scrubs and stops once
# flat, a hard hit knocks back and settles.
echo "--- wall contact ---"
node sim/test/contact.mjs || fail=1
echo "--- localization under disturbance ---"
node sim/test/disturbance.mjs || fail=1
# Mode changes: a motion the autonomous task left running must not keep the
# drive from the driver, and cancelling must not swallow the next motion.
echo "--- competition modes ---"
node sim/test/competition.mjs || fail=1
# A chained move that passes beside its target must still hand off promptly.
echo "--- chained motions ---"
node sim/test/chaining.mjs || fail=1
echo "--- routing accuracy ---"
node sim/test/accuracy.mjs --gate || fail=1

if [ $fail -eq 0 ]; then
    echo "SUITE PASSED"
else
    echo "SUITE FAILED" >&2
fi
exit $fail
