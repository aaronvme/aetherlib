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
# RCL owns the pose now. The two checks that matter are the rejections: a beam
# ranging an alliance partner, and one ranging a post, must not move it.
echo "--- RCL ---"
node sim/test/rcl.mjs || fail=1
echo "--- localization under disturbance ---"
node sim/test/disturbance.mjs || fail=1
echo "--- routing accuracy ---"
node sim/test/accuracy.mjs --gate || fail=1

if [ $fail -eq 0 ]; then
    echo "SUITE PASSED"
else
    echo "SUITE FAILED" >&2
fi
exit $fail
