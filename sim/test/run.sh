#!/usr/bin/env bash
# Run the whole riku-sim test suite.
#
#   ./sim/test/run.sh          test against the current build
#   ./sim/test/run.sh --build  rebuild first
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

if [ "${1:-}" = "--build" ]; then
    ./sim/build.sh
    echo
fi

if [ ! -f sim/web/riku_sim.mjs ]; then
    echo "no build found -- run ./sim/build.sh first" >&2
    exit 1
fi

fail=0
echo "--- route DSL ---"
node sim/test/dsl.mjs   || fail=1
echo "--- simulator ---"
node sim/test/smoke.mjs || fail=1

if [ $fail -eq 0 ]; then
    echo "SUITE PASSED"
else
    echo "SUITE FAILED" >&2
fi
exit $fail
