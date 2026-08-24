#!/usr/bin/env bash
# Run the chad-sim regression suites.
#
#   ./sim/test/run.sh                 every suite, against the current build
#   ./sim/test/run.sh --build         rebuild first
#   ./sim/test/run.sh smoke mpc       named suites only
#   ./sim/test/run.sh --out /tmp/x    build and test in a private directory
#
# Suites are discovered rather than listed, so adding sim/test/<name>.mjs is all
# it takes to get it gated. tune and autotune are excluded: they are parameter
# sweeps that take minutes and have no pass/fail, not regression tests.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

BUILD=0
NAMES=()
while [ $# -gt 0 ]; do
    case "$1" in
        --build) BUILD=1 ;;
        --out) shift; export CHAD_SIM_OUT="$1"; export CHAD_SIM_MJS="$1/chad_sim.mjs"; BUILD=1 ;;
        -*) echo "unknown option: $1" >&2; exit 2 ;;
        *) NAMES+=("$1") ;;
    esac
    shift
done

if [ "$BUILD" = 1 ]; then
    ./sim/build.sh
    echo
fi

MJS="${CHAD_SIM_MJS:-sim/web/chad_sim.mjs}"
if [ ! -f "$MJS" ]; then
    echo "no build at $MJS -- run ./sim/build.sh first" >&2
    exit 1
fi

# dsl needs no wasm and is the fastest signal, so it goes first; smoke is the
# broadest, so it goes second. Everything else follows in a stable order.
ordered=()
for pref in dsl smoke; do
    [ -f "sim/test/$pref.mjs" ] && ordered+=("$pref")
done
for f in $(ls sim/test/*.mjs | sort); do
    name="$(basename "$f" .mjs)"
    case "$name" in
        harness|tune|autotune|dsl|smoke) continue ;;
    esac
    ordered+=("$name")
done

if [ ${#NAMES[@]} -gt 0 ]; then
    selected=()
    for want in "${NAMES[@]}"; do
        if [ -f "sim/test/$want.mjs" ]; then
            selected+=("$want")
        else
            echo "no such suite: sim/test/$want.mjs" >&2
            exit 2
        fi
    done
    ordered=("${selected[@]}")
fi

fail=0
failed=()
for name in "${ordered[@]}"; do
    echo "--- $name ---"
    if node "sim/test/$name.mjs"; then :; else
        fail=1
        failed+=("$name")
    fi
    echo
done

if [ $fail -eq 0 ]; then
    echo "SUITE PASSED (${#ordered[@]}: ${ordered[*]})"
else
    echo "SUITE FAILED (${failed[*]})" >&2
fi
exit $fail
