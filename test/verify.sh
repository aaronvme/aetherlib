#!/usr/bin/env bash
# One gate for the whole tree.
#
#     ./test/verify.sh              every gate this branch can run
#     ./test/verify.sh --no-sim     skip the simulator (no emsdk on this machine)
#
# The simulator lives on the sim branch. Gate 3 is skipped automatically when
# sim/ is absent, so this script is the same file on both branches.
#
# Three things break independently and each needs its own toolchain, so passing
# one proves nothing about the others. A constraint-interface change once
# compiled clean for ARM while the wasm bridge was still calling the old
# signature, because the ARM check does not look at sim/. On main that gate
# cannot run at all, so merge into sim to catch it.
#
#   1. arm-none-eabi   the firmware compiles          test/syntax-check.sh
#   2. MSVC            the PROS-free numerics pass    test/host/run.ps1
#   3. emscripten      the simulator builds and runs  sim/test/run.sh (sim branch)
#   4. python3         API.md's defaults are real     test/check-defaults.py
#
# A failure while a subagent is editing is not necessarily real: -OutDir isolates
# the build directory, not the source tree, so this can compile a half-written
# file. Re-run before believing it.
#
# A missing toolchain is reported as SKIP, never folded into a pass.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

WITH_SIM=1
[ "${1:-}" = "--no-sim" ] && WITH_SIM=0
[ -f sim/test/run.sh ] || WITH_SIM=2   # sim/ can survive as stale wasm output

OUT="${TMPDIR:-/tmp}/chad-verify.$$"
mkdir -p "$OUT"
trap 'rm -rf "$OUT"' EXIT

fail=0
declare -a results

step() {  # step <label> <logfile> <exit-code>
    local label="$1" log="$2" code="$3"
    if [ "$code" -eq 0 ]; then
        results+=("PASS  $label")
    else
        results+=("FAIL  $label")
        fail=1
        echo "--- $label failed, last 20 lines ---" >&2
        tail -20 "$log" >&2
        echo >&2
    fi
}

echo "== 1/4 firmware compiles for ARM =="
bash test/syntax-check.sh > "$OUT/arm.log" 2>&1
code=$?
if [ $code -eq 2 ]; then
    results+=("SKIP  ARM compile (no arm-none-eabi-g++)")
else
    step "ARM compile ($(grep -c '  ok  ' "$OUT/arm.log" || true) sources)" "$OUT/arm.log" $code
fi

echo "== 2/4 host suite =="
if command -v powershell.exe >/dev/null 2>&1; then
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File test/host/run.ps1 \
        -OutDir "$(cygpath -w "$OUT" 2>/dev/null || echo "$OUT")/host" > "$OUT/host.log" 2>&1
    code=$?
    step "host suite ($(grep -oE '[0-9]+ cases, [0-9]+ failed' "$OUT/host.log" | tail -1))" \
         "$OUT/host.log" $code
else
    results+=("SKIP  host suite (no powershell.exe)")
fi

if [ "$WITH_SIM" = 1 ]; then
    echo "== 3/4 simulator builds and its suites pass =="
    # build.sh must not be piped: it exits 0 through a pipe even when em++ fails.
    # run.sh rebuilds by default -- it used to be handed an --out it ignored,
    # which left this gate testing a checked-in wasm and calling it a build.
    ./sim/test/run.sh > "$OUT/sim.log" 2>&1
    step "sim build + suites" "$OUT/sim.log" $?
elif [ "$WITH_SIM" = 2 ]; then
    results+=("SKIP  simulator (not on this branch -- see the sim branch)")
else
    results+=("SKIP  simulator (--no-sim)")
fi

if command -v python3 >/dev/null 2>&1; then
    echo "== 4/4 documented defaults match the headers =="
    python3 test/check-defaults.py > "$OUT/docs.log" 2>&1
    step "documented defaults" "$OUT/docs.log" $?
else
    results+=("SKIP  documented defaults (no python3)")
fi

echo
printf '%s\n' "${results[@]}"
[ $fail -eq 0 ] && echo "VERIFY PASSED" || echo "VERIFY FAILED" >&2
exit $fail
