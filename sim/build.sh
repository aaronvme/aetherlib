#!/usr/bin/env bash
# Build chad-sim to WebAssembly.
#
#   ./sim/build.sh            release build into sim/web/
#   ./sim/build.sh debug      assertions + source maps
#
# Requires the Emscripten SDK. Set EMSDK to override the default location.
#
# CHAD_SIM_OUT=<dir> redirects the build to a private output directory. The
# shared sim/web/ is a single pair of files, so two builds running at once
# corrupt each other -- give every concurrent build its own directory, the same
# way test/host/run.ps1 takes -OutDir.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

EMSDK="${EMSDK:-$HOME/emsdk}"
if ! command -v em++ >/dev/null 2>&1; then
    if [ -f "$EMSDK/emsdk_env.sh" ]; then
        # shellcheck disable=SC1091
        source "$EMSDK/emsdk_env.sh" >/dev/null 2>&1
    else
        echo "error: emcc not found and no emsdk_env.sh at $EMSDK" >&2
        exit 1
    fi
fi

MODE="${1:-release}"
OUTDIR="${CHAD_SIM_OUT:-sim/web}"
OUT="$OUTDIR/chad_sim.mjs"

# The whole chad library, plus every game-specific source, so the sim runs the
# same code that ships to the brain. Discovered rather than listed, because a
# hand-written list silently stops building a routine the moment one is added.
# autonselector.cpp is replaced by a non-LVGL stub; src/main.cpp is excluded
# because the bridge owns the entry points, not the competition control system.
SOURCES=(
    $(find src/chad -name '*.cpp' -not -path '*auton_selector*')
    $(find src/gamespecific -name '*.cpp')
    sim/platform/autonselector_sim.cpp
    sim/platform/pros_support.cpp
    sim/platform/sim_runtime.cpp
    sim/platform/sim_sched.cpp
    sim/world/field.cpp
    sim/world/world.cpp
    $(find sim/bridge -name '*.cpp')
)

FLAGS=(
    -std=gnu++26
    -Isim/platform          # shim pros/ headers must win over include/pros/
    -Iinclude
    -sASYNCIFY=1            # required for emscripten_fiber_t
    -sASYNCIFY_STACK_SIZE=32768
    -sALLOW_MEMORY_GROWTH=1
    -sINITIAL_MEMORY=67108864
    -sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','UTF8ToString','HEAPF32','HEAP32']
    -sEXPORT_NAME=createChadSim
    -sMODULARIZE=1
    -sEXPORT_ES6=1
    -sENVIRONMENT=web,node
    -Wno-unused-command-line-argument
)

if [ "$MODE" = "debug" ]; then
    FLAGS+=(-O1 -g2 -sASSERTIONS=2 -sSAFE_HEAP=0 -sSTACK_OVERFLOW_CHECK=2)
else
    FLAGS+=(-O3 -sASSERTIONS=0)
fi

mkdir -p "$OUTDIR"
echo "building ${#SOURCES[@]} sources -> $OUT ($MODE)"
em++ "${SOURCES[@]}" "${FLAGS[@]}" -o "$OUT"

echo "done:"
ls -lh "$OUTDIR/chad_sim.mjs" "$OUTDIR/chad_sim.wasm" | awk '{printf "  %-28s %s\n", $9, $5}'
