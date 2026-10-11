#!/usr/bin/env bash
# Build chad-sim to WebAssembly.
#
#   ./sim/build.sh            release build into sim/web/
#   ./sim/build.sh debug      assertions + source maps
#
# Requires the Emscripten SDK. Set EMSDK to override the default location.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

EMSDK="${EMSDK:-$HOME/emsdk}"

# A Python that actually runs. Each candidate is run, not just looked up: on
# Windows python3 and python usually resolve first to the Microsoft Store
# placeholder, which exists, prints "Python was not found" and fails. emsdk's
# bundled one comes first; type -ap lists every match on PATH after it, so a
# real install further down still counts.
PY=""
for cand in "$EMSDK"/python/*/python.exe "$EMSDK"/python/*/bin/python3 \
            $(type -ap python3 python py 2>/dev/null); do
    [ -e "$cand" ] || command -v "$cand" >/dev/null 2>&1 || continue
    if "$cand" -c "import sys; sys.exit(0 if sys.version_info >= (3, 6) else 1)" \
            >/dev/null 2>&1; then
        PY="$cand"
        break
    fi
done
# emsdk_env.sh and em++ both run Python, and their own lookup looks for the
# bundled one at a Linux path and then falls back to that same placeholder.
if [ -n "$PY" ] && [ -z "${EMSDK_PYTHON:-}" ]; then
    export EMSDK_PYTHON="$PY"
fi

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
OUT="sim/web/chad_sim.mjs"

# The whole chad library, plus the game-specific config and routines, so the
# sim runs the same code that ships to the brain. autonselector.cpp is replaced
# by a non-LVGL stub; main.cpp is excluded because the bridge owns the entry
# points instead of the competition control system.
#
# Both trees are globbed rather than listed. The old list named
# src/gamespecific/autons.cpp, which stopped existing when the routines moved
# into src/gamespecific/autons/ -- and nothing noticed, because the one gate
# that claims to build the sim was not calling this script.
SOURCES=(
    $(find src/chad -name '*.cpp' -not -path '*auton_selector*')
    $(find src/gamespecific -name '*.cpp')
    sim/platform/autonselector_sim.cpp
    sim/platform/pros_support.cpp
    sim/platform/sim_runtime.cpp
    sim/platform/sim_sched.cpp
    sim/world/actors.cpp
    sim/world/field.cpp
    sim/world/pieces.cpp
    sim/world/toggles.cpp
    sim/world/world.cpp
    sim/bridge/bridge.cpp
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

mkdir -p sim/web

# The front end draws a compiled auton's route by parsing the source it was
# built from, so emit that next to the wasm and they cannot drift.
if [ -n "$PY" ]; then
    "$PY" sim/auton_sources.py
else
    echo "warning: no working python, compiled autons will have no planned path" >&2
    echo "  (on Windows: Settings > Apps > Advanced app settings > App execution" >&2
    echo "   aliases, turn off python.exe and python3.exe, or install Python)" >&2
fi

echo "building ${#SOURCES[@]} sources -> $OUT ($MODE)"
em++ "${SOURCES[@]}" "${FLAGS[@]}" -o "$OUT"

echo "done:"
ls -lh sim/web/chad_sim.mjs sim/web/chad_sim.wasm | awk '{printf "  %-28s %s\n", $9, $5}'
