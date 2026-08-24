#!/bin/sh
# Compile-check PROS-dependent sources with the real ARM toolchain, without
# touching bin/ or .d/ -- so it is safe to run while another build is going and
# safe for several to run at once.
#
#     ./test/syntax-check.sh src/chad/chassis.cpp src/chad/tracking/ekf_tracking.cpp
#     ./test/syntax-check.sh            # every src/**/*.cpp
#
# Windows MAX_PATH: liblvgl includes itself through a long chain of ../ and
# the accumulated path is not canonicalised, so from a checkout more than
# about 120 characters deep every LVGL-dependent source fails with
# "../lv_conf_internal.h: No such file or directory". That is the checkout
# location, not the code -- 20 sources failed from a 150-character path and
# all of them passed from a 17-character one. Check where you are before
# believing it.
set -e
cd "$(dirname "$0")/.."

# The PROS toolchain ships with the VS Code extension and is not on the
# default PATH, so do not rely on the caller having exported it.
if ! command -v arm-none-eabi-g++ >/dev/null 2>&1; then
    PATH="$APPDATA/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-windows/usr/bin:$PATH"
    export PATH
fi
command -v arm-none-eabi-g++ >/dev/null 2>&1 || {
    echo "arm-none-eabi-g++ not found; install the PROS toolchain" >&2
    exit 2
}

set -- ${@:-$(find src -name '*.cpp' | sort)}

fail=0
for f in "$@"; do
    if arm-none-eabi-g++ -fsyntax-only \
        -mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard -mthumb \
        -D_POSIX_THREADS -D_UNIX98_THREAD_MUTEX_ATTRIBUTES -D_POSIX_TIMERS \
        -D_POSIX_MONOTONIC_CLOCK -D_PROS_INCLUDE_LIBLVGL_LLEMU_H \
        -D_PROS_INCLUDE_LIBLVGL_LLEMU_HPP \
        -Wno-psabi --std=gnu++26 -Iinclude -Isrc "$f"; then
        echo "  ok  $f"
    else
        echo "FAIL  $f"
        fail=1
    fi
done
exit $fail
