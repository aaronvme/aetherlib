// chad-sim shim for PROS's umbrella header.
// Mirrors include/api.h minus LVGL and the C-only headers the sim does not need.
#pragma once

#include <cerrno>
#include <cmath>
#include <cstdbool>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define PROS_VERSION_MAJOR 4
#define PROS_VERSION_MINOR 2
#define PROS_VERSION_PATCH 0
#define PROS_VERSION_STRING "4.2.0-sim"

#include "pros/error.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
