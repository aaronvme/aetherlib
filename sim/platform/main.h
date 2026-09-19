// chad-sim shim for the project's main.h.
//
// Mirrors include/main.h so gamespecific/ sources compile unchanged, but
// without the LVGL pull-in. The competition entry points are declared the same
// way; the bridge calls them directly instead of the field control system.
#pragma once

#include "api.h"

#ifdef __cplusplus
extern "C" {
#endif

void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include <iostream>
#include <string>
#include <vector>
#endif
