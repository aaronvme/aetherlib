// chad-sim shim for <pros/misc.h>.
//
// PROS ships the same declarations under both misc.h and misc.hpp, and a
// routine may include either. Without this file the .h form fell through to
// include/pros/misc.h, which redeclares every controller enum the shim's .hpp
// already defined -- so one `#include "pros/misc.h"` anywhere in a routine took
// the whole sim build down with twenty redefinition errors.
#pragma once

#include "misc.hpp"
