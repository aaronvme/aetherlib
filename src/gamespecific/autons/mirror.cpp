#include "gamespecific/autons.hpp"

namespace chad {
void runMirrored(void (*routine)(), FieldMirror mirror) {
    if (routine == nullptr) return;

    // Restore what was set, not None: a routine reached through one mirror
    // may run another, and clearing outright would drop the outer one for
    // everything after the inner call returns.
    struct Restore {
        FieldMirror prev;
        ~Restore() { chassis.setMirror(prev); }
    } restore{chassis.mirror()};

    chassis.setMirror(mirror);
    routine();
}
}
