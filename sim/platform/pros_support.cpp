// Out-of-line definitions for the PROS shim.
#include "pros/llemu.hpp"
#include "pros/misc.hpp"

namespace pros {

namespace sim_input {
ControllerInput& state(int id) {
    static ControllerInput controllers[2];
    return controllers[(id == 0) ? 0 : 1];
}

std::uint8_t& competitionStatus() {
    static std::uint8_t status = 0;
    return status;
}
}  // namespace sim_input

namespace lcd {

std::vector<std::string>& screenLines() {
    static std::vector<std::string> lines(8);
    return lines;
}

namespace { bool gInitialized = false; }

bool initialize(void) { gInitialized = true; return true; }
bool is_initialized(void) { return gInitialized; }

bool clear(void) {
    for (auto& l : screenLines()) l.clear();
    return true;
}

bool clear_line(std::int16_t line) {
    if (line >= 0 && line < static_cast<std::int16_t>(screenLines().size())) {
        screenLines()[line].clear();
    }
    return true;
}

bool set_text(std::int16_t line, std::string text) {
    if (line >= 0 && line < static_cast<std::int16_t>(screenLines().size())) {
        screenLines()[line] = std::move(text);
    }
    return true;
}

}  // namespace lcd
}  // namespace pros
