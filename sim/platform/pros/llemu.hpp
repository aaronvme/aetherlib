// riku-sim PROS shim — pros/llemu.hpp
//
// Brain-screen text is captured into a ring buffer the browser renders as a
// fake V5 screen, so lcd::print debugging from library code stays visible.
#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace pros {
namespace lcd {

/// Captured screen lines (8 rows, as on a V5 brain).
std::vector<std::string>& screenLines();

bool initialize(void);
bool is_initialized(void);
bool clear(void);
bool clear_line(std::int16_t line);
bool set_text(std::int16_t line, std::string text);

template <typename... Params>
bool print(std::int16_t line, const char* fmt, Params... args) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), fmt, args...);
    return set_text(line, std::string(buf));
}

inline bool print(std::int16_t line, const char* fmt) {
    return set_text(line, std::string(fmt));
}

using lcd_btn_cb_fn_t = void (*)(void);
inline void register_btn0_cb(lcd_btn_cb_fn_t) {}
inline void register_btn1_cb(lcd_btn_cb_fn_t) {}
inline void register_btn2_cb(lcd_btn_cb_fn_t) {}
inline std::uint8_t read_buttons(void) { return 0; }

}  // namespace lcd
}  // namespace pros
