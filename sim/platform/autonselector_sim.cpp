// riku-sim replacement for src/riku/auton_selector/autonselector.cpp.
//
// The real implementation draws an LVGL list on the brain screen. The sim
// keeps the same class contract but no UI: routine selection comes from the
// browser via the bridge. Registration order and runSelected() semantics match
// the real selector, so autons registered by gamespecific code appear in the
// dropdown in the same order they would on the brain.
#include "riku/auton_selector/autonselector.hpp"

namespace riku {

void AutonSelector::registerAuton(const std::string& name, AutonFunc func) {
    mAutons.push_back({name, std::move(func), nullptr});
}

void AutonSelector::init() { mInitialised = true; }

void AutonSelector::select(int index) {
    if (index < 0 || static_cast<std::size_t>(index) >= mAutons.size()) return;
    mSelectedIndex = index;
}

std::string AutonSelector::selectedName() const {
    if (mAutons.empty()) return "none";
    if (mSelectedIndex < 0 || static_cast<std::size_t>(mSelectedIndex) >= mAutons.size()) {
        return "none";
    }
    return mAutons[mSelectedIndex].name;
}

void AutonSelector::runSelected() {
    if (mAutons.empty()) return;
    if (mSelectedIndex < 0 || static_cast<std::size_t>(mSelectedIndex) >= mAutons.size()) return;
    if (mAutons[mSelectedIndex].func) mAutons[mSelectedIndex].func();
}

void AutonSelector::buttonCb(lv_event_t*) {}
void AutonSelector::refreshHighlight() {}

}  // namespace riku
