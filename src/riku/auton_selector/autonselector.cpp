#include "riku/auton_selector/autonselector.hpp"

#include "liblvgl/lvgl.h"

namespace riku {

namespace {
constexpr uint32_t kBgColor = 0x1A1A2E;
constexpr uint32_t kTextColor = 0xE0E0E0;
constexpr uint32_t kSelectedColor = 0x2E7D32;
constexpr uint32_t kIdleColor = 0x30304A;
}  // namespace

void AutonSelector::registerAuton(const std::string& name, AutonFunc func) {
    if (mInitialised) return;
    mAutons.push_back({name, std::move(func), nullptr});
}

void AutonSelector::init() {
    if (mInitialised) return;
    mInitialised = true;

    mScreen = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(mScreen, lv_color_hex(kBgColor), 0);
    lv_screen_load(mScreen);

    lv_obj_t* title = lv_label_create(mScreen);
    lv_label_set_text(title, "riku - Auton Selector");
    lv_obj_set_style_text_color(title, lv_color_hex(kTextColor), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 6);

    mStatus = lv_label_create(mScreen);
    lv_obj_set_style_text_color(mStatus, lv_color_hex(kTextColor), 0);
    lv_obj_align(mStatus, LV_ALIGN_BOTTOM_MID, 0, -6);

    mList = lv_list_create(mScreen);
    lv_obj_set_size(mList, 440, 170);
    lv_obj_align(mList, LV_ALIGN_CENTER, 0, 4);
    lv_obj_set_style_bg_color(mList, lv_color_hex(kBgColor), 0);

    for (std::size_t i = 0; i < mAutons.size(); ++i) {
        lv_obj_t* btn = lv_list_add_button(mList, nullptr, mAutons[i].name.c_str());
        lv_obj_set_style_text_color(btn, lv_color_hex(kTextColor), 0);
        // user_data is `this`, not the index: the callback resolves which entry
        // was hit by button pointer, so it stays correct if mAutons reallocates.
        lv_obj_add_event_cb(btn, buttonCb, LV_EVENT_CLICKED, this);
        mAutons[i].button = btn;
    }

    refreshHighlight();
}

void AutonSelector::buttonCb(lv_event_t* e) {
    auto* self = static_cast<AutonSelector*>(lv_event_get_user_data(e));
    lv_obj_t* target = lv_event_get_target_obj(e);
    if (self == nullptr || target == nullptr) return;

    for (std::size_t i = 0; i < self->mAutons.size(); ++i) {
        if (self->mAutons[i].button == target) {
            self->select(static_cast<int>(i));
            return;
        }
    }
}

void AutonSelector::select(int index) {
    if (mAutons.empty()) return;
    if (index < 0 || index >= static_cast<int>(mAutons.size())) return;
    mSelectedIndex = index;
    refreshHighlight();
}

void AutonSelector::refreshHighlight() {
    for (std::size_t i = 0; i < mAutons.size(); ++i) {
        if (mAutons[i].button == nullptr) continue;
        const bool selected = (static_cast<int>(i) == mSelectedIndex);
        lv_obj_set_style_bg_color(mAutons[i].button,
                                  lv_color_hex(selected ? kSelectedColor : kIdleColor), 0);
    }

    if (mStatus != nullptr) {
        lv_label_set_text(mStatus,
                          mAutons.empty() ? "no autons registered"
                                          : mAutons[mSelectedIndex].name.c_str());
    }
}

std::string AutonSelector::selectedName() const {
    if (mAutons.empty()) return {};
    return mAutons[mSelectedIndex].name;
}

void AutonSelector::runSelected() {
    if (mAutons.empty()) return;
    const AutonFunc& func = mAutons[mSelectedIndex].func;
    if (func) func();
}

}  // namespace riku
