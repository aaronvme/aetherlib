#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

// Opaque LVGL types, so including this header does not pull in all of LVGL.
typedef struct _lv_obj_t lv_obj_t;
typedef struct _lv_event_t lv_event_t;

namespace AARONLIB {

using AutonFunc = std::function<void()>;

// Touch selector for autonomous routines, drawn with LVGL 9 on the V5 screen.
class AutonSelector {
public:
    /// Register before calling init(). Routines added afterwards are not drawn.
    void registerAuton(const std::string& name, AutonFunc func);

    /// Build and show the selector screen. Does nothing on a second call.
    void init();

    void select(int index);
    int selectedIndex() const { return mSelectedIndex; }

    /// Empty if nothing is registered.
    std::string selectedName() const;

    std::size_t count() const { return mAutons.size(); }

    /// Invoke the selected routine. No-op if nothing is registered.
    void runSelected();

private:
    struct Entry {
        std::string name;
        AutonFunc func;
        lv_obj_t* button{nullptr};
    };

    static void buttonCb(lv_event_t* e);
    void refreshHighlight();

    std::vector<Entry> mAutons;
    int mSelectedIndex{0};
    bool mInitialised{false};

    lv_obj_t* mScreen{nullptr};
    lv_obj_t* mList{nullptr};
    lv_obj_t* mStatus{nullptr};
};

}  // namespace AARONLIB
