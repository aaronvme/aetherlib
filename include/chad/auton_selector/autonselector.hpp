#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

typedef struct _lv_obj_t lv_obj_t;
typedef struct _lv_event_t lv_event_t;

namespace chad {
using AutonFunc = std::function<void()>;

class AutonSelector {
public:

    void registerAuton(const std::string& name, AutonFunc func);

    void init();

    void select(int index);
    int selectedIndex() const { return mSelectedIndex; }
    std::string selectedName() const;
    std::size_t count() const { return mAutons.size(); }

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
}
