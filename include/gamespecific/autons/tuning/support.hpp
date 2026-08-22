#pragma once

#include <functional>
#include <string>
#include <vector>

#include "gamespecific/config.hpp"
#include "chad/util/characterize.hpp"

namespace chad::autons {

class Report {
public:
    Report();
    ~Report();

    Report(const Report&) = delete;
    Report& operator=(const Report&) = delete;

    void operator()(const char* fmt, ...);

    void result(const char* fmt, ...);

private:
    static constexpr int kProgressLines = 24;
    static constexpr int kResultLines = 11;
    std::vector<std::string> mPage;
    int mLine{1};
};

void setFilters(bool on);

bool headingOk(Report& out);

constexpr float kTiltLimitDeg = 5.0f;
std::function<float()> tiltProbe();

void waitForA(Report& out, const char* why);

bool solvePark(const tuning::RangeNoiseFit& beams, float theta,
               float& cx, float& cy, int& pinned);

tuning::Characterizer makeCharacterizer(Report& out);

}  // namespace chad::autons
