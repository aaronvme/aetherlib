#pragma once

#include <functional>
#include <string>
#include <vector>

#include "gamespecific/config.hpp"
#include "chad/util/characterize.hpp"
#include "chad/util/localization_fit.hpp"

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

constexpr bool kFieldIsClear = true;

struct BeamRead {
    bool valid{false};
    float inches{0.0f};
    float sigmaIn{0.0f};
    float bx{0.0f}, by{0.0f};
    float alongX{0.0f}, alongY{0.0f};
};

struct BeamPair {
    int plus{-1}, minus{-1};
    bool xAxis{false};
};

int readBeams(BeamRead* out, float robotThetaRad);

int findBeamPairs(const BeamRead* beams, BeamPair* out);

bool solveWallTruth(const BeamRead* beams, const BeamPair* pairs, int nPairs,
                    float kSigmas, float& cx, float& cy,
                    float& worstBracketIn);

bool solveWallTruthClear(const BeamRead* beams, float& cx, float& cy);

bool solveTruth(const BeamRead* beams, const BeamPair* pairs, int nPairs,
                float kSigmas, float& cx, float& cy, float& worstBracketIn);
}
