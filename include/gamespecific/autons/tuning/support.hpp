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

    // Start the progress page over. Ignored once results are showing.
    void clear();

private:
    static constexpr int kProgressLines = 24;
    static constexpr int kResultLines = 11;
    std::vector<std::string> mPage;
    int mLine{1};
};

// The controller screen is never cleared, so a write shorter than what was
// there leaves the old tail showing. Everything goes through here, which pads
// and truncates to the full line.
constexpr int kCtlWidth = 18;
void ctlLine(int line, const char* fmt, ...);

void setFilters(bool on);

bool headingOk(Report& out);

constexpr float kTiltLimitDeg = 5.0f;
std::function<float()> tiltProbe();

void waitForA(Report& out, const char* why);

// Like waitForA, but for the gate before a routine moves. It prints as
// progress, not as a result: the first result() call freezes the screen into
// the results page and everything after it stops showing.
void waitForStart(Report& out, const char* what);

bool solvePark(const tuning::RangeNoiseFit& beams, float theta,
               float& cx, float& cy, int& pinned,
               bool* mapDisagrees = nullptr);

tuning::Characterizer makeCharacterizer(Report& out);

constexpr bool kFieldIsClear = false;

using BeamRead = tuning::MapBeam;

constexpr int kMaxFieldCircles = 16;

// Where waitForStart draws the live beam readout, below the progress lines.
constexpr int kBeamRow = 14;

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

bool solveWallTruthMapped(const BeamRead* beams, float& cx, float& cy,
                          int& inliers);

// An opposing pair brackets a blockage outright, so it reports as this rather
// than an inlier count.
constexpr int kBracketedFix = -1;

bool solveTruth(const BeamRead* beams, const BeamPair* pairs, int nPairs,
                float kSigmas, float& cx, float& cy, float& worstBracketIn,
                int* inliers = nullptr);

inline bool fixVerified(int inliers) {
    return inliers == kBracketedFix || inliers >= tuning::MAP_VERIFIED_INLIERS;
}
}
