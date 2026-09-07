#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace chad {
struct StepWeights {
    float overshoot{1.0f};
    float settle{8.0f};
    float steadyState{6.0f};
    float oscillation{4.0f};
};

struct StepMetrics {
    float initial{0};
    float overshootPct{0};
    float steadyState{0};
    float settleMs{0};
    int oscillations{0};
    float cost{0};
    bool valid{false};
};

inline StepMetrics analyseStep(const std::vector<float>& t,
                               const std::vector<float>& error,
                               float step, const StepWeights& w) {
    StepMetrics m;
    if (error.size() < 4 || t.size() != error.size() || step <= 0.0f) return m;

    const float initial = error.front();
    m.initial = initial;

    if (std::fabs(initial) < 0.5f * step) return m;
    const float sign = (initial >= 0.0f) ? 1.0f : -1.0f;

    float worstPast = 0.0f;
    bool crossed = false;
    int oscillations = 0;
    float prev = initial;

    for (float e : error) {
        const float past = -sign * e;
        if (past > worstPast) worstPast = past;
        if (!crossed && sign * e <= 0.0f) crossed = true;
        else if (crossed && ((e < 0.0f) != (prev < 0.0f))) oscillations++;
        prev = e;
    }

    const float tol = step * 0.02f;
    float settle = 0.0f;
    for (std::size_t i = error.size(); i-- > 0;) {
        if (std::fabs(error[i]) > tol) {
            settle = (i + 1 < t.size()) ? t[i + 1] : t[i];
            break;
        }
    }

    m.overshootPct = (worstPast > 0.0f) ? (worstPast / step) * 100.0f : 0.0f;
    m.steadyState = std::fabs(error.back());
    m.settleMs = settle;
    m.oscillations = oscillations;
    m.valid = true;
    m.cost = w.overshoot * m.overshootPct +
             w.settle * (m.settleMs / 1000.0f) +
             w.steadyState * m.steadyState +
             w.oscillation * static_cast<float>(m.oscillations);
    return m;
}
}
