#pragma once

#include <cmath>

#include "chad/util/util.hpp"

namespace chad::tuning {
struct LiftHoldSample {
    float motorDeg{0.0f};
    float holdPower{0.0f};
};

struct LiftSweepBin {
    float motorDeg{0.0f};
    float sumUp{0.0f};
    float sumDown{0.0f};
    int nUp{0};
    int nDown{0};
};

struct LiftGravityFit {
    float kG{0.0f};
    float horizontalMotorDeg{0.0f};
    float rms{0.0f};
    float spanArmDeg{0.0f};
    int samples{0};
    bool valid{false};
};

inline int combineLiftSweeps(const LiftSweepBin* bins, int n,
                             LiftHoldSample* out, float* frictionUnits) {
    if (frictionUnits != nullptr) *frictionUnits = 0.0f;
    if (bins == nullptr || out == nullptr || n <= 0) return 0;

    int m = 0;
    float frictionSum = 0.0f;
    for (int i = 0; i < n; ++i) {
        const LiftSweepBin& b = bins[i];
        if (b.nUp <= 0 || b.nDown <= 0) continue;
        const float up = b.sumUp / static_cast<float>(b.nUp);
        const float down = b.sumDown / static_cast<float>(b.nDown);
        if (!std::isfinite(up) || !std::isfinite(down)) continue;
        out[m].motorDeg = b.motorDeg;
        out[m].holdPower = 0.5f * (up + down);
        frictionSum += 0.5f * (up - down);
        ++m;
    }
    if (frictionUnits != nullptr && m > 0) {
        *frictionUnits = frictionSum / static_cast<float>(m);
    }
    return m;
}

struct LiftDirectionalSplit {
    float frictionUnits{0.0f};
    float emfPerRpm{0.0f};
    bool valid{false};
};

inline LiftDirectionalSplit splitFrictionAndEmf(float gap1, float rpm1,
                                                float gap2, float rpm2) {
    LiftDirectionalSplit out;
    if (!std::isfinite(gap1) || !std::isfinite(gap2) ||
        !std::isfinite(rpm1) || !std::isfinite(rpm2)) {
        return out;
    }
    const float dRpm = rpm2 - rpm1;

    if (dRpm > -1e-3f && dRpm < 1e-3f) return out;

    out.emfPerRpm = (gap2 - gap1) / dRpm;
    out.frictionUnits = gap1 - out.emfPerRpm * rpm1;

    out.valid = out.emfPerRpm > 0.0f;
    return out;
}

struct LiftLinearFit {
    float base{0.0f};
    float perMotorDeg{0.0f};
    float meanPower{0.0f};
    float rms{0.0f};
    float rmsConstant{0.0f};
    int samples{0};
    bool valid{false};
};

inline LiftLinearFit fitLiftConstant(const LiftHoldSample* s, int n) {
    LiftLinearFit f;
    if (s == nullptr || n < 2) return f;

    float sx = 0.0f, sxx = 0.0f, sp = 0.0f, sxp = 0.0f;
    int used = 0;
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(s[i].motorDeg) || !std::isfinite(s[i].holdPower)) {
            continue;
        }
        sx += s[i].motorDeg;
        sxx += s[i].motorDeg * s[i].motorDeg;
        sp += s[i].holdPower;
        sxp += s[i].motorDeg * s[i].holdPower;
        ++used;
    }
    if (used < 2) return f;

    f.samples = used;
    const float fn = static_cast<float>(used);
    f.meanPower = sp / fn;

    const float det = fn * sxx - sx * sx;
    if (std::isfinite(det) && std::fabs(det) > 1e-6f) {
        f.base = (sp * sxx - sxp * sx) / det;
        f.perMotorDeg = (fn * sxp - sx * sp) / det;
    } else {
        f.base = f.meanPower;
        f.perMotorDeg = 0.0f;
    }

    float sse = 0.0f, sseFlat = 0.0f;
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(s[i].motorDeg) || !std::isfinite(s[i].holdPower)) {
            continue;
        }
        const float r = s[i].holdPower -
                        (f.base + f.perMotorDeg * s[i].motorDeg);
        sse += r * r;
        const float rf = s[i].holdPower - f.meanPower;
        sseFlat += rf * rf;
    }
    f.rms = std::sqrt(sse / fn);
    f.rmsConstant = std::sqrt(sseFlat / fn);
    f.valid = std::isfinite(f.base) && std::isfinite(f.perMotorDeg);
    return f;
}

inline LiftGravityFit fitLiftGravity(const LiftHoldSample* s, int n,
                                     float motorDegPerArmDeg,
                                     float minSpanArmDeg = 25.0f) {
    LiftGravityFit f;
    if (s == nullptr || n < 3 || !std::isfinite(motorDegPerArmDeg) ||
        std::fabs(motorDegPerArmDeg) < 1e-6f) {
        return f;
    }

    float scc = 0.0f, scs = 0.0f, sss = 0.0f, spc = 0.0f, sps = 0.0f;
    float lo = 0.0f, hi = 0.0f, sumMotorDeg = 0.0f;
    int used = 0;

    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(s[i].motorDeg) || !std::isfinite(s[i].holdPower)) {
            continue;
        }
        const float phi = s[i].motorDeg / motorDegPerArmDeg * DEG_TO_RAD;
        if (used == 0 || phi < lo) lo = phi;
        if (used == 0 || phi > hi) hi = phi;
        const float c = std::cos(phi), sn = std::sin(phi);
        scc += c * c;
        scs += c * sn;
        sss += sn * sn;
        spc += s[i].holdPower * c;
        sps += s[i].holdPower * sn;
        sumMotorDeg += s[i].motorDeg;
        ++used;
    }
    if (used < 3) return f;

    f.samples = used;
    f.spanArmDeg = (hi - lo) * RAD_TO_DEG;

    const float det = scc * sss - scs * scs;
    if (!std::isfinite(det) || std::fabs(det) < 1e-9f) return f;

    const float a = (spc * sss - sps * scs) / det;
    const float b = (sps * scc - spc * scs) / det;
    if (!std::isfinite(a) || !std::isfinite(b)) return f;

    f.kG = std::sqrt(a * a + b * b);

    float horizontal = std::atan2(b, a) * RAD_TO_DEG * motorDegPerArmDeg;
    const float period = 360.0f * motorDegPerArmDeg;
    if (std::fabs(period) > 1e-6f) {
        const float mean = sumMotorDeg / static_cast<float>(used);
        horizontal += std::round((mean - horizontal) / period) * period;
    }
    f.horizontalMotorDeg = horizontal;

    float sse = 0.0f;
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(s[i].motorDeg) || !std::isfinite(s[i].holdPower)) {
            continue;
        }
        const float phi = s[i].motorDeg / motorDegPerArmDeg * DEG_TO_RAD;
        const float r = s[i].holdPower - (a * std::cos(phi) + b * std::sin(phi));
        sse += r * r;
    }
    f.rms = std::sqrt(sse / static_cast<float>(used));

    f.valid = f.spanArmDeg >= minSpanArmDeg && f.kG > 0.0f;
    return f;
}
}
