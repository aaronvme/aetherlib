#include "chad/movement_control/velocity_profiles.hpp"

#include <algorithm>
#include <cmath>

#include "chad/util/util.hpp"

namespace chad {
std::vector<ProfilePoint> generateTrapezoidal(float distance, float maxVel,
                                               float maxAccel, float dt) {
    std::vector<ProfilePoint> profile;

    if (distance <= 0.0f || maxVel <= 0.0f || maxAccel <= 0.0f || dt <= 0.0f) {
        profile.push_back({0.0f, 0.0f, 0.0f});
        return profile;
    }

    float tAccel = maxVel / maxAccel;

    float dAccel = 0.5f * maxAccel * tAccel * tAccel;

    float tCruise, dCruise;
    if (2.0f * dAccel > distance) {
        tAccel = std::sqrt(distance / maxAccel);
        maxVel = maxAccel * tAccel;

        dAccel = 0.5f * maxAccel * tAccel * tAccel;
        tCruise = 0.0f;
        dCruise = 0.0f;
    } else {
        dCruise = distance - 2.0f * dAccel;
        tCruise = dCruise / maxVel;
    }

    float tTotal = 2.0f * tAccel + tCruise;
    float pos = 0.0f;

    for (float t = 0.0f; t <= tTotal + dt; t += dt) {
        float vel;

        if (t < tAccel) {
            vel = maxAccel * t;
            pos = 0.5f * maxAccel * t * t;
        } else if (t < tAccel + tCruise) {
            vel = maxVel;
            pos = dAccel + maxVel * (t - tAccel);
        } else if (t < tTotal) {
            float tDecel = t - tAccel - tCruise;
            vel = maxVel - maxAccel * tDecel;
            pos = dAccel + dCruise + maxVel * tDecel - 0.5f * maxAccel * tDecel * tDecel;
        } else {
            vel = 0.0f;
            pos = distance;
        }

        profile.push_back({t, vel, pos});
    }

    return profile;
}

std::vector<ProfilePoint> generateSCurve(float distance, float maxVel,
                                          float maxAccel, float maxJerk,
                                          float dt) {
    std::vector<ProfilePoint> profile;

    if (distance <= 0.0f || maxVel <= 0.0f || maxAccel <= 0.0f || dt <= 0.0f) {
        profile.push_back({0.0f, 0.0f, 0.0f});
        return profile;
    }

    if (maxJerk <= 0.0f) return generateTrapezoidal(distance, maxVel, maxAccel, dt);

    float a = maxAccel;
    if (a * a / maxJerk > maxVel) a = std::sqrt(maxVel * maxJerk);

    float vPeak = maxVel;
    float dJerkOnly = 2.0f * a * a * a / (maxJerk * maxJerk);
    float dFull = maxVel * (a / maxJerk + maxVel / a);
    if (distance < dJerkOnly) {
        vPeak = std::pow(0.5f * distance * std::sqrt(maxJerk), 2.0f / 3.0f);
        a = std::sqrt(vPeak * maxJerk);
    } else if (distance < dFull) {
        float aj = a / maxJerk;
        vPeak = 0.5f * a * (std::sqrt(aj * aj + 4.0f * distance / a) - aj);
    }
    if (vPeak < 1e-6f) vPeak = 1e-6f;

    float tJ = a / maxJerk;
    float vJ = 0.5f * a * a / maxJerk;
    float sJ = a * a * a / (6.0f * maxJerk * maxJerk);
    float tA = std::max(0.0f, (vPeak - 2.0f * vJ) / a);

    float v1 = vJ;
    float s1 = sJ;
    float v2 = v1 + a * tA;
    float s2 = s1 + v1 * tA + 0.5f * a * tA * tA;
    float s3 = s2 + v2 * tJ + 0.5f * a * tJ * tJ - sJ;

    float dCruise = std::max(0.0f, distance - 2.0f * s3);
    float tC = dCruise / vPeak;

    float T1 = tJ;
    float T2 = T1 + tA;
    float T3 = T2 + tJ;
    float T4 = T3 + tC;
    float T5 = T4 + tJ;
    float T6 = T5 + tA;
    float T7 = T6 + tJ;

    float v5 = vPeak - vJ;
    float s5 = s3 + dCruise + vPeak * tJ - sJ;
    float v6 = v5 - a * tA;
    float s6 = s5 + v5 * tA - 0.5f * a * tA * tA;

    for (float t = 0.0f; t <= T7 + dt; t += dt) {
        float pos_t, vel_t;

        if (t <= 0.0f) {
            vel_t = 0.0f; pos_t = 0.0f;
        } else if (t < T1) {
            float tau = t;
            vel_t = 0.5f * maxJerk * tau * tau;
            pos_t = maxJerk * tau * tau * tau / 6.0f;
        } else if (t < T2) {
            float tau = t - T1;
            vel_t = v1 + a * tau;
            pos_t = s1 + v1 * tau + 0.5f * a * tau * tau;
        } else if (t < T3) {
            float tau = t - T2;
            vel_t = v2 + a * tau - 0.5f * maxJerk * tau * tau;
            pos_t = s2 + v2 * tau + 0.5f * a * tau * tau
                  - maxJerk * tau * tau * tau / 6.0f;
        } else if (t < T4) {
            float tau = t - T3;
            vel_t = vPeak;
            pos_t = s3 + vPeak * tau;
        } else if (t < T5) {
            float tau = t - T4;
            vel_t = vPeak - 0.5f * maxJerk * tau * tau;
            pos_t = s3 + dCruise + vPeak * tau
                  - maxJerk * tau * tau * tau / 6.0f;
        } else if (t < T6) {
            float tau = t - T5;
            vel_t = v5 - a * tau;
            pos_t = s5 + v5 * tau - 0.5f * a * tau * tau;
        } else if (t < T7) {
            float tau = t - T6;
            vel_t = v6 - a * tau + 0.5f * maxJerk * tau * tau;
            pos_t = s6 + v6 * tau - 0.5f * a * tau * tau
                  + maxJerk * tau * tau * tau / 6.0f;
        } else {
            vel_t = 0.0f;
            pos_t = distance;
        }

        pos_t = clamp(pos_t, 0.0f, distance);
        profile.push_back({t, vel_t, pos_t});

        if (pos_t >= distance - 0.001f && std::fabs(vel_t) < 0.01f) break;
    }

    if (profile.empty()) {
        profile.push_back({0.0f, 0.0f, distance});
    } else if (profile.back().position < distance - 1e-4f ||
               std::fabs(profile.back().velocity) > 1e-4f) {
        profile.push_back({profile.back().time + dt, 0.0f, distance});
    }

    return profile;
}
}
