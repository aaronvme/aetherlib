// Ported from FBLib - https://github.com/jiazegao/FBLib
#include "riku/movement_control/velocity_profiles.hpp"

#include <algorithm>
#include <cmath>

#include "riku/util/util.hpp"

namespace riku {

std::vector<ProfilePoint> generateTrapezoidal(float distance, float maxVel,
                                               float maxAccel, float dt) {
    std::vector<ProfilePoint> profile;

    // Without this the triangular branch takes sqrt of a negative, or divides
    // by a zero accel, and NaN propagates into tTotal — the sampling loop then
    // never runs and callers get an empty profile to index into.
    if (distance <= 0.0f || maxVel <= 0.0f || maxAccel <= 0.0f || dt <= 0.0f) {
        profile.push_back({0.0f, 0.0f, 0.0f});
        return profile;
    }

    // Time to accelerate to max velocity
    float tAccel = maxVel / maxAccel;
    // Distance covered during acceleration
    float dAccel = 0.5f * maxAccel * tAccel * tAccel;

    // Check if we even reach max velocity (triangular profile)
    float tCruise, dCruise;
    if (2.0f * dAccel > distance) {
        // Triangular: never reaches maxVel
        tAccel = std::sqrt(distance / maxAccel);
        maxVel = maxAccel * tAccel;
        // dAccel must follow tAccel down. Left at its full-speed value the
        // decel branch below writes positions starting past `distance`, so the
        // profile's position axis jumps and anything sampling it by position
        // reads peak velocity for the whole second half of the move.
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
            // Accelerating
            vel = maxAccel * t;
            pos = 0.5f * maxAccel * t * t;
        } else if (t < tAccel + tCruise) {
            // Cruising
            vel = maxVel;
            pos = dAccel + maxVel * (t - tAccel);
        } else if (t < tTotal) {
            // Decelerating
            float tDecel = t - tAccel - tCruise;
            vel = maxVel - maxAccel * tDecel;
            pos = dAccel + dCruise + maxVel * tDecel - 0.5f * maxAccel * tDecel * tDecel;
        } else {
            // Done
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
    // Without a jerk limit the 7-phase construction degenerates to a trapezoid.
    if (maxJerk <= 0.0f) return generateTrapezoidal(distance, maxVel, maxAccel, dt);

    // 7 phases: jerk up / const accel / jerk down / cruise / jerk down /
    // const decel / jerk up. Phases 1,3,5,7 last tJ; 2,6 last tA; 4 lasts tC.

    // A jerk-only ramp pair already reaches A²/J. If that is over the speed
    // limit there is no room for a constant-accel phase and A itself has to
    // come down, or the profile overshoots maxVel.
    float a = maxAccel;
    if (a * a / maxJerk > maxVel) a = std::sqrt(maxVel * maxJerk);

    // An acceleration half is symmetric, so it spans (v/2)·(2·tJ + tA) — mean
    // speed times duration. Equating 2× that to `distance` inverts in closed
    // form, which is what sets vPeak when the move is too short to cruise.
    float vPeak = maxVel;
    float dJerkOnly = 2.0f * a * a * a / (maxJerk * maxJerk);  // span at v = A²/J
    float dFull = maxVel * (a / maxJerk + maxVel / a);         // span at v = maxVel
    if (distance < dJerkOnly) {
        // Even jerk-only ramps are too long, so a has to shrink with v.
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
    float tC = dCruise / vPeak;  // cruise duration

    // — Phase time boundaries —
    float T1 = tJ;
    float T2 = T1 + tA;
    float T3 = T2 + tJ;           // end of accel
    float T4 = T3 + tC;           // end of cruise
    float T5 = T4 + tJ;           // end of decel jerk-in
    float T6 = T5 + tA;           // end of const decel
    float T7 = T6 + tJ;           // end of decel

    // Phase 6/7 entry states, chained forward from s3 rather than measured back
    // from `distance`. Anchoring phase 7 to `distance` puts a backward step in
    // the position axis, which breaks any lookup keyed on position.
    float v5 = vPeak - vJ;
    float s5 = s3 + dCruise + vPeak * tJ - sJ;
    float v6 = v5 - a * tA;
    float s6 = s5 + v5 * tA - 0.5f * a * tA * tA;

    // ========================================================================
    // Sample the profile at dt intervals
    // ========================================================================

    for (float t = 0.0f; t <= T7 + dt; t += dt) {
        float pos_t, vel_t;

        if (t <= 0.0f) {
            vel_t = 0.0f; pos_t = 0.0f;
        } else if (t < T1) {
            // Phase 1: jerk up — a(τ) = +J·τ
            float tau = t;
            vel_t = 0.5f * maxJerk * tau * tau;
            pos_t = maxJerk * tau * tau * tau / 6.0f;
        } else if (t < T2) {
            // Phase 2: constant +accel — a(τ) = +A
            float tau = t - T1;
            vel_t = v1 + a * tau;
            pos_t = s1 + v1 * tau + 0.5f * a * tau * tau;
        } else if (t < T3) {
            // Phase 3: jerk down — a(τ) = +A − J·τ
            float tau = t - T2;
            vel_t = v2 + a * tau - 0.5f * maxJerk * tau * tau;
            pos_t = s2 + v2 * tau + 0.5f * a * tau * tau
                  - maxJerk * tau * tau * tau / 6.0f;
        } else if (t < T4) {
            // Phase 4: cruise — a = 0, v = vPeak
            float tau = t - T3;
            vel_t = vPeak;
            pos_t = s3 + vPeak * tau;
        } else if (t < T5) {
            // Phase 5: jerk into decel — a(τ) = −J·τ
            float tau = t - T4;
            vel_t = vPeak - 0.5f * maxJerk * tau * tau;
            pos_t = s3 + dCruise + vPeak * tau
                  - maxJerk * tau * tau * tau / 6.0f;
        } else if (t < T6) {
            // Phase 6: constant −accel — a(τ) = −A
            float tau = t - T5;
            vel_t = v5 - a * tau;
            pos_t = s5 + v5 * tau - 0.5f * a * tau * tau;
        } else if (t < T7) {
            // Phase 7: jerk to zero — a(τ) = −A + J·τ
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

    // The sampling loop can stop a hair short of the end, so land the profile
    // exactly at rest at `distance` — consumers read the final point directly.
    if (profile.empty()) {
        profile.push_back({0.0f, 0.0f, distance});
    } else if (profile.back().position < distance - 1e-4f ||
               std::fabs(profile.back().velocity) > 1e-4f) {
        profile.push_back({profile.back().time + dt, 0.0f, distance});
    }

    return profile;
}

}  // namespace riku
