// Ported from FBLib - https://github.com/jiazegao/FBLib
#pragma once

#include "chad/util/util.hpp"

namespace chad {

enum class TurnDirection {
    CCW,         // counter-clockwise always
    CW,          // clockwise always
    SHORTEST     // shortest angular path (default)
};

enum class SwingSide {
    Left,   // left motors stationary, right motors turn
    Right   // right motors stationary, left motors turn
};

// ============================================================================
// TurnToPoint utility functions
// ============================================================================

/// Resolve signed angular error given a desired turn direction
float resolveTurnError(float rawErrorRad, TurnDirection direction);

/// Heading error to face a target point from current pose
float headingToPoint(const Pose& current, float targetX, float targetY);

/// Heading error with an angular offset
/// @param offsetDeg  angular offset in VEX degrees, so positive is CLOCKWISE
float headingToPointOffset(const Pose& current, float targetX, float targetY,
                            float offsetDeg);

/// Check if heading error is within tolerance (both in radians, internal use)
bool headingWithinTolerance(float headingErrorRad, float toleranceRad);

// ============================================================================
// Parameter structs
// ============================================================================

struct TurntoHeadingParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{-1.0f};  // VEX degrees, negative = ChassisConfig
    /// Run the whole timeout window: no arrival, no stall. What a step-response
    /// capture needs (util/autotune.hpp). Ignored without a timeout.
    bool holdForTimeout{false};
};

struct TurnToPointParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{-1.0f};  // VEX degrees, negative = ChassisConfig
    float offset{0.0f};            // VEX degrees, so positive is clockwise
};

struct SwingParams {
    TurnDirection direction{TurnDirection::SHORTEST};
    float maxSpeed{127.0f};
    float minSpeed{0.0f};
    float targetTolerance{-1.0f};  // VEX degrees, negative = ChassisConfig
};

}  // namespace chad
