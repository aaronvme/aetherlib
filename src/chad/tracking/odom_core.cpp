#include "chad/tracking/odom_core.hpp"

#include <cmath>

namespace chad {

DistanceAverage averageLiveDistance(const float* values, const bool* faulted,
                                    int count) {
    DistanceAverage out;
    if (values == nullptr || count <= 0) return out;

    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        const bool bad = (faulted != nullptr && faulted[i]) || !std::isfinite(values[i]);
        if (bad) {
            out.faulted++;
            continue;
        }
        sum += values[i];
        out.live++;
    }
    if (out.live > 0) out.value = sum / static_cast<float>(out.live);
    return out;
}

void OdomCore::rebaseline(float vertIn, float horizIn, float headingRad) {
    mPrevVertDist = std::isfinite(vertIn) ? vertIn : mPrevVertDist;
    mPrevHorizDist = std::isfinite(horizIn) ? horizIn : mPrevHorizDist;
    mPrevHeadingRad = std::isfinite(headingRad) ? wrapRad(headingRad) : mPrevHeadingRad;
}

void OdomCore::clearDeltas() {
    mLastDelta = {};
    for (auto& acc : mAccumDelta) acc = {};
}

OdomDelta OdomCore::consumeDelta(int channel) {
    if (channel < 0 || channel >= DELTA_CHANNELS) return {};
    const OdomDelta out = mAccumDelta[channel];
    mAccumDelta[channel] = {};
    return out;
}

OdomDelta OdomCore::step(float vertIn, float horizIn, float headingRad,
                         bool headingValid) {
    if (!std::isfinite(vertIn)) vertIn = mPrevVertDist;
    if (!std::isfinite(horizIn)) horizIn = mPrevHorizDist;
    if (!headingValid || !std::isfinite(headingRad)) {
        headingRad = mPrevHeadingRad;
        headingValid = false;
        mHeadingHolds++;
    }
    headingRad = wrapRad(headingRad);

    float dVert = vertIn - mPrevVertDist;
    float dHoriz = horizIn - mPrevHorizDist;
    float dTheta = wrapRad(headingRad - mPrevHeadingRad);

    // A jump past the guard is a dropout, not motion. Drop the tick but still
    // take the new baseline: holding the old one would repeat the rejection for
    // ever and freeze odometry outright.
    if (mConfig.maxDeltaIn > 0.0f && (std::fabs(dVert) > mConfig.maxDeltaIn ||
                                      std::fabs(dHoriz) > mConfig.maxDeltaIn)) {
        dVert = 0.0f;
        dHoriz = 0.0f;
        mJumpRejects++;
    }
    if (mConfig.maxDeltaRad > 0.0f && std::fabs(dTheta) > mConfig.maxDeltaRad) {
        dTheta = 0.0f;
        mHeadingJumpRejects++;
    }

    // Integrate the tick's robot-frame motion along the arc it actually traced.
    // The midpoint-heading form this replaces is only second order, and its
    // error accumulates one way around a turn -- always short on the inside --
    // which is exactly the drift a heading reset cannot recover.
    const float startTheta = mConfig.headingFromImu ? mPrevHeadingRad : mPose.theta;
    const Pose integrated =
        Pose{mPose.x, mPose.y, startTheta}.exp({dVert, dHoriz, dTheta});

    mPose.x = integrated.x;
    mPose.y = integrated.y;
    mPose.theta = mConfig.headingFromImu ? headingRad : wrapRad(mPose.theta + dTheta);

    mPrevVertDist = vertIn;
    mPrevHorizDist = horizIn;
    mPrevHeadingRad = headingRad;

    mLastDelta = {dVert, dHoriz, dTheta};
    for (auto& acc : mAccumDelta) {
        acc.dVert += dVert;
        acc.dHoriz += dHoriz;
        acc.dTheta += dTheta;
    }
    return mLastDelta;
}

}  // namespace chad
