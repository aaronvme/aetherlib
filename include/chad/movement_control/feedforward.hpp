#pragma once

#include <functional>

#include "chad/drivetrain.hpp"

namespace chad {
struct FeedforwardGains {
    float kV{0.0f};
    float kaStraight{0.0f};
    float kaTurn{0.0f};
    float ksStraight{0.0f};
    float ksTurn{0.0f};
    float kP{0.0f};
    float kI{0.0f};

    float integralErrorGate{0.0f};

    float integralClamp{0.0f};
};

struct FeedforwardSample {
    float vTarget{0}, wTarget{0};
    float vLeftTarget{0}, vRightTarget{0};
    float vLeftMeasured{0}, vRightMeasured{0};
    float aLinear{0}, aAngular{0};
    float voltLeft{0}, voltRight{0};
    float dt{0};
};

class FeedforwardController {
public:

    FeedforwardController(const FeedforwardGains& gains, float trackWidth);

    DrivetrainVoltages update(float targetLinearVel, float targetAngularVel,
                              float measuredLeftVel, float measuredRightVel,
                              float dt);

    void reset();

    void setGains(const FeedforwardGains& gains);
    const FeedforwardGains& getGains() const { return mGains; }

    void setTrackWidth(float trackWidth) { mTrackWidth = trackWidth; }
    float getTrackWidth() const { return mTrackWidth; }

    void setMaxVoltage(float volts) { mMaxVoltage = volts; }
    float getMaxVoltage() const { return mMaxVoltage; }

    void setTelemetry(std::function<void(const FeedforwardSample&)> callback);

private:
    std::function<void(const FeedforwardSample&)> mTelemetry;
    FeedforwardGains mGains;
    float mTrackWidth;
    float mMaxVoltage{12.0f};

    float mPrevLinearVel{0.0f};
    float mPrevAngularVel{0.0f};
    float mPrevLeftError{0.0f};
    float mPrevRightError{0.0f};
    float mLeftIntegral{0.0f};
    float mRightIntegral{0.0f};

    bool mFirstUpdate{true};
};
}
