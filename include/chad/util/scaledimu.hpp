#pragma once

#include <cmath>

#include "pros/imu.hpp"

namespace chad {
class ScaledIMU : public pros::Imu {
public:

    ScaledIMU(std::uint8_t port,
              double expectedRotation = 360.0,
              double actualReading = 355.0)
        : pros::Imu(port),
          mScaleFactor(expectedRotation / actualReading)
    {}

    double scaleFactor() const { return mScaleFactor; }

    void setScaleFactor(double expectedRotation, double actualReading) {
        if (actualReading > 0.0) {
            mScaleFactor = expectedRotation / actualReading;
        }
    }

    double get_rotation() const override {
        double raw = pros::Imu::get_rotation();
        if (std::isinf(raw)) return raw;
        return raw * mScaleFactor;
    }

    double get_heading() const override {
        double scaled = get_rotation();
        if (std::isinf(scaled)) return scaled;

        double heading = std::fmod(scaled, 360.0);
        if (heading < 0.0) heading += 360.0;
        return heading;
    }

private:
    double mScaleFactor;
};
}
