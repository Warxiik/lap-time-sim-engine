#include <gtest/gtest.h>

#include "core/math.hpp"

// The torque curve the longitudinal tests use: 400 N m at 5000 rpm, 450 at 10000, 380 at 15000.
static const std::vector<double> kRpm = {5000.0, 10000.0, 15000.0};
static const std::vector<double> kTorque = {400.0, 450.0, 380.0};

TEST(Math, InterpolatesBetweenPoints) {
    EXPECT_DOUBLE_EQ(math::interpolate(kRpm, kTorque, 7500.0), 425.0);
    EXPECT_DOUBLE_EQ(math::interpolate(kRpm, kTorque, 12500.0), 415.0);
}

TEST(Math, IsExactAtThePoints) {
    EXPECT_DOUBLE_EQ(math::interpolate(kRpm, kTorque, 5000.0), 400.0);
    EXPECT_DOUBLE_EQ(math::interpolate(kRpm, kTorque, 10000.0), 450.0);
    EXPECT_DOUBLE_EQ(math::interpolate(kRpm, kTorque, 15000.0), 380.0);
}

TEST(Math, HoldsTheEndsOutsideTheTable) {
    EXPECT_DOUBLE_EQ(math::interpolate(kRpm, kTorque, 1000.0), 400.0);
    EXPECT_DOUBLE_EQ(math::interpolate(kRpm, kTorque, 20000.0), 380.0);
}
