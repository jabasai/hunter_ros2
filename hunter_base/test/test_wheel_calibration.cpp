// Unit tests for the wheel-odometry calibration math (no hardware needed).
#include <gtest/gtest.h>

#include <cmath>

#include "hunter_base/wheel_calibration.hpp"

using westonrobot::WheelCalibration;

namespace {
constexpr double kL = 0.650;          // Hunter 2.0 wheelbase used by the driver
constexpr double kMaxCentral = 0.461; // HunterV2Params::max_steer_angle_central

WheelCalibration Calibrated() {
  WheelCalibration c;
  c.linear_velocity_scale = 0.887;
  c.steering_angle_scale = 0.868;
  c.steering_angle_offset = -0.0005;
  c.odom_reference_offset_x = 0.319;
  return c;
}
}  // namespace

TEST(WheelCalibration, DefaultIsIdentity) {
  WheelCalibration c;
  EXPECT_TRUE(c.IsIdentity());
  EXPECT_DOUBLE_EQ(c.TrueSpeed(1.2), 1.2);
  EXPECT_DOUBLE_EQ(c.TrueSteering(0.3, 1.0), 0.3);
  EXPECT_DOUBLE_EQ(c.TrueSteering(-0.3, -1.0), -0.3);
  EXPECT_NO_THROW(c.Validate());
}

TEST(WheelCalibration, ReportedToTrueAndBack) {
  const auto c = Calibrated();
  EXPECT_FALSE(c.IsIdentity());
  EXPECT_NEAR(c.TrueSpeed(1.0), 0.887, 1e-12);
  EXPECT_NEAR(c.ReportedSpeed(c.TrueSpeed(1.3)), 1.3, 1e-12);
  for (double v : {1.0, -1.0})
    for (double d : {-0.4, -0.02, 0.0, 0.1, 0.45})
      EXPECT_NEAR(c.ReportedSteering(c.TrueSteering(d, v), v), d, 1e-12);
}

TEST(WheelCalibration, CommandRealisesRequestedCurvature) {
  const auto c = Calibrated();
  // a 2.5 m radius left turn, forward and in reverse
  for (double v : {1.0, -0.8}) {
    const double w = std::fabs(v) / 2.5 * (v > 0 ? 1.0 : -1.0);  // left turn
    const double reported = c.CommandSteering(v, w, kL, kMaxCentral);
    const double central_true = c.TrueSteering(reported, v);
    const double w_true = c.TrueSpeed(c.ReportedSpeed(v)) * std::tan(central_true) / kL;
    EXPECT_NEAR(w_true, w, 1e-9) << "v=" << v;
  }
}

TEST(WheelCalibration, StraightCommandCompensatesOffset) {
  const auto c = Calibrated();
  const double reported = c.CommandSteering(1.0, 0.0, kL, kMaxCentral);
  EXPECT_NEAR(c.TrueSteering(reported, 1.0), 0.0, 1e-12);
}

static WheelCalibration Asymmetric() {
  auto c = Calibrated();
  c.steering_angle_left_right_asymmetry = 0.016;
  c.steering_angle_forward_reverse_asymmetry = -0.012;
  return c;
}

TEST(WheelCalibration, AsymmetryScalesEachSideAndDirection) {
  const auto c = Asymmetric();
  EXPECT_FALSE(c.IsIdentity());
  const double s = c.steering_angle_scale, o = c.steering_angle_offset;
  EXPECT_NEAR(c.TrueSteering(0.2, 1.0), s * 1.016 * 0.988 * 0.2 + o, 1e-12);    // left, forward
  EXPECT_NEAR(c.TrueSteering(-0.2, 1.0), -s * 0.984 * 0.988 * 0.2 + o, 1e-12);  // right, forward
  EXPECT_NEAR(c.TrueSteering(0.2, -1.0), s * 1.016 * 1.012 * 0.2 + o, 1e-12);   // left, reverse
  EXPECT_NEAR(c.TrueSteering(-0.2, -1.0), -s * 0.984 * 1.012 * 0.2 + o, 1e-12); // right, reverse
  // continuous through zero: both sides meet at the offset
  EXPECT_NEAR(c.TrueSteering(1e-9, 1.0), o, 1e-8);
  EXPECT_NEAR(c.TrueSteering(-1e-9, 1.0), o, 1e-8);
  EXPECT_DOUBLE_EQ(c.TrueSteering(0.0, 1.0), o);
  // standing still counts as forward
  EXPECT_DOUBLE_EQ(c.TrueSteering(0.2, 0.0), c.TrueSteering(0.2, 1.0));
}

TEST(WheelCalibration, AsymmetricRoundTripAndCommand) {
  const auto c = Asymmetric();
  for (double v : {1.0, -1.0, 0.0})
    for (double d : {-0.4, -0.02, -0.0005, 0.0, 0.0005, 0.1, 0.45}) {
      EXPECT_NEAR(c.ReportedSteering(c.TrueSteering(d, v), v), d, 1e-12) << "v=" << v << " d=" << d;
      EXPECT_NEAR(c.TrueSteering(c.ReportedSteering(d, v), v), d, 1e-12) << "v=" << v << " d=" << d;
    }
  // 2.5 m radius, left and right, forward and reverse
  for (double v : {1.0, -0.8})
    for (double side : {1.0, -1.0}) {
      const double w = side * v / 2.5;
      const double reported = c.CommandSteering(v, w, kL, kMaxCentral);
      const double w_true = c.TrueSpeed(c.ReportedSpeed(v)) * std::tan(c.TrueSteering(reported, v)) / kL;
      EXPECT_NEAR(w_true, w, 1e-9) << "v=" << v << " side=" << side;
    }
  EXPECT_NEAR(c.TrueSteering(c.CommandSteering(-1.0, 0.0, kL, kMaxCentral), -1.0), 0.0, 1e-12);
}

TEST(WheelCalibration, SaturatesAtChassisLimit) {
  const auto c = Calibrated();
  // radius below what the chassis can do, and turning on the spot
  EXPECT_DOUBLE_EQ(c.CommandSteering(0.5, 1.0, kL, kMaxCentral), kMaxCentral);
  EXPECT_DOUBLE_EQ(c.CommandSteering(0.0, -0.5, kL, kMaxCentral), -kMaxCentral);
}

TEST(WheelCalibration, RejectsImplausibleValues) {
  WheelCalibration c;
  c.linear_velocity_scale = 0.2;
  EXPECT_THROW(c.Validate(), std::invalid_argument);
  c = WheelCalibration{};
  c.steering_angle_offset = 0.3;
  EXPECT_THROW(c.Validate(), std::invalid_argument);
  c = WheelCalibration{};
  c.odom_reference_offset_x = 5.0;
  EXPECT_THROW(c.Validate(), std::invalid_argument);
  c = WheelCalibration{};
  c.steering_angle_left_right_asymmetry = 0.15;
  EXPECT_THROW(c.Validate(), std::invalid_argument);
  c = WheelCalibration{};
  c.steering_angle_forward_reverse_asymmetry = -0.15;
  EXPECT_THROW(c.Validate(), std::invalid_argument);
}
