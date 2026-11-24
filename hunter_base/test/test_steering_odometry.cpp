// Regression test: odometry, /hunter_status and the steering command on wide
// curves, through the real HunterMessenger with a stand-in chassis (no
// hardware or CAN bus needed).
//
// The stand-in encodes commands like the SDK's CAN codec ((int16_t)(x * 1000)
// / 1000: 1 mm/s and 1 mrad, truncated), reports back what it was commanded,
// and moves as an Ackermann vehicle: the inner front-wheel angle puts the
// instantaneous centre of rotation on the rear-axle line at
// R = w/2 + l / tan|inner| from the rear-axle centre. With a wheel calibration
// it obeys the calibration model exactly.
//
// All topics are remapped into a private namespace and no TF is published, so
// running this next to a live robot cannot command it or disturb its topics.
#include <gtest/gtest.h>
#include <rcl/time.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "hunter_base/hunter_messenger.hpp"
#include "hunter_base/hunter_params.hpp"

using westonrobot::HunterMessenger;
using westonrobot::HunterV2Params;
using westonrobot::WheelCalibration;

namespace {

constexpr double kL = HunterV2Params::wheelbase;
constexpr double kW = HunterV2Params::track;
constexpr double kDt = 0.02;  // driver loop, 50 Hz
constexpr double kDeg = 3.14159265358979323846 / 180.0;

// the SDK's CAN encoding: int16 in 1/1000 units, truncated
double Can(double x) { return static_cast<int16_t>(x * 1000) / 1000.0; }

// robot_configurations platforms/hunter/robots/hunter_002/hunter_base.yaml
WheelCalibration Hunter002() {
  WheelCalibration c;
  c.linear_velocity_scale = 0.88688;
  c.steering_angle_scale = 0.86786;
  c.steering_angle_offset = -0.000495;
  c.odom_reference_offset_x = 0.3186;
  return c;
}

struct StandInHunter {
  WheelCalibration chassis;  // true speed / steering vs the reported ones
  double v_reported = 0.0;
  double inner_reported = 0.0;
  double x = 0.0, y = 0.0, yaw = 0.0;  // rear-axle centre

  void SetMotionCommand(double v, double steering) {
    v_reported = Can(v);
    inner_reported = Can(steering);
  }
  westonrobot::HunterCoreState GetRobotState() {
    westonrobot::HunterCoreState s{};
    s.system_state.vehicle_state = VEHICLE_STATE_NORMAL;
    s.motion_state.linear_velocity = v_reported;
    s.motion_state.steering_angle = inner_reported;
    return s;
  }
  westonrobot::HunterActuatorState GetActuatorState() { return {}; }
  westonrobot::HunterCommonSensorState GetCommonSensorState() { return {}; }

  void Step(double dt) {
    double central = 0.0;  // of the reported inner angle, from the ICR
    if (inner_reported != 0.0) {
      const double r = kW / 2 + kL / std::tan(std::fabs(inner_reported));
      central = std::copysign(std::atan(kL / r), inner_reported);
    }
    const double v = chassis.TrueSpeed(v_reported);
    const double yaw_rate = v * std::tan(chassis.TrueSteering(central, v_reported)) / kL;
    const double dyaw = yaw_rate * dt;
    if (dyaw == 0.0) {
      x += v * dt * std::cos(yaw);
      y += v * dt * std::sin(yaw);
    } else {  // exact arc
      const double r = v / yaw_rate;
      x += r * (std::sin(yaw + dyaw) - std::sin(yaw));
      y -= r * (std::cos(yaw + dyaw) - std::cos(yaw));
    }
    yaw += dyaw;
  }
};

struct DriveResult {
  double base_yaw;         // true heading change, rad
  double odom_yaw;         // odometry heading change, rad
  double sideways;         // odometry position error across the true heading, m
  double inner_command;    // inner-wheel angle the chassis was sent, rad
  double status_steering;  // /hunter_status steering_angle, rad
};

// One HunterMessenger for the whole suite: PublishStateToROS() keeps its
// first-call flag in a function-local static.
class SteeringOdometryTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
    const std::string ns = "/hunter_base_steering_test";
    rclcpp::NodeOptions options;
    options.use_global_arguments(false)
        .use_intra_process_comms(true)  // delivered on the next spin_some
        .start_parameter_services(false)
        .start_parameter_event_publisher(false)
        .enable_rosout(false)
        .arguments({"--ros-args",
                    "-r", "/cmd_vel:=" + ns + "/cmd_vel",
                    "-r", "/hunter_status:=" + ns + "/hunter_status",
                    "-r", "/hunter_bms:=" + ns + "/hunter_bms",
                    "-r", "/odom:=" + ns + "/odom"});
    node_ = std::make_shared<rclcpp::Node>("hunter_base_steering_test", options);
    clock_ = node_->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock_, now_ns_), RCL_RET_OK);

    base_ = std::make_shared<StandInHunter>();
    messenger_ = std::make_unique<HunterMessenger<StandInHunter>>(base_, node_.get());
    // as HunterBaseRos::Run() for robot_model "hunter2"
    messenger_->SetTrack(HunterV2Params::track);
    messenger_->SetWeelbase(HunterV2Params::wheelbase);
    messenger_->SetMaxSteerAngleCentral(HunterV2Params::max_steer_angle_central);
    messenger_->SetMaxSteerAngle(HunterV2Params::max_steer_angle);
    messenger_->SetOdometryFrame("odom");
    messenger_->SetBaseFrame("base_link");
    messenger_->SetOdometryTopicName("odom");
    messenger_->SetPublishOdometryTF(false);
    messenger_->SetupSubscription();

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom_ = *m; });
    status_sub_ = node_->create_subscription<hunter_msgs::msg::HunterStatus>(
        "/hunter_status", 10,
        [](hunter_msgs::msg::HunterStatus::ConstSharedPtr m) { status_ = *m; });
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    messenger_->PublishStateToROS();  // first call only latches the clock
  }

  static void TearDownTestSuite() {
    cmd_pub_.reset();
    status_sub_.reset();
    odom_sub_.reset();
    messenger_.reset();
    base_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  // Drives `distance` m at `speed` m/s (< 0 reverses) on a circle of
  // `radius` m (> 0 left, < 0 right, 0 straight), from a fresh odometry origin.
  static DriveResult Drive(double radius, double speed, double distance,
                           const WheelCalibration &calibration = WheelCalibration{}) {
    messenger_->SetWheelCalibration(calibration);  // also restarts the odometry
    *base_ = StandInHunter{};
    base_->chassis = calibration;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed;
    cmd.angular.z = radius == 0.0 ? 0.0 : speed / radius;
    cmd_pub_->publish(cmd);
    rclcpp::spin_some(node_);  // /cmd_vel -> SetMotionCommand

    const int steps = static_cast<int>(std::lround(distance / std::fabs(speed) / kDt));
    for (int k = 0; k < steps; ++k) {
      base_->Step(kDt);
      now_ns_ += static_cast<int64_t>(kDt * 1e9);
      EXPECT_EQ(rcl_set_ros_time_override(clock_, now_ns_), RCL_RET_OK);
      messenger_->PublishStateToROS();
      rclcpp::spin_some(node_);  // odometry, /hunter_status
    }

    const auto &q = odom_.pose.pose.orientation;
    const double odom_yaw =
        std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    // the odometry publishes base_link, odom_reference_offset_x ahead of the
    // rear axle, starting at the origin: compare rear-axle positions
    const double ref = calibration.odom_reference_offset_x;
    const double ex = odom_.pose.pose.position.x - ref * std::cos(odom_yaw) + ref - base_->x;
    const double ey = odom_.pose.pose.position.y - ref * std::sin(odom_yaw) - base_->y;

    DriveResult r;
    r.base_yaw = base_->yaw;
    r.odom_yaw = odom_yaw;
    r.sideways = -ex * std::sin(base_->yaw) + ey * std::cos(base_->yaw);
    r.inner_command = base_->inner_reported;
    r.status_steering = status_.steering_angle;
    return r;
  }

  // The odometry integrator runs up to 101 RK4 sub-steps of dt/100 per cycle
  // (up to 1 % too far), so heading is compared to 2 % (+0.05 deg).
  static void ExpectOdometryFollowsChassis(const DriveResult &r) {
    EXPECT_NEAR(r.odom_yaw, r.base_yaw, 0.02 * std::fabs(r.base_yaw) + 0.05 * kDeg)
        << "odometry " << r.odom_yaw / kDeg << " deg, chassis " << r.base_yaw / kDeg << " deg";
    EXPECT_NEAR(r.sideways, 0.0, 0.05);
  }

  static rclcpp::Node::SharedPtr node_;
  static rcl_clock_t *clock_;
  static int64_t now_ns_;
  static std::shared_ptr<StandInHunter> base_;
  static std::unique_ptr<HunterMessenger<StandInHunter>> messenger_;
  static rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  static rclcpp::Subscription<hunter_msgs::msg::HunterStatus>::SharedPtr status_sub_;
  static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  static nav_msgs::msg::Odometry odom_;
  static hunter_msgs::msg::HunterStatus status_;
};

rclcpp::Node::SharedPtr SteeringOdometryTest::node_;
rcl_clock_t *SteeringOdometryTest::clock_ = nullptr;
int64_t SteeringOdometryTest::now_ns_ = 1000000000;
std::shared_ptr<StandInHunter> SteeringOdometryTest::base_;
std::unique_ptr<HunterMessenger<StandInHunter>> SteeringOdometryTest::messenger_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr SteeringOdometryTest::odom_sub_;
rclcpp::Subscription<hunter_msgs::msg::HunterStatus>::SharedPtr SteeringOdometryTest::status_sub_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr SteeringOdometryTest::cmd_pub_;
nav_msgs::msg::Odometry SteeringOdometryTest::odom_;
hunter_msgs::msg::HunterStatus SteeringOdometryTest::status_;

struct Curve {
  double radius, speed, distance;
};

}  // namespace

// A 150 m radius at 1 m/s goes out as a 4 mrad inner-wheel angle, inside the
// former +-5 mrad deadband: over 20 m the chassis turns 7.0 deg.
TEST_F(SteeringOdometryTest, OdometryFollowsWideCurve) {
  const auto r = Drive(150.0, 1.0, 20.0);
  EXPECT_DOUBLE_EQ(r.inner_command, 0.004);
  EXPECT_NEAR(r.base_yaw, 7.04 * kDeg, 0.01 * kDeg);
  ExpectOdometryFollowsChassis(r);
  // the calibration fits /hunter_status: same central angle as the odometry
  EXPECT_NEAR(r.status_steering, 0.0039926, 1e-6);
}

TEST_F(SteeringOdometryTest, OdometryFollowsWideCurvesRightReversingAndAt1mrad) {
  for (const Curve c : {Curve{-150.0, 1.0, 20.0}, Curve{150.0, -1.0, 10.0},
                        Curve{500.0, 1.0, 20.0}}) {
    SCOPED_TRACE("radius " + std::to_string(c.radius) + " m, speed " +
                 std::to_string(c.speed) + " m/s");
    const auto r = Drive(c.radius, c.speed, c.distance);
    EXPECT_NE(r.inner_command, 0.0);
    ExpectOdometryFollowsChassis(r);
  }
}

// Outside the former deadband the conversions are unchanged.
TEST_F(SteeringOdometryTest, OdometryFollowsTighterCurvesAndStraight) {
  for (const Curve c : {Curve{50.0, 1.0, 20.0}, Curve{-5.0, 1.0, 10.0},
                        Curve{0.0, 1.0, 20.0}}) {
    SCOPED_TRACE("radius " + std::to_string(c.radius) + " m");
    ExpectOdometryFollowsChassis(Drive(c.radius, c.speed, c.distance));
  }
}

// Calibrated, the command goes through the central -> inner conversion: a
// 200 m curve (4.3 mrad) must be commanded, not sent as straight.
TEST_F(SteeringOdometryTest, CalibratedCommandAndOdometryFollowWideCurve) {
  const auto r = Drive(200.0, 1.0, 20.0, Hunter002());
  EXPECT_GT(r.inner_command, 0.0);
  // 20 m / 200 m, short only by the 1 mrad truncation on the CAN bus
  EXPECT_GT(r.base_yaw, 0.5 * 20.0 / 200.0);
  ExpectOdometryFollowsChassis(r);

  ExpectOdometryFollowsChassis(Drive(150.0, 1.0, 20.0, Hunter002()));
}

// A chassis whose steering gain differs left/right and forward/reverse
// (hunter_001, jabas_calibration 2026-11-24): commands still realise the
// requested curvature and the odometry follows, in all four cases.
TEST_F(SteeringOdometryTest, AsymmetricCalibrationFollowsAllFourCases) {
  WheelCalibration c = Hunter002();
  c.steering_angle_offset = 0.005028;
  c.steering_angle_left_right_asymmetry = 0.01572;
  c.steering_angle_forward_reverse_asymmetry = -0.01212;
  for (const Curve k : {Curve{5.0, 1.0, 10.0}, Curve{-5.0, 1.0, 10.0},
                        Curve{5.0, -1.0, 10.0}, Curve{-5.0, -1.0, 10.0}}) {
    SCOPED_TRACE("radius " + std::to_string(k.radius) + " m, speed " +
                 std::to_string(k.speed) + " m/s");
    const auto r = Drive(k.radius, k.speed, k.distance, c);
    // 10 m on a 5 m radius: 2 rad, short only by the 1 mrad / 1 mm/s CAN truncation (< 1 %)
    EXPECT_NEAR(std::fabs(r.base_yaw), k.distance / std::fabs(k.radius), 0.02);
    ExpectOdometryFollowsChassis(r);
  }
}
