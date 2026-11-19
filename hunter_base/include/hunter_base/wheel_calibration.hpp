/*
 * wheel_calibration.hpp
 *
 * Description: per-robot correction of the Hunter's reported speed and
 *   steering angle, as estimated by jabas_calibration against RTK GNSS.
 *
 *   true speed              v = linear_velocity_scale * v_reported
 *   true central steering   d = steering_angle_scale * d_reported + steering_angle_offset
 *   yaw rate                w = v * tan(d) / wheelbase
 *
 * /hunter_status keeps the raw (reported) values; the odometry uses the true
 * ones, and velocity commands are mapped through the inverse so the robot
 * drives the commanded speed and curvature. The defaults (1, 1, 0) reproduce
 * the uncalibrated behaviour exactly.
 */

#ifndef HUNTER_BASE_WHEEL_CALIBRATION_HPP
#define HUNTER_BASE_WHEEL_CALIBRATION_HPP

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace westonrobot {

struct WheelCalibration {
  double linear_velocity_scale = 1.0;
  double steering_angle_scale = 1.0;
  double steering_angle_offset = 0.0;  // rad, central angle
  // Distance (m) of the published base frame ahead of the rear-axle centre,
  // the point the bicycle model integrates. 0 = publish the rear-axle pose.
  double odom_reference_offset_x = 0.0;

  bool IsIdentity() const {
    return linear_velocity_scale == 1.0 && steering_angle_scale == 1.0 &&
           steering_angle_offset == 0.0;
  }

  // Rejects values that can only come from a typo or a failed calibration.
  void Validate() const {
    std::ostringstream err;
    if (!(linear_velocity_scale > 0.5 && linear_velocity_scale < 1.5))
      err << "linear_velocity_scale " << linear_velocity_scale << " outside (0.5, 1.5); ";
    if (!(steering_angle_scale > 0.5 && steering_angle_scale < 1.5))
      err << "steering_angle_scale " << steering_angle_scale << " outside (0.5, 1.5); ";
    if (!(std::fabs(steering_angle_offset) < 0.1))
      err << "steering_angle_offset " << steering_angle_offset << " rad outside (-0.1, 0.1); ";
    if (!(std::fabs(odom_reference_offset_x) < 2.0))
      err << "odom_reference_offset_x " << odom_reference_offset_x << " m outside (-2, 2); ";
    if (!err.str().empty())
      throw std::invalid_argument("invalid wheel calibration: " + err.str());
  }

  // reported -> true
  double TrueSpeed(double v_reported) const { return linear_velocity_scale * v_reported; }
  double TrueSteering(double central_reported) const {
    return steering_angle_scale * central_reported + steering_angle_offset;
  }

  // true -> value to command (inverse of the above)
  double ReportedSpeed(double v_true) const { return v_true / linear_velocity_scale; }
  double ReportedSteering(double central_true) const {
    return (central_true - steering_angle_offset) / steering_angle_scale;
  }

  // Central steering angle (in the chassis' reported units, clamped to
  // +-max_central) that realises a twist command (v, w) with the true
  // bicycle model of wheelbase l. Standing still with w != 0 steers to the
  // limit in the direction of w, like the uncalibrated driver.
  double CommandSteering(double v, double w, double l, double max_central) const {
    double central_true = 0.0;
    if (w != 0.0) {
      const double radius = std::fabs(v) / std::fabs(w);
      central_true = std::atan2(l, radius);                    // in (0, pi/2]
      const double sign = (w > 0 ? 1.0 : -1.0) * (v < 0 ? -1.0 : 1.0);
      central_true *= sign;
    }
    // C++14 (no std::clamp)
    return std::max(-max_central, std::min(max_central, ReportedSteering(central_true)));
  }
};

}  // namespace westonrobot

#endif  // HUNTER_BASE_WHEEL_CALIBRATION_HPP
