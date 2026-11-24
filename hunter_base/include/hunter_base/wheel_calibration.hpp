/*
 * wheel_calibration.hpp
 *
 * Description: per-robot correction of the Hunter's reported speed and
 *   steering angle, as estimated by jabas_calibration against RTK GNSS.
 *
 *   true speed              v = linear_velocity_scale * v_reported
 *   true central steering   d = steering_angle_scale * g * d_reported + steering_angle_offset
 *                           g = (1 + steering_angle_left_right_asymmetry * sgn(d_reported))
 *                             * (1 + steering_angle_forward_reverse_asymmetry * (v_reported < 0 ? -1 : 1))
 *   yaw rate                w = v * tan(d) / wheelbase
 *
 * The two asymmetries are small fractions (0 = symmetric, as before): the
 * steering gain is (1 + a) times the mean for left turns and (1 - a) times
 * for right turns, and likewise forward against reverse. A linkage with play
 * or unequal stops, or a steering sensor zeroed off-centre, gives different
 * left and right gains; tyre slip and steering compliance differ between
 * pulling and pushing the steered axle. A single scale and offset cannot
 * represent either, so the offset ends up absorbing them in turns.
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
  double steering_angle_left_right_asymmetry = 0.0;     // + = left turns steer more
  double steering_angle_forward_reverse_asymmetry = 0.0;  // + = forward steers more
  // Distance (m) of the published base frame ahead of the rear-axle centre,
  // the point the bicycle model integrates. 0 = publish the rear-axle pose.
  double odom_reference_offset_x = 0.0;

  bool IsIdentity() const {
    return linear_velocity_scale == 1.0 && steering_angle_scale == 1.0 &&
           steering_angle_offset == 0.0 && steering_angle_left_right_asymmetry == 0.0 &&
           steering_angle_forward_reverse_asymmetry == 0.0;
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
    if (!(std::fabs(steering_angle_left_right_asymmetry) < 0.1))
      err << "steering_angle_left_right_asymmetry " << steering_angle_left_right_asymmetry
          << " outside (-0.1, 0.1); ";
    if (!(std::fabs(steering_angle_forward_reverse_asymmetry) < 0.1))
      err << "steering_angle_forward_reverse_asymmetry " << steering_angle_forward_reverse_asymmetry
          << " outside (-0.1, 0.1); ";
    if (!(std::fabs(odom_reference_offset_x) < 2.0))
      err << "odom_reference_offset_x " << odom_reference_offset_x << " m outside (-2, 2); ";
    if (!err.str().empty())
      throw std::invalid_argument("invalid wheel calibration: " + err.str());
  }

  // reported -> true
  double TrueSpeed(double v_reported) const { return linear_velocity_scale * v_reported; }
  // v: speed (reported or true, only its sign is used; 0 counts as forward)
  double TrueSteering(double central_reported, double v) const {
    return Gain(central_reported > 0 ? 1.0 : central_reported < 0 ? -1.0 : 0.0, v) * central_reported +
           steering_angle_offset;
  }

  // true -> value to command (inverse of the above)
  double ReportedSpeed(double v_true) const { return v_true / linear_velocity_scale; }
  // Exact inverse: the gain is positive (|asymmetries| < 0.1), so the
  // reported angle has the sign of central_true - offset.
  double ReportedSteering(double central_true, double v) const {
    const double d = central_true - steering_angle_offset;
    return d / Gain(d > 0 ? 1.0 : d < 0 ? -1.0 : 0.0, v);
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
    return std::max(-max_central, std::min(max_central, ReportedSteering(central_true, v)));
  }

 private:
  // steering scale for a turn side (+1 left, -1 right, 0 straight) and driving direction
  double Gain(double side, double v) const {
    return steering_angle_scale * (1.0 + steering_angle_left_right_asymmetry * side) *
           (1.0 + steering_angle_forward_reverse_asymmetry * (v < 0 ? -1.0 : 1.0));
  }
};

}  // namespace westonrobot

#endif  // HUNTER_BASE_WHEEL_CALIBRATION_HPP
