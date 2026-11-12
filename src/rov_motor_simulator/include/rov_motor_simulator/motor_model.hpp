#ifndef ROV_MOTOR_SIMULATOR__MOTOR_MODEL_HPP_
#define ROV_MOTOR_SIMULATOR__MOTOR_MODEL_HPP_

#include <algorithm>
#include <cmath>

#include "rov_motor_simulator/motor_params.hpp"

namespace rov_motor_simulator
{

constexpr double kPi = 3.14159265358979323846;

struct MotorRpm
{
  double left = 0.0;
  double right = 0.0;
};

// Wheel RPM from a wheel surface speed in m/s: rpm = v * 60 / (2 * pi * r)
inline double wheel_rpm_from_speed(double wheel_speed_mps, double wheel_radius_m)
{
  return wheel_speed_mps * 60.0 / (2.0 * kPi * wheel_radius_m);
}

// Converts a body velocity command into target motor-shaft RPM.
// Positive angular_z is anticlockwise seen from above (REP 103), so the right side runs faster.
// If either side exceeds max_motor_rpm, both are scaled by the same factor so the
// commanded turning ratio is preserved. Non-finite results give a zero target.
inline MotorRpm velocity_to_target_rpm(double linear_x, double angular_z, const MotorParams & p)
{
  const double half_track = angular_z * p.track_width_m / 2.0;
  const double left_speed = linear_x - half_track;
  const double right_speed = linear_x + half_track;

  MotorRpm rpm;
  rpm.left = wheel_rpm_from_speed(left_speed, p.wheel_radius_m) * p.gear_ratio;
  rpm.right = wheel_rpm_from_speed(right_speed, p.wheel_radius_m) * p.gear_ratio;

  if (!std::isfinite(rpm.left) || !std::isfinite(rpm.right)) {
    return MotorRpm{};
  }

  const double largest = std::max(std::fabs(rpm.left), std::fabs(rpm.right));
  if (largest > p.max_motor_rpm) {
    const double scale = p.max_motor_rpm / largest;
    rpm.left *= scale;
    rpm.right *= scale;
  }
  return rpm;
}

// Moves current toward target by at most max_delta. Never overshoots the target.
inline double step_toward(double current, double target, double max_delta)
{
  const double error = target - current;
  if (std::fabs(error) <= max_delta) {
    return target;
  }
  return current + std::copysign(max_delta, error);
}

// One simulation tick of a motor with a limited acceleration (RPM per second).
inline double update_motor_rpm(
  double current_rpm, double target_rpm, double max_acceleration_rpm_s, double dt_s)
{
  return step_toward(current_rpm, target_rpm, max_acceleration_rpm_s * dt_s);
}

// A command is stale if none has ever arrived, or the last one is older than the timeout.
inline bool command_is_stale(bool has_command, double age_s, double timeout_s)
{
  return !has_command || age_s > timeout_s;
}

}  // namespace rov_motor_simulator

#endif  // ROV_MOTOR_SIMULATOR__MOTOR_MODEL_HPP_
