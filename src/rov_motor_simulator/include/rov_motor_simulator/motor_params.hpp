#ifndef ROV_MOTOR_SIMULATOR__MOTOR_PARAMS_HPP_
#define ROV_MOTOR_SIMULATOR__MOTOR_PARAMS_HPP_

#include <cmath>
#include <string>

namespace rov_motor_simulator
{

// Default values are assumptions for a small differential-drive test platform.
// They are not EyeROV hardware specifications.
struct MotorParams
{
  double wheel_radius_m = 0.05;
  double track_width_m = 0.30;
  double gear_ratio = 10.0;               // motor-shaft revolutions per wheel revolution
  double max_motor_rpm = 3000.0;          // motor-shaft limit
  double max_acceleration_rpm_s = 1500.0; // motor-shaft RPM change per second
  double update_rate_hz = 50.0;
  double command_timeout_s = 0.5;
};

constexpr double kMaxUpdateRateHz = 1000.0;

// Returns an empty string when the parameters are usable, otherwise the reason.
inline std::string validate(const MotorParams & p)
{
  const auto positive = [](double value) {return std::isfinite(value) && value > 0.0;};

  if (!positive(p.wheel_radius_m)) {return "wheel_radius_m must be a finite value > 0";}
  if (!positive(p.track_width_m)) {return "track_width_m must be a finite value > 0";}
  if (!positive(p.gear_ratio)) {return "gear_ratio must be a finite value > 0";}
  if (!positive(p.max_motor_rpm)) {return "max_motor_rpm must be a finite value > 0";}
  if (!positive(p.max_acceleration_rpm_s)) {
    return "max_acceleration_rpm_s must be a finite value > 0";
  }
  if (!positive(p.update_rate_hz) || p.update_rate_hz > kMaxUpdateRateHz) {
    return "update_rate_hz must be in the range (0, 1000]";
  }
  if (!positive(p.command_timeout_s)) {return "command_timeout_s must be a finite value > 0";}
  return "";
}

}  // namespace rov_motor_simulator

#endif  // ROV_MOTOR_SIMULATOR__MOTOR_PARAMS_HPP_
