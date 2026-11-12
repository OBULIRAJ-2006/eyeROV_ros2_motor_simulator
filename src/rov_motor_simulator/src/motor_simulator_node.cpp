#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "rov_motor_simulator/motor_model.hpp"
#include "rov_motor_simulator/motor_params.hpp"

namespace rov_motor_simulator
{

class MotorSimulatorNode : public rclcpp::Node
{
public:
  MotorSimulatorNode()
  : Node("motor_simulator")
  {
    declare_and_load_parameters();

    const std::string error = validate(params_);
    if (!error.empty()) {
      throw std::invalid_argument("Invalid parameters: " + error);
    }

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&MotorSimulatorNode::on_cmd_vel, this, std::placeholders::_1));

    motor_rpm_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/motor_rpm", 10);
    left_rpm_pub_ = create_publisher<std_msgs::msg::Float32>("/left_motor/rpm", 10);
    right_rpm_pub_ = create_publisher<std_msgs::msg::Float32>("/right_motor/rpm", 10);

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&MotorSimulatorNode::on_parameters_changed, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / params_.update_rate_hz),
      std::bind(&MotorSimulatorNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "Started: %.1f Hz, timeout %.2f s, max %.0f motor RPM, gear ratio %.2f",
      params_.update_rate_hz, params_.command_timeout_s, params_.max_motor_rpm,
      params_.gear_ratio);
  }

private:
  void declare_and_load_parameters()
  {
    const MotorParams defaults;

    params_.wheel_radius_m = declare_parameter("wheel_radius_m", defaults.wheel_radius_m);
    params_.track_width_m = declare_parameter("track_width_m", defaults.track_width_m);
    params_.gear_ratio = declare_parameter("gear_ratio", defaults.gear_ratio);
    params_.max_motor_rpm = declare_parameter("max_motor_rpm", defaults.max_motor_rpm);
    params_.max_acceleration_rpm_s =
      declare_parameter("max_acceleration_rpm_s", defaults.max_acceleration_rpm_s);
    params_.command_timeout_s =
      declare_parameter("command_timeout_s", defaults.command_timeout_s);

    // The timer period is fixed at startup, so this one cannot change at runtime.
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    params_.update_rate_hz = declare_parameter("update_rate_hz", defaults.update_rate_hz, read_only);
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
    const std::vector<rclcpp::Parameter> & changes)
  {
    MotorParams candidate = params_;
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // rclcpp has already rejected wrong-typed values (e.g. an integer) at this point.
    for (const auto & change : changes) {
      const std::string & name = change.get_name();
      if (name == "wheel_radius_m") {
        candidate.wheel_radius_m = change.as_double();
      } else if (name == "track_width_m") {
        candidate.track_width_m = change.as_double();
      } else if (name == "gear_ratio") {
        candidate.gear_ratio = change.as_double();
      } else if (name == "max_motor_rpm") {
        candidate.max_motor_rpm = change.as_double();
      } else if (name == "max_acceleration_rpm_s") {
        candidate.max_acceleration_rpm_s = change.as_double();
      } else if (name == "command_timeout_s") {
        candidate.command_timeout_s = change.as_double();
      }
    }

    const std::string error = validate(candidate);
    if (!error.empty()) {
      result.successful = false;
      result.reason = error;
      return result;
    }

    params_ = candidate;
    return result;
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring cmd_vel with NaN or infinite value");
      return;
    }

    cmd_linear_x_ = msg->linear.x;
    cmd_angular_z_ = msg->angular.z;
    last_command_time_ = std::chrono::steady_clock::now();
    has_command_ = true;

    if (timed_out_) {
      timed_out_ = false;
      RCLCPP_INFO(get_logger(), "Velocity commands resumed");
    }
  }

  void on_timer()
  {
    const double dt = 1.0 / params_.update_rate_hz;
    const double command_age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_command_time_).count();
    const bool stale = command_is_stale(has_command_, command_age_s, params_.command_timeout_s);

    MotorRpm target;  // zero unless a fresh command is available
    if (stale) {
      if (has_command_ && !timed_out_) {
        timed_out_ = true;
        RCLCPP_WARN(
          get_logger(), "No cmd_vel for %.2f s (timeout %.2f s): ramping motors to zero",
          command_age_s, params_.command_timeout_s);
      }
    } else {
      target = velocity_to_target_rpm(cmd_linear_x_, cmd_angular_z_, params_);
    }

    left_rpm_ = update_motor_rpm(left_rpm_, target.left, params_.max_acceleration_rpm_s, dt);
    right_rpm_ = update_motor_rpm(right_rpm_, target.right, params_.max_acceleration_rpm_s, dt);

    publish_telemetry();
  }

  void publish_telemetry()
  {
    std_msgs::msg::Float32 left_msg;
    std_msgs::msg::Float32 right_msg;
    left_msg.data = static_cast<float>(left_rpm_);
    right_msg.data = static_cast<float>(right_rpm_);

    // data[0] = left motor RPM, data[1] = right motor RPM
    std_msgs::msg::Float32MultiArray combined_msg;
    combined_msg.data = {left_msg.data, right_msg.data};

    left_rpm_pub_->publish(left_msg);
    right_rpm_pub_->publish(right_msg);
    motor_rpm_pub_->publish(combined_msg);
  }

  MotorParams params_;

  double cmd_linear_x_ = 0.0;
  double cmd_angular_z_ = 0.0;
  bool has_command_ = false;
  bool timed_out_ = false;
  // Use a monotonic clock for watchdog timing; ROS simulated time may pause or jump.
  std::chrono::steady_clock::time_point last_command_time_{};

  double left_rpm_ = 0.0;
  double right_rpm_ = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_rpm_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace rov_motor_simulator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rov_motor_simulator::MotorSimulatorNode> node;
  try {
    node = std::make_shared<rov_motor_simulator::MotorSimulatorNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("motor_simulator"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
