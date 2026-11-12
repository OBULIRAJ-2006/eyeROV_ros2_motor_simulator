# ROS 2 Motor Simulator

**Name:** Obuliraj A N S
**Assignment:** EyeROV Technologies - Robotics and Firmware Intern Assignment
**Environment used for reported verification:** ROS 2 Humble in WSL2

## Project summary

This package receives velocity commands on `/cmd_vel`, calculates left and right motor-shaft RPM using a two-sided differential-drive model, simulates changes in motor speed with an acceleration limit, and publishes RPM telemetry. The work is a software simulation.

The model uses `linear.x` for forward or reverse motion and `angular.z` for turning. The remaining components of `geometry_msgs/msg/Twist` are not used.

## Data flow

1. The `motor_simulator` node receives a `geometry_msgs/msg/Twist` message on `/cmd_vel`.
2. The node converts the two supported velocity components to target left and right RPM.
3. A timer applies the configured acceleration limit and publishes the current simulated RPM.
4. If commands stop arriving for the configured timeout, the target returns to zero and the RPM ramps down.

The model calculations are in `include/rov_motor_simulator/motor_model.hpp`; ROS subscriptions, timers, parameters and publishers are handled in `src/motor_simulator_node.cpp`.

## Model

For linear velocity `v`, angular velocity `w`, and assumed track width `W`, the side speeds are:

```text
v_left  = v - (w * W / 2)
v_right = v + (w * W / 2)
```

For assumed wheel radius `r` and gear ratio `G`, the motor-shaft RPM is calculated as:

```text
wheel_rpm = speed * 60 / (2 * pi * r)
motor_rpm = wheel_rpm * G
```

The parameters in `config/motor_params.yaml` are example assumptions. The model scales both side targets together if either exceeds the RPM limit, preserving the requested left/right ratio. Simulated RPM approaches each target by at most `max_acceleration_rpm_s / update_rate_hz` per timer update.

## ROS 2 topics

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Input | Uses `linear.x` and `angular.z` |
| `/motor_rpm` | `std_msgs/msg/Float32MultiArray` | Output | `data[0]` left RPM; `data[1]` right RPM |
| `/left_motor/rpm` | `std_msgs/msg/Float32` | Output | Left simulated motor RPM |
| `/right_motor/rpm` | `std_msgs/msg/Float32` | Output | Right simulated motor RPM |

The node publishes telemetry at the configured update rate, including when stopped.

## Build and run

The package is intended for Ubuntu 22.04 with ROS 2 Humble. The reported verification environment was WSL2; setup details should match the ROS installation available on the machine being used.

```bash
# In the ROS 2 workspace, with this package under src/
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select rov_motor_simulator
source install/setup.bash
ros2 launch rov_motor_simulator motor_simulator.launch.py
```

To publish a forward command from a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

To test a turn while moving forward, publish a nonzero angular velocity:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.5}}"
```

In another terminal, inspect the topics and values:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /motor_rpm
```

Stop the command publisher with Ctrl+C; after the command timeout the simulated RPM should ramp down. Stop the simulator with Ctrl+C.

## Unit tests

Run the package tests with:

```bash
cd ~/ros2_ws
colcon test --packages-select rov_motor_simulator --event-handlers console_direct+
colcon test-result --verbose
```

The test source contains six cases: zero velocity, forward motion, combined linear and angular motion, RPM saturation, acceleration-limited response, and command timeout. In my earlier WSL2 run, the test summary reported six tests passing. I have not rerun the ROS 2 build or test suite for this update.

## Verification summary

In my WSL2 run, the package built and launched, the ROS 2 topics appeared, and I observed RPM telemetry for forward and turning commands. The test summary from that run reported six tests passing.

## Assumptions and limitations

- This is an assumed differential-drive model. It does not represent a confirmed EyeROV thruster layout.
- The radius, track width, gear ratio, speed limit, acceleration limit and timeout are example values, not confirmed hardware specifications.
- Only planar forward/reverse motion and yaw turning are represented. Heave, lateral motion, pitch, roll, fluid forces and buoyancy are outside the model.
- Verification was reported in WSL2 as a software simulation. No physical motors, ROV, underwater operation or hardware integration were tested.

## Package contents

- `src/rov_motor_simulator/src/` - ROS 2 node implementation.
- `src/rov_motor_simulator/include/` - motor calculations and parameters.
- `src/rov_motor_simulator/test/` - unit test source.
- `src/rov_motor_simulator/launch/` - launch file.
- `src/rov_motor_simulator/config/` - example parameter values.

