#include <gtest/gtest.h>

#include "rov_motor_simulator/motor_model.hpp"
#include "rov_motor_simulator/motor_params.hpp"

using rov_motor_simulator::MotorParams;
using rov_motor_simulator::MotorRpm;

namespace
{

// Defaults: r = 0.05 m, track = 0.30 m, gear ratio 10, max 3000 motor RPM.
// 1 m/s at the wheel = 60 / (2 * pi * 0.05) = 190.986 wheel RPM = 1909.86 motor RPM.
constexpr double kMotorRpmPerMps = 1909.859317;
constexpr double kTol = 1e-3;

MotorParams defaults() {return MotorParams{};}

}  // namespace

TEST(Conversion, ZeroVelocityGivesZeroRpm)
{
  const MotorRpm rpm = rov_motor_simulator::velocity_to_target_rpm(0.0, 0.0, defaults());
  EXPECT_DOUBLE_EQ(rpm.left, 0.0);
  EXPECT_DOUBLE_EQ(rpm.right, 0.0);
}

TEST(Conversion, ForwardGivesEqualPositiveRpm)
{
  const MotorRpm rpm = rov_motor_simulator::velocity_to_target_rpm(0.5, 0.0, defaults());
  EXPECT_NEAR(rpm.left, 0.5 * kMotorRpmPerMps, kTol);
  EXPECT_NEAR(rpm.right, 0.5 * kMotorRpmPerMps, kTol);
}

TEST(Conversion, CombinedLinearAndAngular)
{
  // vL = 0.5 - 0.075 = 0.425 m/s; vR = 0.5 + 0.075 = 0.575 m/s
  const MotorRpm rpm = rov_motor_simulator::velocity_to_target_rpm(0.5, 0.5, defaults());
  EXPECT_NEAR(rpm.left, 0.425 * kMotorRpmPerMps, kTol);
  EXPECT_NEAR(rpm.right, 0.575 * kMotorRpmPerMps, kTol);
}

TEST(Saturation, LinearCommandIsClampedToMaxRpm)
{
  const MotorRpm rpm = rov_motor_simulator::velocity_to_target_rpm(5.0, 0.0, defaults());
  EXPECT_DOUBLE_EQ(rpm.left, 3000.0);
  EXPECT_DOUBLE_EQ(rpm.right, 3000.0);
}

TEST(Response, RampsUpAtAccelerationLimit)
{
  // 1500 RPM/s at 50 Hz = 30 RPM per tick
  double rpm = 0.0;
  const double target = 954.9296;
  for (int tick = 1; tick <= 10; ++tick) {
    const double previous = rpm;
    rpm = rov_motor_simulator::update_motor_rpm(rpm, target, 1500.0, 0.02);
    EXPECT_NEAR(rpm - previous, 30.0, 1e-9);
  }
  EXPECT_NEAR(rpm, 300.0, 1e-9);
}

TEST(Timeout, OldCommandIsStale)
{
  EXPECT_TRUE(rov_motor_simulator::command_is_stale(true, 0.51, 0.5));
}
