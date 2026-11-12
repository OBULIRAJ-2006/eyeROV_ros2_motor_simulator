#include <gtest/gtest.h>

#include <cmath>
#include <limits>

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

TEST(Conversion, ReverseGivesEqualNegativeRpm)
{
  const MotorRpm rpm = rov_motor_simulator::velocity_to_target_rpm(-0.5, 0.0, defaults());
  EXPECT_NEAR(rpm.left, -0.5 * kMotorRpmPerMps, kTol);
  EXPECT_NEAR(rpm.right, -0.5 * kMotorRpmPerMps, kTol);
}

TEST(Conversion, AnticlockwiseInPlaceTurnsRightForwardLeftBackward)
{
  // vR = +0.15 m/s, vL = -0.15 m/s for angular.z = +1 rad/s and track 0.30 m
  const MotorRpm rpm = rov_motor_simulator::velocity_to_target_rpm(0.0, 1.0, defaults());
  EXPECT_NEAR(rpm.left, -0.15 * kMotorRpmPerMps, kTol);
  EXPECT_NEAR(rpm.right, 0.15 * kMotorRpmPerMps, kTol);
}

TEST(Conversion, ClockwiseInPlaceIsTheMirrorImage)
{
  const MotorRpm rpm = rov_motor_simulator::velocity_to_target_rpm(0.0, -1.0, defaults());
  EXPECT_NEAR(rpm.left, 0.15 * kMotorRpmPerMps, kTol);
  EXPECT_NEAR(rpm.right, -0.15 * kMotorRpmPerMps, kTol);
}

TEST(Conversion, CombinedLinearAndAngular)
{
  // vL = 0.5 - 0.15 = 0.35 m/s, vR = 0.5 + 0.15 = 0.65 m/s
  const MotorRpm rpm = rov_motor_simulator::velocity_to_target_rpm(0.5, 1.0, defaults());
  EXPECT_NEAR(rpm.left, 0.35 * kMotorRpmPerMps, kTol);
  EXPECT_NEAR(rpm.right, 0.65 * kMotorRpmPerMps, kTol);
}

TEST(Conversion, GearRatioScalesMotorRpmButNotWheelRpm)
{
  const double wheel_rpm = rov_motor_simulator::wheel_rpm_from_speed(1.0, 0.05);
  EXPECT_NEAR(wheel_rpm, 190.9859, 1e-3);

  MotorParams direct = defaults();
  direct.gear_ratio = 1.0;
  const MotorRpm rpm = rov_motor_simulator::velocity_to_target_rpm(1.0, 0.0, direct);
  EXPECT_NEAR(rpm.left, wheel_rpm, kTol);

  MotorParams geared = defaults();
  geared.gear_ratio = 4.0;
  const MotorRpm geared_rpm = rov_motor_simulator::velocity_to_target_rpm(1.0, 0.0, geared);
  EXPECT_NEAR(geared_rpm.left, 4.0 * wheel_rpm, kTol);
}

TEST(Saturation, LinearCommandIsClampedToMaxRpm)
{
  const MotorRpm fwd = rov_motor_simulator::velocity_to_target_rpm(5.0, 0.0, defaults());
  EXPECT_DOUBLE_EQ(fwd.left, 3000.0);
  EXPECT_DOUBLE_EQ(fwd.right, 3000.0);

  const MotorRpm rev = rov_motor_simulator::velocity_to_target_rpm(-5.0, 0.0, defaults());
  EXPECT_DOUBLE_EQ(rev.left, -3000.0);
  EXPECT_DOUBLE_EQ(rev.right, -3000.0);
}

TEST(Saturation, TurningRatioIsPreservedWhenSaturating)
{
  // Unsaturated: left = 0.5 m/s, right = 1.5 m/s -> ratio 1:3
  const MotorRpm rpm = rov_motor_simulator::velocity_to_target_rpm(1.0, 3.3333333, defaults());
  const MotorRpm big = rov_motor_simulator::velocity_to_target_rpm(2.0, 6.6666666, defaults());
  EXPECT_NEAR(std::fabs(big.right), 3000.0, kTol);
  EXPECT_NEAR(big.left / big.right, rpm.left / rpm.right, 1e-6);
  EXPECT_LE(std::fabs(big.left), 3000.0);
}

TEST(Saturation, SaturationFollowsConfiguredLimit)
{
  MotorParams p = defaults();
  p.max_motor_rpm = 500.0;
  const MotorRpm rpm = rov_motor_simulator::velocity_to_target_rpm(1.0, 0.0, p);
  EXPECT_DOUBLE_EQ(rpm.left, 500.0);
  EXPECT_DOUBLE_EQ(rpm.right, 500.0);
}

TEST(Saturation, NonFiniteResultGivesZeroTarget)
{
  const MotorRpm huge = rov_motor_simulator::velocity_to_target_rpm(1e308, 0.0, defaults());
  EXPECT_DOUBLE_EQ(huge.left, 0.0);
  EXPECT_DOUBLE_EQ(huge.right, 0.0);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const MotorRpm bad = rov_motor_simulator::velocity_to_target_rpm(nan, 0.0, defaults());
  EXPECT_DOUBLE_EQ(bad.left, 0.0);
  EXPECT_DOUBLE_EQ(bad.right, 0.0);
}

TEST(Response, StepNeverOvershootsTarget)
{
  EXPECT_DOUBLE_EQ(rov_motor_simulator::step_toward(95.0, 100.0, 30.0), 100.0);
  EXPECT_DOUBLE_EQ(rov_motor_simulator::step_toward(-95.0, -100.0, 30.0), -100.0);
  EXPECT_DOUBLE_EQ(rov_motor_simulator::step_toward(100.0, 100.0, 30.0), 100.0);
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

TEST(Response, ReachesTargetInExpectedNumberOfTicks)
{
  double rpm = 0.0;
  const double target = 954.9296;
  int ticks = 0;
  while (rpm != target && ticks < 1000) {
    rpm = rov_motor_simulator::update_motor_rpm(rpm, target, 1500.0, 0.02);
    ++ticks;
  }
  EXPECT_EQ(ticks, 32);  // ceil(954.9296 / 30)
  EXPECT_DOUBLE_EQ(rpm, target);
}

TEST(Response, RampsDownToZeroAndNeverGoesNegative)
{
  double rpm = 100.0;
  for (int tick = 0; tick < 10; ++tick) {
    const double previous = rpm;
    rpm = rov_motor_simulator::update_motor_rpm(rpm, 0.0, 1500.0, 0.02);
    EXPECT_LE(rpm, previous);
    EXPECT_GE(rpm, 0.0);
  }
  EXPECT_DOUBLE_EQ(rpm, 0.0);
}

TEST(Response, SuddenReversalPassesThroughZeroWithLimitedSteps)
{
  double rpm = 900.0;
  const double target = -900.0;
  int ticks = 0;
  bool crossed_zero = false;
  while (rpm != target && ticks < 1000) {
    const double previous = rpm;
    rpm = rov_motor_simulator::update_motor_rpm(rpm, target, 1500.0, 0.02);
    EXPECT_LE(std::fabs(rpm - previous), 30.0 + 1e-9);
    if (previous > 0.0 && rpm <= 0.0) {crossed_zero = true;}
    ++ticks;
  }
  EXPECT_TRUE(crossed_zero);
  EXPECT_EQ(ticks, 60);  // 1800 RPM at 30 RPM per tick
}

TEST(Timeout, NoCommandEverReceivedIsStale)
{
  EXPECT_TRUE(rov_motor_simulator::command_is_stale(false, 0.0, 0.5));
}

TEST(Timeout, FreshCommandIsNotStale)
{
  EXPECT_FALSE(rov_motor_simulator::command_is_stale(true, 0.1, 0.5));
  EXPECT_FALSE(rov_motor_simulator::command_is_stale(true, 0.5, 0.5));
}

TEST(Timeout, OldCommandIsStale)
{
  EXPECT_TRUE(rov_motor_simulator::command_is_stale(true, 0.51, 0.5));
}

TEST(Parameters, DefaultsAreValid)
{
  EXPECT_EQ(rov_motor_simulator::validate(defaults()), "");
}

TEST(Parameters, RejectsZeroNegativeAndNonFiniteValues)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  MotorParams p = defaults();
  p.wheel_radius_m = 0.0;
  EXPECT_NE(rov_motor_simulator::validate(p), "");

  p = defaults();
  p.track_width_m = -0.3;
  EXPECT_NE(rov_motor_simulator::validate(p), "");

  p = defaults();
  p.gear_ratio = nan;
  EXPECT_NE(rov_motor_simulator::validate(p), "");

  p = defaults();
  p.max_motor_rpm = inf;
  EXPECT_NE(rov_motor_simulator::validate(p), "");

  p = defaults();
  p.max_acceleration_rpm_s = 0.0;
  EXPECT_NE(rov_motor_simulator::validate(p), "");

  p = defaults();
  p.command_timeout_s = -1.0;
  EXPECT_NE(rov_motor_simulator::validate(p), "");
}

TEST(Parameters, RejectsUpdateRateOutOfRange)
{
  MotorParams p = defaults();
  p.update_rate_hz = 0.0;
  EXPECT_NE(rov_motor_simulator::validate(p), "");
  p.update_rate_hz = 5000.0;
  EXPECT_NE(rov_motor_simulator::validate(p), "");
  p.update_rate_hz = 1000.0;
  EXPECT_EQ(rov_motor_simulator::validate(p), "");
}
