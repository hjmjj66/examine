#include <cmath>

#include <gtest/gtest.h>

#include "aim_armor_controller/armor_selection_policy.hpp"
#include "aim_armor_controller/outpost_fire_gate.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

aim_armor_controller::ArmorSelectionConfig makeConfig()
{
  aim_armor_controller::ArmorSelectionConfig config;
  config.coming_angle_rad = 60.0 * kPi / 180.0;
  config.leaving_angle_rad = 20.0 * kPi / 180.0;
  config.response_speed_rad_s = 0.01;
  config.min_angular_velocity_rad_s = 0.6;
  config.min_switch_angle_rad = 30.0 * kPi / 180.0;
  return config;
}

}  // namespace

TEST(ArmorSelectionPolicy, PositiveRotationSelectsPreviousIndex)
{
  auto config = makeConfig();
  aim_armor_controller::ArmorSelectionInput input;
  input.candidates = {{0.70, true}, {-0.87, true}, {-2.44, true}, {0.70 - kPi / 2.0, true}};
  input.locked_index = 0;
  input.angular_velocity = 1.0;
  input.radius = 0.2765;
  input.distance = 1.0;

  const auto result = aim_armor_controller::selectArmorIndex(input, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.index, 3);
  EXPECT_TRUE(result.switched);
}

TEST(ArmorSelectionPolicy, NegativeRotationSelectsNextIndex)
{
  auto config = makeConfig();
  aim_armor_controller::ArmorSelectionInput input;
  input.candidates = {{-0.70, true}, {0.87, true}, {2.44, true}, {-0.70 + kPi / 2.0, true}};
  input.locked_index = 0;
  input.angular_velocity = -1.0;
  input.radius = 0.2765;
  input.distance = 1.0;

  const auto result = aim_armor_controller::selectArmorIndex(input, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.index, 1);
  EXPECT_TRUE(result.switched);
}

TEST(ArmorSelectionPolicy, JumpAngleUsesDistanceRadiusAndAngularVelocity)
{
  const auto config = makeConfig();
  const double near_angle =
    aim_armor_controller::computeArmorJumpAngle(1.0, 0.3, 1.0, config);
  const double far_angle =
    aim_armor_controller::computeArmorJumpAngle(100.0, 0.3, 1.0, config);
  const double fast_angle =
    aim_armor_controller::computeArmorJumpAngle(100.0, 0.3, 2.0, config);

  EXPECT_GE(near_angle, config.min_switch_angle_rad);
  EXPECT_GT(far_angle, near_angle);
  EXPECT_LT(fast_angle, far_angle);
}

TEST(ArmorSelectionPolicy, LowSpeedKeepsTheLock)
{
  auto config = makeConfig();
  aim_armor_controller::ArmorSelectionInput input;
  input.candidates = {{0.2, true}, {-0.2, true}, {2.9, true}, {-2.9, true}};
  input.locked_index = 1;
  input.angular_velocity = 0.3;
  input.radius = 0.3;
  input.distance = 2.0;

  const auto result = aim_armor_controller::selectArmorIndex(input, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.index, 1);
  EXPECT_FALSE(result.switched);
}

TEST(ArmorSelectionPolicy, ConvergedOutpostCanUsePredictedSlot)
{
  auto config = makeConfig();
  aim_armor_controller::ArmorSelectionInput input;
  input.is_outpost = true;
  input.tracking_converged = true;
  input.candidates = {{1.8, true}, {-2.39, false}, {-0.29, false}};
  input.locked_index = 0;
  input.angular_velocity = 1.0;
  input.radius = 0.2765;
  input.distance = 1.0;

  const auto result = aim_armor_controller::selectArmorIndex(input, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.index, 2);
}


TEST(ArmorSelectionPolicy, ConvergedOutpostCanSwitchToUnobservedIncomingSlot)
{
  auto config = makeConfig();
  aim_armor_controller::ArmorSelectionInput input;
  input.is_outpost = true;
  input.tracking_converged = true;
  input.candidates = {{0.7, true}, {-2.39, false}, {-0.8, false}};
  input.locked_index = 0;
  input.angular_velocity = 1.0;
  input.radius = 0.2765;
  input.distance = 1.0;

  const auto result = aim_armor_controller::selectArmorIndex(input, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.index, 2);
  EXPECT_TRUE(result.switched);
}

TEST(ArmorSelectionPolicy, UnconvergedOutpostUsesObservedFallback)
{
  auto config = makeConfig();
  aim_armor_controller::ArmorSelectionInput input;
  input.is_outpost = true;
  input.tracking_converged = false;
  input.candidates = {{0.7, true}, {-0.8, false}, {-0.2, false}};
  input.locked_index = 0;
  input.angular_velocity = 1.0;
  input.radius = 0.2765;
  input.distance = 1.0;

  const auto result = aim_armor_controller::selectArmorIndex(input, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.index, 0);
}

TEST(ArmorSelectionPolicy, UnavailableIncomingSlotFallsBackToCurrent)
{
  auto config = makeConfig();
  aim_armor_controller::ArmorSelectionInput input;
  input.is_outpost = true;
  input.tracking_converged = false;
  input.candidates = {{0.7, true}, {-0.8, false}, {-2.2, false}};
  input.locked_index = 0;
  input.angular_velocity = 1.0;
  input.radius = 0.2765;
  input.distance = 1.0;

  const auto result = aim_armor_controller::selectArmorIndex(input, config);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.index, 0);
}

TEST(ArmorSelectionPolicy, FireGateRemainsSeparateFromAimingSelection)
{
  const double gate = 60.0 * kPi / 180.0;
  EXPECT_TRUE(aim_armor_controller::allowOutpostFireByFacingGate(-1.0, 0.0, 0.0, gate));
  EXPECT_FALSE(aim_armor_controller::allowOutpostFireByFacingGate(-1.0, 0.0, kPi, gate));
}
