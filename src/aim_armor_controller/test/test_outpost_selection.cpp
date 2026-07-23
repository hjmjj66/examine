#include <cmath>

#include <gtest/gtest.h>

#include "aim_armor_controller/legacy_target_model.hpp"
#include "aim_armor_controller/target_state_selection.hpp"
#include "aim_armor_controller/mpc_target_model.hpp"
#include "aim_outpost_predictor/front_camera_arbitrator.hpp"

namespace
{

aim_armor_controller::LegacyTargetModel makeOutpostTarget(double yaw)
{
  aim_armor_controller::LegacyTargetModel target;
  target.center.x = 1.0;
  target.center.y = 0.0;
  target.center.z = 0.0;
  target.yaw = yaw;
  target.radius = 0.2765;
  target.armor_count = 3;
  return target;
}

aim_armor_controller::TargetModel makeMpcOutpostTarget(double yaw)
{
  aim_armor_controller::TargetModel target;
  target.center.x = 1.0;
  target.center.y = 0.0;
  target.center.z = 0.0;
  target.yaw = yaw;
  target.radius = 0.2765;
  target.armor_count = 3;
  return target;
}
aim_armor_controller::TargetModel makeMpcNormalTarget(
  double yaw, double angular_velocity, int armor_count = 4)
{
  auto target = makeMpcOutpostTarget(yaw);
  target.armor_count = armor_count;
  target.angular_velocity = angular_velocity;
  target.jumped = true;
  target.visible_slots = {true, true, true};
  return target;
}


TEST(TargetStateSelection, RequiresTrackingAndConvergenceForControl)
{
  aim_msgs::msg::TargetState target;
  target.tracking = true;
  target.converged = true;
  EXPECT_TRUE(aim_armor_controller::isTargetStateReadyForControl(target));

  target.tracking = false;
  EXPECT_FALSE(aim_armor_controller::isTargetStateReadyForControl(target));

  target.tracking = true;
  target.converged = false;
  EXPECT_FALSE(aim_armor_controller::isTargetStateReadyForControl(target));
}

}  // namespace

TEST(OutpostSelection, SelectsVisibleSlotThatFacesGun)
{
  auto target = makeOutpostTarget(-2.0 * aim_armor_controller::kPi / 3.0);
  target.visible_slots = {false, true, false};

  const auto selected = aim_armor_controller::chooseLegacyOutpostArmorIndex(
    target, 45.0 * aim_armor_controller::kPi / 180.0);

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(*selected, 1);
}

TEST(OutpostSelection, RejectsInvisibleSlotEvenWhenItFacesGun)
{
  auto target = makeOutpostTarget(0.0);
  target.visible_slots = {false, true, false};

  const auto selected = aim_armor_controller::chooseLegacyOutpostArmorIndex(
    target, 45.0 * aim_armor_controller::kPi / 180.0);

  EXPECT_FALSE(selected.has_value());
}

TEST(OutpostSelection, RejectsWhenNoSlotIsVisible)
{
  auto target = makeOutpostTarget(0.0);

  const auto selected = aim_armor_controller::chooseLegacyOutpostArmorIndex(
    target, 45.0 * aim_armor_controller::kPi / 180.0);

  EXPECT_FALSE(selected.has_value());
}

TEST(OutpostSelection, MpcSelectsOnlyVisibleSlot)
{
  auto target = makeOutpostTarget(0.0);
  target.visible_slots = {false, true, false};
  double lock_index = -1.0;

  const auto selected = aim_armor_controller::chooseMpcAimPoint(
    target, nullptr, lock_index, 2.0);

  ASSERT_TRUE(selected.valid);
  EXPECT_EQ(selected.armor_index, 1);
}

TEST(OutpostSelection, MpcRejectsWhenNoSlotIsVisible)
{
  auto target = makeOutpostTarget(0.0);
  double lock_index = -1.0;

  const auto selected = aim_armor_controller::chooseMpcAimPoint(
    target, nullptr, lock_index, 2.0);

  EXPECT_FALSE(selected.valid);
}

TEST(OutpostSelection, MigratedMpcTargetSelectsOnlyVisibleSlot)
{
  auto target = makeMpcOutpostTarget(0.0);
  target.visible_slots = {false, true, false};
  double lock_index = -1.0;
  aim_armor_controller::AimSelectionConfig config;
  config.low_speed_angular_velocity_threshold = 2.0;

  const auto selected = aim_armor_controller::chooseAimPoint(
    target, lock_index, config);

  ASSERT_TRUE(selected.valid);
  EXPECT_EQ(selected.armor_index, 1);
}

TEST(OutpostSelection, MigratedMpcTargetRejectsWhenNoSlotIsVisible)
{
  auto target = makeMpcOutpostTarget(0.0);
  double lock_index = -1.0;
  aim_armor_controller::AimSelectionConfig config;
  config.low_speed_angular_velocity_threshold = 2.0;

  const auto selected = aim_armor_controller::chooseAimPoint(
    target, lock_index, config);

  EXPECT_FALSE(selected.valid);
}
TEST(ArmorSelection, LowSpeedPositiveRotationSwitchesToIncomingPreviousPlate)
{
  auto target = makeMpcNormalTarget(0.70, 1.0);
  double lock_index = 0.0;
  aim_armor_controller::AimSelectionConfig config;
  config.low_speed_angular_velocity_threshold = 2.0;

  const auto selected = aim_armor_controller::chooseAimPoint(
    target, lock_index, config);

  ASSERT_TRUE(selected.valid);
  EXPECT_EQ(selected.armor_index, 3);
}

TEST(ArmorSelection, LowSpeedNegativeRotationSwitchesToIncomingNextPlate)
{
  auto target = makeMpcNormalTarget(-0.70, -1.0);
  double lock_index = 0.0;
  aim_armor_controller::AimSelectionConfig config;
  config.low_speed_angular_velocity_threshold = 2.0;

  const auto selected = aim_armor_controller::chooseAimPoint(
    target, lock_index, config);

  ASSERT_TRUE(selected.valid);
  EXPECT_EQ(selected.armor_index, 1);
}

TEST(ArmorSelection, OutpostCanSelectPredictedIncomingPlateWithoutCurrentFrameDetection)
{
  auto target = makeMpcOutpostTarget(1.80);
  target.angular_velocity = 1.0;
  target.converged = true;
  target.has_primary_armor = true;
  target.primary_slot = 0;
  target.visible_slots = {true, false, false};
  double lock_index = 0.0;
  aim_armor_controller::AimSelectionConfig config;
  config.low_speed_angular_velocity_threshold = 2.0;

  const auto selected = aim_armor_controller::chooseAimPoint(
    target, lock_index, config);

  ASSERT_TRUE(selected.valid);
  EXPECT_EQ(selected.armor_index, 2);
}



TEST(ArmorSelection, MpcAndLegacyUseTheSameSelectionIndex)
{
  auto mpc_target = makeMpcNormalTarget(0.70, 1.0);
  double mpc_lock_index = 0.0;
  aim_armor_controller::AimSelectionConfig mpc_config;
  const auto mpc_selected = aim_armor_controller::chooseAimPoint(
    mpc_target, mpc_lock_index, mpc_config);

  aim_armor_controller::LegacyTargetModel legacy_target;
  legacy_target.center.x = 1.0;
  legacy_target.center.y = 0.0;
  legacy_target.yaw = 0.70;
  legacy_target.angular_velocity = 1.0;
  legacy_target.radius = 0.2765;
  legacy_target.armor_count = 4;
  double legacy_lock_index = 0.0;
  const auto legacy_selected = aim_armor_controller::chooseLegacyAimPoint(
    legacy_target, nullptr, legacy_lock_index, true, 2.0,
    60.0 * aim_armor_controller::kPi / 180.0,
    20.0 * aim_armor_controller::kPi / 180.0);

  ASSERT_TRUE(mpc_selected.valid);
  ASSERT_TRUE(legacy_selected.valid);
  EXPECT_EQ(mpc_selected.armor_index, legacy_selected.armor_index);
}

TEST(FrontCameraArbitrator, DisabledFallbackNeverProcessesFront1)
{
  using Arbitrator = aim_outpost_predictor::FrontCameraArbitrator;
  const auto start = Arbitrator::TimePoint{};
  Arbitrator arbitrator(0.2, start);
  arbitrator.setFallbackEnabled(false);

  EXPECT_FALSE(arbitrator.shouldProcessFront1(start + std::chrono::seconds(10)));
}

