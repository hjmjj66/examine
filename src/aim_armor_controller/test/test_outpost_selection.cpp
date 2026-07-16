#include <cmath>

#include <gtest/gtest.h>

#include "aim_armor_controller/legacy_target_model.hpp"
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

TEST(FrontCameraArbitrator, DisabledFallbackNeverProcessesFront1)
{
  using Arbitrator = aim_outpost_predictor::FrontCameraArbitrator;
  const auto start = Arbitrator::TimePoint{};
  Arbitrator arbitrator(0.2, start);
  arbitrator.setFallbackEnabled(false);

  EXPECT_FALSE(arbitrator.shouldProcessFront1(start + std::chrono::seconds(10)));
}

