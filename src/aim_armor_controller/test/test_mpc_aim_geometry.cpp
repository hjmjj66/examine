#include <cassert>
#include <string>

#include "aim_armor_controller/mpc_aim_geometry.hpp"

namespace
{

bool passthroughTransform(
  const geometry_msgs::msg::PointStamped & source,
  const geometry_msgs::msg::TransformStamped &,
  geometry_msgs::msg::PointStamped & target)
{
  target = source;
  return true;
}

aim_armor_controller::TargetModel makeTarget()
{
  aim_armor_controller::TargetModel target;
  target.header.frame_id = "gimbal_world";
  target.header.stamp.sec = 1;
  target.center.x = 5.0;
  target.center.y = 0.0;
  target.center.z = 0.0;
  target.radius = 0.25;
  target.armor_count = 4;
  return target;
}

void testReportsInvalidAimCandidate()
{
  auto target = makeTarget();
  target.armor_count = 3;
  target.visible_slots = {false, false, false};
  double lock_index = -1.0;
  aim_armor_controller::AimComputationConfig config;
  geometry_msgs::msg::TransformStamped world_to_gun;

  const auto result = aim_armor_controller::computeAimAngles(
    target, "gimbal_world", "gimbal_world", nullptr, world_to_gun, rclcpp::Time(target.header.stamp),
    23.0, 0.0, lock_index, config, passthroughTransform);

  assert(!result.valid);
  assert(result.failure_reason.find("aim_candidate_invalid") != std::string::npos);
  assert(result.failure_reason.find("visible_slots=[0,0,0]") != std::string::npos);
}

void testReportsTransformFailure()
{
  auto target = makeTarget();
  double lock_index = -1.0;
  aim_armor_controller::AimComputationConfig config;
  geometry_msgs::msg::TransformStamped world_to_gun;

  const auto result = aim_armor_controller::computeAimAngles(
    target, "target_frame", "gimbal_world", nullptr, world_to_gun, rclcpp::Time(target.header.stamp),
    23.0, 0.0, lock_index, config, passthroughTransform);

  assert(!result.valid);
  assert(result.failure_reason.find("aim_point_transform_failed") != std::string::npos);
  assert(result.failure_reason.find("source_frame=target_frame") != std::string::npos);
}

void testReportsUnsolvableTrajectory()
{
  auto target = makeTarget();
  double lock_index = -1.0;
  aim_armor_controller::AimComputationConfig config;
  geometry_msgs::msg::TransformStamped world_to_gun;

  const auto result = aim_armor_controller::computeAimAngles(
    target, "gimbal_world", "gimbal_world", nullptr, world_to_gun, rclcpp::Time(target.header.stamp),
    0.0, 0.0, lock_index, config, passthroughTransform);

  assert(!result.valid);
  assert(result.failure_reason.find("trajectory_unsolvable") != std::string::npos);
  assert(result.failure_reason.find("solver_reason=bullet_speed_non_positive") !=
    std::string::npos);
  assert(result.failure_reason.find("bullet_speed=0.000") != std::string::npos);
}

}  // namespace

int main()
{
  testReportsInvalidAimCandidate();
  testReportsTransformFailure();
  testReportsUnsolvableTrajectory();
  return 0;
}
