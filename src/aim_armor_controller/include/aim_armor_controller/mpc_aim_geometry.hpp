#pragma once

#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/time.hpp>

#include "aim_armor_controller/mpc_planner.hpp"
#include "aim_armor_controller/mpc_target_model.hpp"
#include "aim_armor_controller/mpc_trajectory_solver.hpp"

namespace aim_armor_controller
{

struct AimAngles
{
  bool valid{false};
  std::string failure_reason;
  double yaw_rad{0.0};
  double pitch_rad{0.0};
  MpcTrajectorySolution trajectory;
  AimCandidate candidate;
  TargetModel predicted_target;
};

struct AimComputationConfig
{
  bool use_air_resistance{true};
  double yaw_offset_rad{0.0};
  double pitch_offset_rad{0.0};
  double muzzle_offset_x_m{0.0};
  geometry_msgs::msg::Vector3 target_offset;
  AimSelectionConfig selection;
};

struct GimbalStateSample
{
  rclcpp::Time stamp;
  double yaw_rad{0.0};
  double pitch_rad{0.0};
  double yaw_velocity_rad_s{0.0};
  double pitch_velocity_rad_s{0.0};
};

using TransformPointFn = bool (*)(
  const geometry_msgs::msg::PointStamped &,
  const geometry_msgs::msg::TransformStamped &,
  geometry_msgs::msg::PointStamped &);

AimAngles computeAimAngles(
  const TargetModel & target,
  const std::string & target_frame_id,
  const std::string & world_frame_id,
  const geometry_msgs::msg::TransformStamped * target_to_world,
  const geometry_msgs::msg::TransformStamped & world_to_gun,
  const rclcpp::Time & measurement_time,
  double bullet_speed,
  double delay_time,
  double & lock_index,
  const AimComputationConfig & config,
  TransformPointFn transform_point);

std::optional<MpcReference> buildMpcReference(
  const TargetModel & base_target,
  const std::string & target_frame_id,
  const std::string & world_frame_id,
  const geometry_msgs::msg::TransformStamped * target_to_world,
  const geometry_msgs::msg::TransformStamped & world_to_gun,
  const rclcpp::Time & measurement_time,
  double bullet_speed,
  double base_delay_time,
  double dt,
  int horizon,
  double yaw0,
  double & lock_index,
  const AimComputationConfig & config,
  TransformPointFn transform_point,
  const std::vector<GimbalStateSample> & gimbal_state_history = {});

}  // namespace aim_armor_controller
