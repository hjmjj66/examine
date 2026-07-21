#include "aim_armor_controller/mpc_aim_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>

namespace aim_armor_controller
{

namespace
{

bool transformAimPointToWorld(
  const std::string & target_frame_id,
  const std::string & world_frame_id,
  const geometry_msgs::msg::TransformStamped * target_to_world,
  const rclcpp::Time & stamp,
  const AimCandidate & candidate,
  TransformPointFn transform_point,
  geometry_msgs::msg::PointStamped & aim_world)
{
  geometry_msgs::msg::PointStamped aim_source;
  aim_source.header.stamp = stamp;
  aim_source.header.frame_id = target_frame_id;
  aim_source.point = candidate.point_world;

  if (target_frame_id == world_frame_id) {
    aim_world = aim_source;
    aim_world.header.frame_id = world_frame_id;
    return true;
  }

  if (target_to_world == nullptr) {
    return false;
  }
  return transform_point(aim_source, *target_to_world, aim_world);
}

bool interpolateGimbalState(
  const std::vector<GimbalStateSample> & history,
  const rclcpp::Time & stamp,
  GimbalStateSample & sample)
{
  if (history.empty() || stamp < history.front().stamp || stamp > history.back().stamp) {
    return false;
  }

  const auto upper = std::lower_bound(
    history.begin(), history.end(), stamp,
    [](const GimbalStateSample & lhs, const rclcpp::Time & rhs) {
      return lhs.stamp < rhs;
    });
  if (upper == history.begin()) {
    sample = *upper;
    return true;
  }
  if (upper == history.end()) {
    sample = history.back();
    return true;
  }

  const auto & next = *upper;
  const auto & prev = *(upper - 1);
  const double interval_sec = (next.stamp - prev.stamp).seconds();
  if (interval_sec <= 1e-6) {
    sample = next;
    return true;
  }

  const double ratio = std::clamp((stamp - prev.stamp).seconds() / interval_sec, 0.0, 1.0);
  sample.stamp = stamp;
  sample.yaw_rad = mpcLimitRad(prev.yaw_rad + mpcLimitRad(next.yaw_rad - prev.yaw_rad) * ratio);
  sample.pitch_rad = prev.pitch_rad + (next.pitch_rad - prev.pitch_rad) * ratio;
  sample.yaw_velocity_rad_s =
    prev.yaw_velocity_rad_s + (next.yaw_velocity_rad_s - prev.yaw_velocity_rad_s) * ratio;
  sample.pitch_velocity_rad_s =
    prev.pitch_velocity_rad_s + (next.pitch_velocity_rad_s - prev.pitch_velocity_rad_s) * ratio;
  return true;
}

geometry_msgs::msg::Point applyTargetOffset(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::Vector3 & offset)
{
  geometry_msgs::msg::Point adjusted = point;
  adjusted.x += offset.x;
  adjusted.y += offset.y;
  adjusted.z += offset.z;
  return adjusted;
}

std::string formatVisibleSlots(const TargetModel & target)
{
  std::ostringstream out;
  out << '[';
  for (std::size_t i = 0; i < target.visible_slots.size(); ++i) {
    if (i > 0U) {
      out << ',';
    }
    out << (target.visible_slots[i] ? '1' : '0');
  }
  out << ']';
  return out.str();
}

std::string formatAimCandidateFailure(const TargetModel & target)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "aim_candidate_invalid"
      << " id=" << static_cast<int>(target.id)
      << " armor_count=" << target.armor_count
      << " tracking=" << (target.tracking ? "true" : "false")
      << " jumped=" << (target.jumped ? "true" : "false")
      << " center=(" << target.center.x << ',' << target.center.y << ',' << target.center.z << ')'
      << " radius=" << target.radius
      << " angular_velocity=" << target.angular_velocity;
  if (target.armor_count == 3) {
    out << " visible_slots=" << formatVisibleSlots(target);
  }
  return out.str();
}

}  // namespace

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
  TransformPointFn transform_point)
{
  AimAngles result;
  TargetModel predicted_target = target;
  predictTarget(predicted_target, delay_time);

  TargetModel low_speed_selection_target = target;
  predictTarget(low_speed_selection_target, delay_time);

  const AimCandidate candidate =
    chooseAimPoint(predicted_target, lock_index, config.selection, &low_speed_selection_target);
  if (!candidate.valid) {
    result.failure_reason = formatAimCandidateFailure(predicted_target);
    return result;
  }
  AimCandidate adjusted_candidate = candidate;
  adjusted_candidate.point_world =
    applyTargetOffset(candidate.point_world, config.target_offset);

  const rclcpp::Time aim_time = measurement_time + rclcpp::Duration::from_seconds(delay_time);
  geometry_msgs::msg::PointStamped aim_world;
  if (!transformAimPointToWorld(
      target_frame_id, world_frame_id, target_to_world, aim_time, adjusted_candidate, transform_point,
      aim_world))
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(9)
        << "aim_point_transform_failed"
        << " source_frame=" << target_frame_id
        << " world_frame=" << world_frame_id
        << " aim_stamp=" << aim_time.seconds();
    result.failure_reason = out.str();
    return result;
  }

  const double target_x = aim_world.point.x - world_to_gun.transform.translation.x;
  const double target_y = aim_world.point.y - world_to_gun.transform.translation.y;
  const double target_z = aim_world.point.z - world_to_gun.transform.translation.z;
  MpcTrajectorySolution traj =
    solveMpcTrajectory(
      bullet_speed, target_x, target_y, target_z,
      config.use_air_resistance);
  if (traj.unsolvable) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "trajectory_unsolvable"
        << " solver_reason="
        << (traj.failure_reason.empty() ? "unspecified" : traj.failure_reason)
        << " bullet_speed=" << bullet_speed
        << " target_xyz=(" << target_x << ',' << target_y << ',' << target_z << ')'
        << " use_air_resistance=" << (config.use_air_resistance ? "true" : "false");
    result.failure_reason = out.str();
    return result;
  }

  result.valid = true;
  result.yaw_rad = mpcLimitRad(traj.yaw + config.yaw_offset_rad);
  result.pitch_rad = traj.pitch + config.pitch_offset_rad;
  result.trajectory = traj;
  result.candidate = adjusted_candidate;
  result.predicted_target = predicted_target;
  return result;
}

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
  const std::vector<GimbalStateSample> & gimbal_state_history)
{
  if (horizon < 2 || horizon % 2 != 0) {
    return std::nullopt;
  }
  const int half_horizon = horizon / 2;
  const int knot_count = horizon + 1;

  MpcReference reference;
  reference.yaw = Eigen::MatrixXd::Zero(2, knot_count);
  reference.pitch = Eigen::MatrixXd::Zero(2, knot_count);

  std::vector<double> yaw_values(static_cast<std::size_t>(knot_count + 2), 0.0);
  std::vector<double> pitch_values(static_cast<std::size_t>(knot_count + 2), 0.0);

  double reference_lock_index = lock_index;
  std::optional<double> last_valid_yaw;
  std::optional<double> last_valid_pitch;
  for (int i = -1; i <= knot_count; ++i) {
    const double delay = base_delay_time + static_cast<double>(i - half_horizon) * dt;
    const AimAngles aim = computeAimAngles(
      base_target, target_frame_id, world_frame_id, target_to_world, world_to_gun,
      measurement_time, bullet_speed, delay, reference_lock_index, config, transform_point);
    if (!aim.valid) {
      if (!last_valid_yaw.has_value() || !last_valid_pitch.has_value()) {
        return std::nullopt;
      }
      yaw_values[static_cast<std::size_t>(i + 1)] = *last_valid_yaw;
      pitch_values[static_cast<std::size_t>(i + 1)] = *last_valid_pitch;
      continue;
    }
    yaw_values[static_cast<std::size_t>(i + 1)] = aim.yaw_rad;
    pitch_values[static_cast<std::size_t>(i + 1)] = aim.pitch_rad;
    last_valid_yaw = aim.yaw_rad;
    last_valid_pitch = aim.pitch_rad;
  }

  for (int i = 0; i < knot_count; ++i) {
    const double yaw_center = mpcLimitRad(yaw_values[static_cast<std::size_t>(i + 1)] - yaw0);
    const double yaw_velocity =
      mpcLimitRad(
        yaw_values[static_cast<std::size_t>(i + 2)] -
        yaw_values[static_cast<std::size_t>(i)]) /
      (2.0 * dt);
    const double pitch_velocity =
      (pitch_values[static_cast<std::size_t>(i + 2)] -
      pitch_values[static_cast<std::size_t>(i)]) /
      (2.0 * dt);

    reference.yaw.col(i) << yaw_center, yaw_velocity;
    reference.pitch.col(i) << pitch_values[static_cast<std::size_t>(i + 1)], pitch_velocity;
  }

  if (!gimbal_state_history.empty()) {
    const rclcpp::Time current_gimbal_time = gimbal_state_history.back().stamp;
    for (int i = 0; i < half_horizon; ++i) {
      const rclcpp::Time sample_time =
        current_gimbal_time + rclcpp::Duration::from_seconds(static_cast<double>(i - half_horizon) * dt);
      GimbalStateSample sample;
      if (!interpolateGimbalState(gimbal_state_history, sample_time, sample)) {
        continue;
      }

      reference.yaw.col(i) << mpcLimitRad(sample.yaw_rad - yaw0), sample.yaw_velocity_rad_s;
      reference.pitch.col(i) << sample.pitch_rad, sample.pitch_velocity_rad_s;
    }
  }

  return reference;
}

}  // namespace aim_armor_controller
