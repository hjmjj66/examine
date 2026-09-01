#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gtsam/geometry/Pose3.h>

#include "aim_msgs/msg/armor_pose_observation.hpp"

namespace tracker
{

enum class CameraSource
{
  Front0,
  Front1,
  Back
};

struct ArmorObservation
{
  std::uint8_t target_id{0};
  CameraSource source{CameraSource::Front0};
  builtin_interfaces::msg::Time stamp{};
  geometry_msgs::msg::Pose world_pose{};
  geometry_msgs::msg::Pose camera_pose{};
  gtsam::Pose3 camera_to_world{};
  std::array<geometry_msgs::msg::Point, 4> corners{};
  aim_msgs::msg::ArmorClass armor_class{};
};

inline ArmorObservation fromRosObservation(
  std::uint8_t target_id,
  CameraSource source,
  const builtin_interfaces::msg::Time & stamp,
  const aim_msgs::msg::ArmorPoseObservation & message)
{
  ArmorObservation observation;
  observation.target_id = target_id;
  observation.source = source;
  observation.stamp = stamp;
  observation.world_pose = message.pose;
  observation.camera_pose = message.camera_pose;
  const double quaternion_norm = std::hypot(
    std::hypot(message.camera_to_world.rotation.x, message.camera_to_world.rotation.y),
    std::hypot(message.camera_to_world.rotation.z, message.camera_to_world.rotation.w));
  if (std::isfinite(quaternion_norm) && quaternion_norm > 1e-9) {
    observation.camera_to_world = gtsam::Pose3(
      gtsam::Rot3::Quaternion(
        message.camera_to_world.rotation.w,
        message.camera_to_world.rotation.x,
        message.camera_to_world.rotation.y,
        message.camera_to_world.rotation.z),
      gtsam::Point3(
        message.camera_to_world.translation.x,
        message.camera_to_world.translation.y,
        message.camera_to_world.translation.z));
  }
  observation.corners = message.corners;
  observation.armor_class = message.armor_class;
  return observation;
}

}  // namespace tracker
