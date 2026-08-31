#pragma once

#include <array>
#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

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
  observation.corners = message.corners;
  observation.armor_class = message.armor_class;
  return observation;
}

}  // namespace tracker
