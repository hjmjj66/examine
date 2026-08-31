#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include "aim_msgs/msg/armor.hpp"
#include "aim_msgs/msg/armor_pose_observation.hpp"
#include "aim_msgs/msg/armor_pose_set.hpp"

namespace aim_solver
{

inline aim_msgs::msg::ArmorPoseObservation makeArmorPoseObservation(
  const aim_msgs::msg::Armor & source,
  const geometry_msgs::msg::Pose & camera_pose,
  const geometry_msgs::msg::Pose & world_pose)
{
  aim_msgs::msg::ArmorPoseObservation observation;
  observation.pose = world_pose;
  observation.camera_pose = camera_pose;
  observation.corners = source.corners;
  observation.armor_class = source.armor_class;
  return observation;
}

inline bool appendArmorPoseObservation(
  aim_msgs::msg::ArmorPoseSet & pose_set,
  const aim_msgs::msg::Armor & source,
  const geometry_msgs::msg::Pose & camera_pose,
  const geometry_msgs::msg::Pose * world_pose)
{
  if (world_pose == nullptr) {
    return false;
  }

  pose_set.armor_poses.push_back(*world_pose);
  pose_set.observations.push_back(makeArmorPoseObservation(
      source, camera_pose, *world_pose));
  return true;
}

}  // namespace aim_solver
