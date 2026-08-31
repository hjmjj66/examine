#include <gtest/gtest.h>

#include "aim_solver/aim_solver_node.hpp"
#include "aim_msgs/msg/armor_pose_set.hpp"

TEST(PoseObservationForwarding, CopiesSourceFieldsAndBothPoses)
{
  aim_msgs::msg::Armor source;
  source.corners[0].x = 123.5;
  source.corners[0].y = 67.25;
  source.armor_class.class_id = 3U;

  geometry_msgs::msg::Pose camera_pose;
  camera_pose.position.z = 2.5;
  camera_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose world_pose;
  world_pose.position.x = -4.0;
  world_pose.orientation.z = 0.5;
  world_pose.orientation.w = 0.5;

  const auto observation = aim_solver::makeArmorPoseObservation(
    source, camera_pose, world_pose);

  EXPECT_DOUBLE_EQ(observation.corners[0].x, source.corners[0].x);
  EXPECT_DOUBLE_EQ(observation.corners[0].y, source.corners[0].y);
  EXPECT_EQ(observation.armor_class.class_id, source.armor_class.class_id);
  EXPECT_DOUBLE_EQ(observation.camera_pose.position.z, camera_pose.position.z);
  EXPECT_DOUBLE_EQ(observation.camera_pose.orientation.w, camera_pose.orientation.w);
  EXPECT_DOUBLE_EQ(observation.pose.position.x, world_pose.position.x);
  EXPECT_DOUBLE_EQ(observation.pose.orientation.z, world_pose.orientation.z);
  EXPECT_DOUBLE_EQ(observation.pose.orientation.w, world_pose.orientation.w);

  aim_msgs::msg::ArmorPoseSet pose_set;
  // Deprecated compatibility field; new consumers use observations.
  pose_set.armor_poses.push_back(world_pose);
  pose_set.observations.push_back(observation);
  EXPECT_EQ(pose_set.armor_poses.size(), 1U);
  EXPECT_EQ(pose_set.observations.size(), 1U);
}
