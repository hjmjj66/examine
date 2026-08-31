#include <cstddef>

#include <gtest/gtest.h>

#include "aim_solver/pose_observation.hpp"

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
  ASSERT_TRUE(aim_solver::appendArmorPoseObservation(
    pose_set, source, camera_pose, &world_pose));
  EXPECT_EQ(pose_set.armor_poses.size(), 1U);
  EXPECT_EQ(pose_set.observations.size(), 1U);

  // Deprecated compatibility field; observations is the tracker interface.
  ASSERT_EQ(pose_set.armor_poses.size(), pose_set.observations.size());
  for (std::size_t i = 0; i < pose_set.armor_poses.size(); ++i) {
    const auto & legacy_pose = pose_set.armor_poses[i];
    const auto & observation_pose = pose_set.observations[i].pose;
    EXPECT_DOUBLE_EQ(legacy_pose.position.x, observation_pose.position.x);
    EXPECT_DOUBLE_EQ(legacy_pose.position.y, observation_pose.position.y);
    EXPECT_DOUBLE_EQ(legacy_pose.position.z, observation_pose.position.z);
    EXPECT_DOUBLE_EQ(legacy_pose.orientation.x, observation_pose.orientation.x);
    EXPECT_DOUBLE_EQ(legacy_pose.orientation.y, observation_pose.orientation.y);
    EXPECT_DOUBLE_EQ(legacy_pose.orientation.z, observation_pose.orientation.z);
    EXPECT_DOUBLE_EQ(legacy_pose.orientation.w, observation_pose.orientation.w);
  }
}

TEST(PoseObservationForwarding, LeavesBothArraysUnchangedWhenSkipped)
{
  aim_msgs::msg::Armor source;
  geometry_msgs::msg::Pose camera_pose;
  aim_msgs::msg::ArmorPoseSet pose_set;

  const auto armor_pose_count = pose_set.armor_poses.size();
  const auto observation_count = pose_set.observations.size();

  // A failed solve/transform is skipped; neither compatibility nor tracker data is written.
  EXPECT_FALSE(aim_solver::appendArmorPoseObservation(
    pose_set, source, camera_pose, nullptr));
  EXPECT_EQ(pose_set.armor_poses.size(), armor_pose_count);
  EXPECT_EQ(pose_set.observations.size(), observation_count);
}
