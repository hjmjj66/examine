#include <cstddef>

#include <gtest/gtest.h>

#include "aim_solver/pose_observation.hpp"

namespace
{

void expectPoseFieldsEqual(
  const geometry_msgs::msg::Pose & expected,
  const geometry_msgs::msg::Pose & actual)
{
  EXPECT_DOUBLE_EQ(expected.position.x, actual.position.x);
  EXPECT_DOUBLE_EQ(expected.position.y, actual.position.y);
  EXPECT_DOUBLE_EQ(expected.position.z, actual.position.z);
  EXPECT_DOUBLE_EQ(expected.orientation.x, actual.orientation.x);
  EXPECT_DOUBLE_EQ(expected.orientation.y, actual.orientation.y);
  EXPECT_DOUBLE_EQ(expected.orientation.z, actual.orientation.z);
  EXPECT_DOUBLE_EQ(expected.orientation.w, actual.orientation.w);
}

}  // namespace

TEST(PoseObservationForwarding, CopiesSourceFieldsAndBothPoses)
{
  aim_msgs::msg::Armor source;
  for (std::size_t i = 0; i < source.corners.size(); ++i) {
    source.corners[i].x = 100.0 + static_cast<double>(i);
    source.corners[i].y = 200.0 + static_cast<double>(i);
    source.corners[i].z = 300.0 + static_cast<double>(i);
  }
  source.armor_class.class_id = 3U;
  source.armor_class.team = 2U;

  geometry_msgs::msg::Pose camera_pose;
  camera_pose.position.x = 1.0;
  camera_pose.position.y = 2.0;
  camera_pose.position.z = 3.0;
  camera_pose.orientation.x = 0.1;
  camera_pose.orientation.y = 0.2;
  camera_pose.orientation.z = 0.3;
  camera_pose.orientation.w = 0.4;

  geometry_msgs::msg::Pose world_pose;
  world_pose.position.x = -4.0;
  world_pose.position.y = -5.0;
  world_pose.position.z = -6.0;
  world_pose.orientation.x = -0.1;
  world_pose.orientation.y = -0.2;
  world_pose.orientation.z = -0.3;
  world_pose.orientation.w = -0.4;

  geometry_msgs::msg::Transform camera_to_world;
  camera_to_world.translation.x = 10.0;
  camera_to_world.translation.y = 11.0;
  camera_to_world.translation.z = 12.0;
  camera_to_world.rotation.z = 0.25;
  camera_to_world.rotation.w = 0.9682458365518543;

  const auto observation = aim_solver::makeArmorPoseObservation(
    source, camera_pose, world_pose, camera_to_world);

  for (std::size_t i = 0; i < source.corners.size(); ++i) {
    EXPECT_DOUBLE_EQ(observation.corners[i].x, source.corners[i].x);
    EXPECT_DOUBLE_EQ(observation.corners[i].y, source.corners[i].y);
    EXPECT_DOUBLE_EQ(observation.corners[i].z, source.corners[i].z);
  }
  EXPECT_EQ(observation.armor_class.class_id, source.armor_class.class_id);
  EXPECT_EQ(observation.armor_class.team, source.armor_class.team);
  expectPoseFieldsEqual(camera_pose, observation.camera_pose);
  expectPoseFieldsEqual(world_pose, observation.pose);
  EXPECT_DOUBLE_EQ(
    observation.camera_to_world.translation.x, camera_to_world.translation.x);
  EXPECT_DOUBLE_EQ(
    observation.camera_to_world.translation.y, camera_to_world.translation.y);
  EXPECT_DOUBLE_EQ(
    observation.camera_to_world.translation.z, camera_to_world.translation.z);
  EXPECT_DOUBLE_EQ(
    observation.camera_to_world.rotation.z, camera_to_world.rotation.z);
  EXPECT_DOUBLE_EQ(
    observation.camera_to_world.rotation.w, camera_to_world.rotation.w);

  aim_msgs::msg::ArmorPoseSet pose_set;
  ASSERT_TRUE(aim_solver::appendArmorPoseObservation(
    pose_set, source, camera_pose, camera_to_world, &world_pose));
  ASSERT_EQ(pose_set.armor_poses.size(), 1U);
  ASSERT_EQ(pose_set.observations.size(), 1U);

  // Deprecated compatibility field; observations is the tracker interface.
  ASSERT_EQ(pose_set.armor_poses.size(), pose_set.observations.size());
  for (std::size_t i = 0; i < pose_set.armor_poses.size(); ++i) {
    expectPoseFieldsEqual(
      pose_set.armor_poses[i], pose_set.observations[i].pose);
  }
}

TEST(PoseObservationForwarding, LeavesBothArraysUnchangedWhenSkipped)
{
  aim_msgs::msg::Armor source;
  geometry_msgs::msg::Pose camera_pose;
  geometry_msgs::msg::Transform camera_to_world;
  camera_to_world.rotation.w = 1.0;
  aim_msgs::msg::ArmorPoseSet pose_set;

  geometry_msgs::msg::Pose solved_pose;
  ASSERT_TRUE(aim_solver::appendArmorPoseObservation(
    pose_set, source, camera_pose, camera_to_world, &solved_pose));
  const auto armor_pose_count = pose_set.armor_poses.size();
  const auto observation_count = pose_set.observations.size();

  // A failed solve/transform is skipped; neither compatibility nor tracker data is written.
  EXPECT_FALSE(aim_solver::appendArmorPoseObservation(
    pose_set, source, camera_pose, camera_to_world, nullptr));
  EXPECT_EQ(pose_set.armor_poses.size(), armor_pose_count);
  EXPECT_EQ(pose_set.observations.size(), observation_count);
}
