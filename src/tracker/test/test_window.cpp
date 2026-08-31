#include "tracker/target_tracker.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace
{

builtin_interfaces::msg::Time stamp(std::int32_t seconds)
{
  builtin_interfaces::msg::Time result;
  result.sec = seconds;
  return result;
}
tracker::ArmorObservation observation(std::int32_t seconds)
{
  tracker::ArmorObservation result;
  result.target_id = 2;
  result.source = tracker::CameraSource::Front0;
  result.stamp = stamp(seconds);
  result.world_pose.position.x = 0.0;
  result.world_pose.position.y = 0.0;
  result.world_pose.position.z = 1.0;
  result.camera_pose = result.world_pose;
  result.world_pose.orientation.w = 1.0;
  result.camera_pose.orientation.w = 1.0;
  result.armor_class.class_id = 0;
  return result;
}

}  // namespace

TEST(TargetTrackerWindow, RetainsExactlyTheNewestThirtyFramesAfterThirtyOneInputs)
{
  tracker::TrackerConfig config;
  config.window_size = 30;
  tracker::TargetTracker target(config);
  ASSERT_TRUE(target.initialize(observation(0)));

  for (std::int32_t seconds = 1; seconds <= 30; ++seconds) {
    ASSERT_TRUE(target.addMeasurement(observation(seconds)));
  }

  ASSERT_EQ(target.frameCount(), 30U);
  ASSERT_TRUE(target.optimize());
  EXPECT_EQ(target.frameCount(), 30U);
  ASSERT_TRUE(target.oldestFrameStamp().has_value());
  ASSERT_TRUE(target.latestFrameStamp().has_value());
  EXPECT_EQ(target.oldestFrameStamp()->sec, 1);
  EXPECT_EQ(target.latestFrameStamp()->sec, 30);
  EXPECT_TRUE(target.allFactorKeysHaveValues());
}
