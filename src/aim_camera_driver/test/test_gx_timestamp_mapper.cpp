#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "aim_camera_driver/gx_timestamp_mapper.hpp"

TEST(GxTimestampMapper, MapsRelativeDeviceTicksToRosTime)
{
  aim_camera_driver::GxTimestampMapper mapper({1'000'000.0, 0.002, 0.1, 0.02});
  const rclcpp::Time anchor(10, 0, RCL_ROS_TIME);
  ASSERT_TRUE(mapper.initialize(1'000'000U, anchor));

  const auto mapped = mapper.map(1'500'000U, anchor);
  EXPECT_EQ(mapped.nanoseconds(), 10'502'000'000LL);
}

TEST(GxTimestampMapper, RepeatedLatchCorrectsHostDeviceOffset)
{
  aim_camera_driver::GxTimestampMapper mapper({1'000'000.0, 0.0, 1.0, 0.05});
  ASSERT_TRUE(mapper.initialize(1'000'000U, rclcpp::Time(10, 0, RCL_ROS_TIME)));
  ASSERT_TRUE(mapper.updateLatch(2'000'000U, rclcpp::Time(11, 0, RCL_ROS_TIME)));

  EXPECT_NEAR(mapper.estimatedLatchCorrectionSec(), 0.0, 1e-12);
  ASSERT_TRUE(
    mapper.updateLatch(
      3'000'000U,
      rclcpp::Time(12'010'000'000LL, RCL_ROS_TIME)));
  EXPECT_NEAR(mapper.estimatedLatchCorrectionSec(), 0.01, 1e-9);
}

TEST(GxTimestampMapper, FallsBackOnTickRollback)
{
  aim_camera_driver::GxTimestampMapper mapper({1'000'000.0, 0.0, 0.1, 0.02});
  ASSERT_TRUE(mapper.initialize(100U, rclcpp::Time(10, 0, RCL_ROS_TIME)));
  const rclcpp::Time fallback(99, 0, RCL_ROS_TIME);

  EXPECT_EQ(mapper.map(50U, fallback), fallback);
}
