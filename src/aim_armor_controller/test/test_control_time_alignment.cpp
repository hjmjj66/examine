#include <cstdint>

#include <gtest/gtest.h>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>

#include "aim_armor_controller/control_time_alignment.hpp"
#include "aim_armor_controller/command_rate_limiter.hpp"
#include "aim_armor_controller/outpost_tracking_hold.hpp"

namespace
{

builtin_interfaces::msg::Time makeStamp(std::int32_t sec, std::uint32_t nanosec = 0U)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

TEST(ControlTimeAlignment, UsesNonzeroGimbalFeedbackStamp)
{
  const auto gimbal_stamp = makeStamp(42, 123U);
  const rclcpp::Time fallback_now(99, 0, RCL_ROS_TIME);

  EXPECT_EQ(
    aim_armor_controller::resolveControlStamp(gimbal_stamp, fallback_now),
    rclcpp::Time(gimbal_stamp));
}

TEST(ControlTimeAlignment, UsesFallbackWhenGimbalFeedbackStampIsZero)
{
  const builtin_interfaces::msg::Time zero_stamp{};
  const rclcpp::Time fallback_now(99, 456U, RCL_ROS_TIME);

  EXPECT_EQ(
    aim_armor_controller::resolveControlStamp(zero_stamp, fallback_now),
    fallback_now);
}

TEST(ControlTimeAlignment, ComputesMeasurementAgeAtControlTime)
{
  const auto measurement_stamp = makeStamp(10, 250000000U);
  const rclcpp::Time control_stamp(10, 750000000U, RCL_ROS_TIME);

  EXPECT_DOUBLE_EQ(
    aim_armor_controller::ageAtControlStamp(measurement_stamp, control_stamp),
    0.5);
}

TEST(ControlTimeAlignment, ClampsFutureMeasurementAgeToZero)
{
  const auto measurement_stamp = makeStamp(11);
  const rclcpp::Time control_stamp(10, 0, RCL_ROS_TIME);

  EXPECT_DOUBLE_EQ(
    aim_armor_controller::ageAtControlStamp(measurement_stamp, control_stamp),
    0.0);
}

TEST(OutpostTrackingHold, HoldsOnlyCachedSelectedOutpostWithinTimeout)
{
  EXPECT_TRUE(aim_armor_controller::shouldHoldOutpostTarget(true, true, 0.15, 0.15));
  EXPECT_FALSE(aim_armor_controller::shouldHoldOutpostTarget(false, true, 0.01, 0.15));
  EXPECT_FALSE(aim_armor_controller::shouldHoldOutpostTarget(true, false, 0.01, 0.15));
  EXPECT_FALSE(aim_armor_controller::shouldHoldOutpostTarget(true, true, 0.151, 0.15));
}

TEST(CommandRateLimiter, LimitsYawAndPitchPerSecond)
{
  aim_armor_controller::CommandRateLimiter limiter;
  limiter.setMaxRateDegPerSec(60.0);
  limiter.reset(0.0, 0.0);

  const auto result = limiter.update(90.0, -30.0, 0.1);
  EXPECT_DOUBLE_EQ(result.yaw_deg, 6.0);
  EXPECT_DOUBLE_EQ(result.pitch_deg, -6.0);
}

TEST(CommandRateLimiter, UsesShortestYawPathAcrossWrap)
{
  aim_armor_controller::CommandRateLimiter limiter;
  limiter.setMaxRateDegPerSec(60.0);
  limiter.reset(179.0, 0.0);

  const auto result = limiter.update(-179.0, 0.0, 0.1);
  EXPECT_DOUBLE_EQ(result.yaw_deg, 181.0);
}

}  // namespace
