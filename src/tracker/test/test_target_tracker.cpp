#include "tracker/target_tracker.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

namespace
{

constexpr double kPi = 3.14159265358979323846;

builtin_interfaces::msg::Time stamp(std::int32_t seconds, std::uint32_t nanoseconds = 0)
{
  builtin_interfaces::msg::Time result;
  result.sec = seconds;
  result.nanosec = nanoseconds;
  return result;
}
tracker::ArmorObservation observation(
  std::uint8_t target_id, const builtin_interfaces::msg::Time & time, double yaw,
  const geometry_msgs::msg::Point & position)
{
  tracker::ArmorObservation result;
  result.target_id = target_id;
  result.source = tracker::CameraSource::Front0;
  result.stamp = time;
  result.world_pose.position = position;
  result.camera_pose.position = position;
  result.world_pose.orientation.z = std::sin(yaw / 2.0);
  result.world_pose.orientation.w = std::cos(yaw / 2.0);
  result.camera_pose.orientation = result.world_pose.orientation;
  result.armor_class.class_id = 0;
  return result;
}

geometry_msgs::msg::Point point(double x, double y, double z)
{
  geometry_msgs::msg::Point result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

}  // namespace

TEST(TargetTracker, InitializesTheDocumentedElevenStateOrder)
{
  tracker::TrackerConfig config;
  config.geometry_initialization.radius = 0.22;
  tracker::TargetTracker target(config);
  const auto input = observation(3, stamp(10), 0.4, point(1.0, 2.0, 3.0));

  ASSERT_TRUE(target.initialize(input));
  ASSERT_EQ(target.state().size(), 11);
  EXPECT_NEAR(target.state()[0], 1.0 + 0.22 * std::cos(0.4), 1e-9);
  EXPECT_DOUBLE_EQ(target.state()[1], 0.0);
  EXPECT_NEAR(target.state()[2], 2.0 + 0.22 * std::sin(0.4), 1e-9);
  EXPECT_DOUBLE_EQ(target.state()[3], 0.0);
  EXPECT_DOUBLE_EQ(target.state()[4], 3.0);
  EXPECT_DOUBLE_EQ(target.state()[5], 0.0);
  EXPECT_NEAR(target.state()[6], 0.4, 1e-9);
  EXPECT_DOUBLE_EQ(target.state()[7], 0.0);
  EXPECT_DOUBLE_EQ(target.state()[8], 0.22);
  EXPECT_DOUBLE_EQ(target.state()[9], config.geometry_initialization.radius_offset);
  EXPECT_DOUBLE_EQ(target.state()[10], config.geometry_initialization.height_offset);
  EXPECT_EQ(target.target_id(), 3);
  EXPECT_TRUE(target.active());
}

TEST(TargetTracker, PredictsConstantVelocityStateWithoutMovingZeroVelocityTarget)
{
  tracker::TargetTracker target;
  ASSERT_TRUE(target.initialize(observation(1, stamp(10), 0.2, point(1.0, 2.0, 3.0))));
  const Eigen::VectorXd before = target.state();

  ASSERT_TRUE(target.predict(stamp(11, 500000000)));

  EXPECT_TRUE(target.state().isApprox(before));
  EXPECT_TRUE(target.acceptTimestamp(stamp(11, 500000000)));
}

TEST(TargetTracker, RejectsOnlyTimestampsOlderThanTheLastAcceptedStamp)
{
  tracker::TargetTracker target;
  ASSERT_TRUE(target.initialize(observation(1, stamp(10), 0.2, point(1.0, 2.0, 3.0))));

  EXPECT_FALSE(target.acceptTimestamp(stamp(9)));
  EXPECT_TRUE(target.acceptTimestamp(stamp(10)));
  EXPECT_FALSE(target.predict(stamp(9)));
  EXPECT_FALSE(target.addMeasurement(observation(1, stamp(9), 0.2, point(1.0, 2.0, 3.0))));
  EXPECT_EQ(target.frameCount(), 1U);
}

TEST(TargetTracker, KeepsYawNormalizedWhenObservationsCrossThePiBranch)
{
  tracker::TargetTracker target;
  const auto first = observation(1, stamp(10), kPi - 0.01, point(-0.22, 0.0, 1.0));
  const auto second = observation(1, stamp(11), -kPi + 0.01, point(-0.22, 0.0, 1.0));
  ASSERT_TRUE(target.initialize(first));
  ASSERT_TRUE(target.addMeasurement(second));
  ASSERT_TRUE(target.optimize());

  EXPECT_GE(target.state()[6], -kPi);
  EXPECT_LE(target.state()[6], kPi);
  EXPECT_GE(target.toMessage().yaw, -kPi);
  EXPECT_LE(target.toMessage().yaw, kPi);
}

TEST(TargetTracker, MarksInvalidInitialGeometryAsDiverged)
{
  tracker::TrackerConfig config;
  config.geometry_initialization.radius = 0.0;
  tracker::TargetTracker target(config);

  ASSERT_TRUE(target.initialize(observation(1, stamp(10), 0.2, point(1.0, 2.0, 3.0))));

  EXPECT_TRUE(target.diverged());
}

TEST(TargetTracker, MapsTheStateIntoTheExistingTargetStateMessage)
{
  tracker::TargetTracker target;
  ASSERT_TRUE(target.initialize(observation(7, stamp(10), 0.2, point(1.0, 2.0, 3.0))));

  const auto message = target.toMessage();

  EXPECT_EQ(message.id, 7);
  EXPECT_TRUE(message.tracking);
  EXPECT_DOUBLE_EQ(message.center.x, target.state()[0]);
  EXPECT_DOUBLE_EQ(message.center.y, target.state()[2]);
  EXPECT_DOUBLE_EQ(message.center.z, target.state()[4]);
  EXPECT_DOUBLE_EQ(message.velocity.x, target.state()[1]);
  EXPECT_DOUBLE_EQ(message.velocity.y, target.state()[3]);
  EXPECT_DOUBLE_EQ(message.velocity.z, target.state()[5]);
  EXPECT_DOUBLE_EQ(message.yaw, target.state()[6]);
  EXPECT_DOUBLE_EQ(message.angular_velocity, target.state()[7]);
  EXPECT_DOUBLE_EQ(message.radius, target.state()[8]);
  EXPECT_DOUBLE_EQ(message.radius_offset, target.state()[9]);
  EXPECT_DOUBLE_EQ(message.height_offset, target.state()[10]);
  EXPECT_EQ(message.predicted_armors.size(), 4U);
}
