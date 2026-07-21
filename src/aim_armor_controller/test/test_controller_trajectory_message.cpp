#include <type_traits>

#include <gtest/gtest.h>

#include <gimbal_driver/msg/gimbal_trajectory.hpp>

#include "aim_armor_controller/controller_trajectory_message.hpp"

TEST(ControllerTrajectoryMessage, UsesGimbalDriverTrajectoryType)
{
  static_assert(std::is_same_v<
    aim_armor_controller::ControllerTrajectoryMessage,
    gimbal_driver::msg::GimbalTrajectory>);
}

TEST(ControllerTrajectoryMessage, PreservesAllTrajectoryFields)
{
  const auto message = aim_armor_controller::makeControllerTrajectoryMessage(
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  EXPECT_FLOAT_EQ(message.yaw, 1.0F);
  EXPECT_FLOAT_EQ(message.yaw_omega, 2.0F);
  EXPECT_FLOAT_EQ(message.yaw_alpha, 3.0F);
  EXPECT_FLOAT_EQ(message.pitch, 4.0F);
  EXPECT_FLOAT_EQ(message.pitch_omega, 5.0F);
  EXPECT_FLOAT_EQ(message.pitch_alpha, 6.0F);
}
