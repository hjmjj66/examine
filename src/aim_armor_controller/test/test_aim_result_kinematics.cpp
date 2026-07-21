#include <gtest/gtest.h>

#include "aim_armor_controller/aim_result_kinematics.hpp"
#include "sentry_msgs/msg/aim_result.hpp"

TEST(AimResultKinematicsTest, CopiesAnglesAndDerivatives)
{
  sentry_msgs::msg::AimResult result;

  aim_armor_controller::setAimResultKinematics(
    result, 10.0, 20.0, 1.5, 2.5, 3.5, 4.5);

  EXPECT_FLOAT_EQ(result.yaw, 10.0F);
  EXPECT_FLOAT_EQ(result.pitch, 20.0F);
  EXPECT_FLOAT_EQ(result.yaw_omega, 1.5F);
  EXPECT_FLOAT_EQ(result.pitch_omega, 2.5F);
  EXPECT_FLOAT_EQ(result.yaw_alpha, 3.5F);
  EXPECT_FLOAT_EQ(result.pitch_alpha, 4.5F);
}
