#include <gtest/gtest.h>

#include <cmath>

#include "aim_handeye_calibrator/gimbal_kinematics.hpp"

TEST(GimbalKinematicsTest, PlacesPitchAxisAboveYawAxisBeforePitchRotation)
{
  const auto transform = aim_handeye_calibrator::yawPitchTransform(
    M_PI_2, M_PI_2, 0.320);

  EXPECT_NEAR(transform(0, 3), 0.0, 1e-12);
  EXPECT_NEAR(transform(1, 3), 0.0, 1e-12);
  EXPECT_NEAR(transform(2, 3), 0.320, 1e-12);
  EXPECT_NEAR(transform(0, 0), 0.0, 1e-12);
  EXPECT_NEAR(transform(1, 0), 0.0, 1e-12);
  EXPECT_NEAR(transform(2, 0), -1.0, 1e-12);
}
