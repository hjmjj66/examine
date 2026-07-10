#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "aim_solver/coordinate_utils.hpp"

TEST(AimSolverCoordinateUtilsTest, ConvertsOpticalPointToEngineeringCameraFrame)
{
  const cv::Vec3d optical_point(1.0, 2.0, 3.0);
  const cv::Vec3d camera_point = aim_solver::opticalPointToCameraFrame(optical_point);

  EXPECT_DOUBLE_EQ(camera_point[0], 3.0);
  EXPECT_DOUBLE_EQ(camera_point[1], -1.0);
  EXPECT_DOUBLE_EQ(camera_point[2], -2.0);
}

TEST(AimSolverCoordinateUtilsTest, ConvertsOpticalRotationToEngineeringCameraFrame)
{
  const cv::Matx33d optical_rotation = cv::Matx33d::eye();
  const cv::Matx33d camera_rotation = aim_solver::opticalRotationToCameraFrame(optical_rotation);

  EXPECT_DOUBLE_EQ(camera_rotation(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(camera_rotation(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(camera_rotation(0, 2), 1.0);
  EXPECT_DOUBLE_EQ(camera_rotation(1, 0), -1.0);
  EXPECT_DOUBLE_EQ(camera_rotation(1, 1), 0.0);
  EXPECT_DOUBLE_EQ(camera_rotation(1, 2), 0.0);
  EXPECT_DOUBLE_EQ(camera_rotation(2, 0), 0.0);
  EXPECT_DOUBLE_EQ(camera_rotation(2, 1), -1.0);
  EXPECT_DOUBLE_EQ(camera_rotation(2, 2), 0.0);
}
