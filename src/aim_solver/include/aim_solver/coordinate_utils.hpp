#pragma once

#include <opencv2/core.hpp>

namespace aim_solver
{

inline cv::Matx33d opticalToCameraBasisRotation()
{
  return cv::Matx33d(
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0);
}

inline cv::Matx33d cameraToOpticalBasisRotation()
{
  return opticalToCameraBasisRotation().t();
}

inline cv::Vec3d opticalPointToCameraFrame(const cv::Vec3d & optical_point)
{
  return opticalToCameraBasisRotation() * optical_point;
}

inline cv::Vec3d cameraPointToOpticalFrame(const cv::Vec3d & camera_point)
{
  return cameraToOpticalBasisRotation() * camera_point;
}

inline cv::Matx33d opticalRotationToCameraFrame(const cv::Matx33d & optical_rotation)
{
  return opticalToCameraBasisRotation() * optical_rotation;
}

inline cv::Matx33d cameraRotationToOpticalFrame(const cv::Matx33d & camera_rotation)
{
  return cameraToOpticalBasisRotation() * camera_rotation;
}

}  // namespace aim_solver
