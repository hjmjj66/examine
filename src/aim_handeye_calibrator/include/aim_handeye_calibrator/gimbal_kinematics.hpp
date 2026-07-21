#pragma once

#include <cmath>

#include <opencv2/core.hpp>

namespace aim_handeye_calibrator
{

inline cv::Matx44d yawPitchTransform(
  double yaw_rad, double pitch_rad, double pitch_axis_offset_z)
{
  const double cy = std::cos(yaw_rad);
  const double sy = std::sin(yaw_rad);
  const double cp = std::cos(pitch_rad);
  const double sp = std::sin(pitch_rad);

  const cv::Matx44d yaw(
    cy, -sy, 0.0, 0.0,
    sy, cy, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0);
  const cv::Matx44d pitch_axis_offset(
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, pitch_axis_offset_z,
    0.0, 0.0, 0.0, 1.0);
  const cv::Matx44d pitch(
    cp, 0.0, sp, 0.0,
    0.0, 1.0, 0.0, 0.0,
    -sp, 0.0, cp, 0.0,
    0.0, 0.0, 0.0, 1.0);

  return yaw * pitch_axis_offset * pitch;
}

}  // namespace aim_handeye_calibrator
