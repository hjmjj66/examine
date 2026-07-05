#pragma once

#include <opencv2/opencv.hpp>

namespace aim_armor_detector
{

struct DetectionObject
{
  cv::Rect2f rect;
  float landmarks[8];
  int label;
  float probability;
  int color;
};

}  // namespace aim_armor_detector
