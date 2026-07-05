#pragma once

#include <optional>
#include <vector>

#include <opencv2/opencv.hpp>

#include "aim_armor_detector/detection_types.hpp"

namespace aim_armor_detector
{

struct LightbarPcaCorrectorConfig
{
  int pass_optimize_lightbar_width{3};
  float normalize_max_brightness{25.0F};
  float lightbar_min_mean_brightness{30.0F};
  float padding_scale{0.07F};
  float search_start_ratio{0.4F};
  float search_end_ratio{0.6F};
  float estimated_width_ratio{0.18F};
  int min_sample_width{5};
};

class LightbarPcaCorrector
{
public:
  explicit LightbarPcaCorrector(const LightbarPcaCorrectorConfig & config);

  void correctDetections(std::vector<DetectionObject> & detections, const cv::Mat & gray_image) const;

private:
  struct SymmetryAxis
  {
    cv::Point2f centroid;
    cv::Point2f direction;
    float mean_val;
  };

  struct LightBar
  {
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f center;
    cv::Point2f axis;
    float length{0.0F};
    float width{0.0F};

    cv::Rect boundingRect() const;
  };

  bool correctDetection(DetectionObject & detection, const cv::Mat & gray_image) const;
  bool correctLightBar(LightBar & lightbar, const cv::Mat & gray_image) const;
  std::optional<SymmetryAxis> findSymmetryAxis(const cv::Mat & gray_image, const LightBar & lightbar) const;
  std::optional<cv::Point2f> findCorner(
    const cv::Mat & gray_image,
    const LightBar & lightbar,
    const SymmetryAxis & axis,
    bool find_top) const;
  float estimateLightBarWidth(const DetectionObject & detection) const;
  static void updateDetectionRect(DetectionObject & detection);

  LightbarPcaCorrectorConfig config_;
};

}  // namespace aim_armor_detector
