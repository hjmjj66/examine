#include "aim_armor_detector/lightbar_pca_corrector.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace aim_armor_detector
{

namespace
{

float pointDistance(const cv::Point2f & a, const cv::Point2f & b)
{
  return cv::norm(a - b);
}

}  // namespace

LightbarPcaCorrector::LightbarPcaCorrector(const LightbarPcaCorrectorConfig & config)
: config_(config)
{
}

void LightbarPcaCorrector::correctDetections(
  std::vector<DetectionObject> & detections,
  const cv::Mat & gray_image) const
{
  if (gray_image.empty()) {
    return;
  }

  for (auto & detection : detections) {
    correctDetection(detection, gray_image);
  }
}

bool LightbarPcaCorrector::correctDetection(DetectionObject & detection, const cv::Mat & gray_image) const
{
  LightBar left{};
  left.top = cv::Point2f(detection.landmarks[0], detection.landmarks[1]);
  left.bottom = cv::Point2f(detection.landmarks[6], detection.landmarks[7]);
  left.length = pointDistance(left.top, left.bottom);
  left.width = estimateLightBarWidth(detection);

  LightBar right{};
  right.top = cv::Point2f(detection.landmarks[2], detection.landmarks[3]);
  right.bottom = cv::Point2f(detection.landmarks[4], detection.landmarks[5]);
  right.length = pointDistance(right.top, right.bottom);
  right.width = left.width;

  if (left.length <= 1.0F || right.length <= 1.0F) {
    return false;
  }

  LightBar left_copy = left;
  LightBar right_copy = right;
  const bool left_ok = correctLightBar(left_copy, gray_image);
  const bool right_ok = correctLightBar(right_copy, gray_image);
  if (!left_ok || !right_ok) {
    return false;
  }

  detection.landmarks[0] = left_copy.top.x;
  detection.landmarks[1] = left_copy.top.y;
  detection.landmarks[2] = right_copy.top.x;
  detection.landmarks[3] = right_copy.top.y;
  detection.landmarks[4] = right_copy.bottom.x;
  detection.landmarks[5] = right_copy.bottom.y;
  detection.landmarks[6] = left_copy.bottom.x;
  detection.landmarks[7] = left_copy.bottom.y;
  updateDetectionRect(detection);
  return true;
}

bool LightbarPcaCorrector::correctLightBar(LightBar & lightbar, const cv::Mat & gray_image) const
{
  if (lightbar.width <= static_cast<float>(config_.pass_optimize_lightbar_width)) {
    return false;
  }

  const auto axis = findSymmetryAxis(gray_image, lightbar);
  if (!axis.has_value()) {
    return false;
  }

  lightbar.center = axis->centroid;
  lightbar.axis = axis->direction;
  const auto top = findCorner(gray_image, lightbar, axis.value(), true);
  const auto bottom = findCorner(gray_image, lightbar, axis.value(), false);
  if (!top.has_value() || !bottom.has_value()) {
    return false;
  }

  lightbar.top = top.value();
  lightbar.bottom = bottom.value();
  return true;
}

std::optional<LightbarPcaCorrector::SymmetryAxis> LightbarPcaCorrector::findSymmetryAxis(
  const cv::Mat & gray_image,
  const LightBar & lightbar) const
{
  auto light_box = lightbar.boundingRect();
  const int pad_x = static_cast<int>(std::lround(static_cast<float>(light_box.width) * config_.padding_scale));
  const int pad_y = static_cast<int>(std::lround(static_cast<float>(light_box.height) * config_.padding_scale));
  light_box.x -= pad_x;
  light_box.y -= pad_y;
  light_box.width += pad_x * 2;
  light_box.height += pad_y * 2;

  light_box.x = std::clamp(light_box.x, 0, gray_image.cols - 1);
  light_box.y = std::clamp(light_box.y, 0, gray_image.rows - 1);
  light_box.width = std::min(light_box.width, gray_image.cols - light_box.x);
  light_box.height = std::min(light_box.height, gray_image.rows - light_box.y);
  if (light_box.width <= 1 || light_box.height <= 1) {
    return std::nullopt;
  }

  cv::Mat roi = gray_image(light_box).clone();
  const float mean_val = static_cast<float>(cv::mean(roi)[0]);
  if (mean_val <= config_.lightbar_min_mean_brightness) {
    return std::nullopt;
  }

  roi.convertTo(roi, CV_32F);
  cv::normalize(roi, roi, 0, config_.normalize_max_brightness, cv::NORM_MINMAX);

  const cv::Moments moments = cv::moments(roi, false);
  if (moments.m00 == 0.0) {
    return std::nullopt;
  }

  cv::Point2f centroid(
    static_cast<float>(moments.m10 / moments.m00) + static_cast<float>(light_box.x),
    static_cast<float>(moments.m01 / moments.m00) + static_cast<float>(light_box.y));

  const double mu20 = moments.mu20;
  const double mu11 = moments.mu11;
  const double mu02 = moments.mu02;
  if (mu20 == 0.0 && mu11 == 0.0 && mu02 == 0.0) {
    return std::nullopt;
  }

  const double theta = 0.5 * std::atan2(2.0 * mu11, mu20 - mu02);
  cv::Point2f direction(static_cast<float>(std::cos(theta)), static_cast<float>(std::sin(theta)));
  const float norm = cv::norm(direction);
  if (norm <= 1e-6F) {
    return std::nullopt;
  }
  direction /= norm;

  const cv::Point2f hint = lightbar.top - lightbar.bottom;
  if (direction.dot(hint) < 0.0F) {
    direction = -direction;
  }

  return SymmetryAxis{centroid, direction, mean_val};
}

std::optional<cv::Point2f> LightbarPcaCorrector::findCorner(
  const cv::Mat & gray_image,
  const LightBar & lightbar,
  const SymmetryAxis & axis,
  bool find_top) const
{
  const auto is_in_image = [&gray_image](const cv::Point & point) {
      return point.x >= 0 && point.x < gray_image.cols && point.y >= 0 && point.y < gray_image.rows;
    };

  const int direction_sign = find_top ? 1 : -1;
  const float length = lightbar.length;
  const float dx = axis.direction.x * static_cast<float>(direction_sign);
  const float dy = axis.direction.y * static_cast<float>(direction_sign);

  std::vector<cv::Point2f> candidates;
  const int sample_width = std::max(
    config_.min_sample_width,
    static_cast<int>(std::lround(lightbar.width)) - 2);
  const int half_width = std::max(0, sample_width / 2);

  for (int i = -half_width; i <= half_width; ++i) {
    const float x0 = axis.centroid.x + length * config_.search_start_ratio * dx + static_cast<float>(i);
    const float y0 = axis.centroid.y + length * config_.search_start_ratio * dy;

    cv::Point2f prev(x0, y0);
    cv::Point2f corner(x0, y0);
    float max_brightness_diff = 0.0F;
    bool has_corner = false;
    for (float x = x0 + dx, y = y0 + dy;
      pointDistance(cv::Point2f(x, y), cv::Point2f(x0, y0)) <
      length * (config_.search_end_ratio - config_.search_start_ratio);
      x += dx, y += dy)
    {
      const cv::Point prev_pt(
        static_cast<int>(std::lround(prev.x)),
        static_cast<int>(std::lround(prev.y)));
      const cv::Point cur_pt(
        static_cast<int>(std::lround(x)),
        static_cast<int>(std::lround(y)));
      if (!is_in_image(prev_pt) || !is_in_image(cur_pt)) {
        break;
      }

      const float brightness_diff =
        static_cast<float>(gray_image.at<uchar>(prev_pt) - gray_image.at<uchar>(cur_pt));
      if (brightness_diff > max_brightness_diff &&
        static_cast<float>(gray_image.at<uchar>(prev_pt)) > axis.mean_val)
      {
        max_brightness_diff = brightness_diff;
        corner = prev;
        has_corner = true;
      }
      prev = cv::Point2f(x, y);
    }

    if (has_corner) {
      candidates.emplace_back(corner);
    }
  }

  if (candidates.empty()) {
    return std::nullopt;
  }

  const auto sum = std::accumulate(
    candidates.begin(),
    candidates.end(),
    cv::Point2f(0.0F, 0.0F));
  return sum * (1.0F / static_cast<float>(candidates.size()));
}

float LightbarPcaCorrector::estimateLightBarWidth(const DetectionObject & detection) const
{
  const float top_width = pointDistance(
    cv::Point2f(detection.landmarks[0], detection.landmarks[1]),
    cv::Point2f(detection.landmarks[2], detection.landmarks[3]));
  const float bottom_width = pointDistance(
    cv::Point2f(detection.landmarks[6], detection.landmarks[7]),
    cv::Point2f(detection.landmarks[4], detection.landmarks[5]));
  return std::max(1.0F, 0.5F * (top_width + bottom_width) * config_.estimated_width_ratio);
}

void LightbarPcaCorrector::updateDetectionRect(DetectionObject & detection)
{
  float min_x = detection.landmarks[0];
  float max_x = detection.landmarks[0];
  float min_y = detection.landmarks[1];
  float max_y = detection.landmarks[1];
  for (int i = 1; i < 4; ++i) {
    min_x = std::min(min_x, detection.landmarks[i * 2]);
    max_x = std::max(max_x, detection.landmarks[i * 2]);
    min_y = std::min(min_y, detection.landmarks[i * 2 + 1]);
    max_y = std::max(max_y, detection.landmarks[i * 2 + 1]);
  }
  detection.rect = cv::Rect2f(min_x, min_y, max_x - min_x, max_y - min_y);
}

cv::Rect LightbarPcaCorrector::LightBar::boundingRect() const
{
  const float half_width = width * 0.5F;
  const float min_x = std::min(top.x, bottom.x) - half_width;
  const float min_y = std::min(top.y, bottom.y) - half_width;
  const float max_x = std::max(top.x, bottom.x) + half_width;
  const float max_y = std::max(top.y, bottom.y) + half_width;
  return cv::Rect(
    cv::Point(
      static_cast<int>(std::floor(min_x)),
      static_cast<int>(std::floor(min_y))),
    cv::Point(
      static_cast<int>(std::ceil(max_x)),
      static_cast<int>(std::ceil(max_y))));
}

}  // namespace aim_armor_detector
