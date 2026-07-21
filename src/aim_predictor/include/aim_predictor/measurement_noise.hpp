#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace aim_predictor
{

enum class CameraSource
{
  Front0,
  Front1,
  Back
};

// Multipliers for the legacy measurement-noise model. A value of 1.0 preserves
// the measurement covariance that was used before the multi-camera refactor.
struct MeasurementNoiseConfig
{
  double yaw_variance_scale{1.0};
  double pitch_variance_scale{1.0};
  double distance_variance_scale{1.0};
  double armor_yaw_variance_scale{1.0};
};

inline Eigen::Vector4d measurementNoiseDiagonal(
  double delta_angle,
  double distance,
  const MeasurementNoiseConfig & config)
{
  const Eigen::Vector4d legacy_variances(
    4e-3,
    4e-3,
    std::log(std::abs(delta_angle) + 1.0) + 0.1,
    std::log(std::abs(distance) + 1.0) / 200.0 + 9e-2);
  const Eigen::Vector4d variance_scales(
    std::max(config.yaw_variance_scale, 1e-6),
    std::max(config.pitch_variance_scale, 1e-6),
    std::max(config.distance_variance_scale, 1e-6),
    std::max(config.armor_yaw_variance_scale, 1e-6));
  return legacy_variances.cwiseProduct(variance_scales);
}

}  // namespace aim_predictor
