#pragma once

#include <array>
#include <cstddef>

namespace tracker
{

struct GeometryInitialization
{
  double radius{0.22};
  double radius_offset{-0.01};
  double height_offset{0.02};
};

struct LifecycleConfig
{
  std::size_t confirmation_count{4};
  double confirmation_min_interval_sec{0.03};
  double target_lost_timeout_sec{0.20};
  double front_target_hold_sec{0.10};
  double min_radius{0.05};
  double max_radius{0.50};
};

struct ProcessNoiseBand
{
  double process_noise_xy{1600.0};
  double process_noise_z{1600.0};
  double process_noise_yaw{400.0};
};

struct ProcessNoiseConfig
{
  double middle_speed_angular_velocity_threshold{2.0};
  double high_speed_angular_velocity_threshold{4.0};
  ProcessNoiseBand low_speed{};
  ProcessNoiseBand middle_speed{};
  ProcessNoiseBand high_speed{};
};

struct SigmaConfig
{
  std::array<double, 11> prior_sigma{
    1.0, 8.0, 1.0, 8.0, 1.0, 8.0, 0.6324555320, 10.0, 1.0, 1.0, 1.0};
  double translation_sigma{1.0};
  double velocity_sigma{1.0};
  double yaw_sigma{1.0};
  double yaw_velocity_sigma{1.0};
  std::array<double, 4> geometry_sigma{1.0, 1.0, 1.0, 1.0};
  double pixel_sigma{1.0};
};

struct CameraNoiseScales
{
  double front_0{1.0};
  double front_1{1.0};
  double back{1.0};
};

struct TrackerConfig
{
  std::size_t window_size{30};
  GeometryInitialization geometry_initialization{};
  LifecycleConfig lifecycle{};
  ProcessNoiseConfig process_noise{};
  SigmaConfig sigma{};
  CameraNoiseScales camera_noise_scales{};
};

}  // namespace tracker
