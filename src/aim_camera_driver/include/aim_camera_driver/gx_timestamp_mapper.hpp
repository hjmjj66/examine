#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <rclcpp/time.hpp>

namespace aim_camera_driver
{

/**
 * Maps the monotonic timestamp counter of a GX camera to the ROS clock.
 *
 * The device counter is deliberately kept as a relative value.  This avoids
 * converting a large device tick count to a double and losing sub-millisecond
 * precision.  Repeated latch samples slowly correct the host/device offset;
 * the tick frequency itself remains the device-provided value.
 */
class GxTimestampMapper
{
public:
  struct Config
  {
    double tick_frequency_hz{0.0};
    double fixed_offset_sec{0.0};
    double latch_alpha{0.1};
    double max_latch_correction_sec{0.02};
  };

  GxTimestampMapper()
  : GxTimestampMapper(Config{})
  {
  }

  explicit GxTimestampMapper(const Config & config)
  : config_(config)
  {
    config_.latch_alpha = std::clamp(config_.latch_alpha, 0.0, 1.0);
    config_.max_latch_correction_sec =
      std::max(0.0, config_.max_latch_correction_sec);
  }

  void setTickFrequency(double tick_frequency_hz)
  {
    config_.tick_frequency_hz = tick_frequency_hz;
  }

  [[nodiscard]] bool valid() const
  {
    return config_.tick_frequency_hz > 0.0 && initialized_;
  }

  [[nodiscard]] double tickFrequencyHz() const
  {
    return config_.tick_frequency_hz;
  }

  [[nodiscard]] double fixedOffsetSec() const
  {
    return config_.fixed_offset_sec;
  }

  [[nodiscard]] double estimatedLatchCorrectionSec() const
  {
    return latch_correction_sec_;
  }

  [[nodiscard]] std::uint64_t lastDeviceTick() const
  {
    return last_device_tick_;
  }

  void reset()
  {
    initialized_ = false;
    last_device_tick_ = 0U;
    anchor_device_tick_ = 0U;
    anchor_ros_sec_ = 0.0;
    latch_correction_sec_ = 0.0;
  }

  bool initialize(std::uint64_t device_tick, const rclcpp::Time & ros_time)
  {
    if (config_.tick_frequency_hz <= 0.0 || !std::isfinite(ros_time.seconds())) {
      return false;
    }

    initialized_ = true;
    anchor_device_tick_ = device_tick;
    anchor_ros_sec_ = ros_time.seconds();
    last_device_tick_ = device_tick;
    latch_correction_sec_ = 0.0;
    return true;
  }

  /**
   * Incorporates a host/device latch pair.  The host time should be sampled
   * around the SDK command and preferably be the midpoint of before/after.
   */
  bool updateLatch(std::uint64_t device_tick, const rclcpp::Time & ros_time)
  {
    if (config_.tick_frequency_hz <= 0.0 || !std::isfinite(ros_time.seconds())) {
      return false;
    }

    if (!initialized_ || device_tick < last_device_tick_) {
      return initialize(device_tick, ros_time);
    }

    const double predicted_ros_sec =
      anchor_ros_sec_ +
      static_cast<double>(device_tick - anchor_device_tick_) /
      config_.tick_frequency_hz + latch_correction_sec_;
    const double error_sec = ros_time.seconds() - predicted_ros_sec;
    const double bounded_error = std::clamp(
      error_sec,
      -config_.max_latch_correction_sec,
      config_.max_latch_correction_sec);
    latch_correction_sec_ += config_.latch_alpha * bounded_error;
    last_device_tick_ = device_tick;
    return true;
  }

  [[nodiscard]] rclcpp::Time map(
    std::uint64_t device_tick,
    const rclcpp::Time & fallback) const
  {
    if (!valid() || device_tick < anchor_device_tick_) {
      return fallback;
    }

    const double mapped_sec =
      anchor_ros_sec_ +
      static_cast<double>(device_tick - anchor_device_tick_) /
      config_.tick_frequency_hz + latch_correction_sec_ + config_.fixed_offset_sec;
    if (!std::isfinite(mapped_sec) || mapped_sec <= 0.0) {
      return fallback;
    }
    return rclcpp::Time(static_cast<std::int64_t>(std::llround(mapped_sec * 1e9)));
  }

private:
  Config config_;
  bool initialized_{false};
  std::uint64_t last_device_tick_{0U};
  std::uint64_t anchor_device_tick_{0U};
  double anchor_ros_sec_{0.0};
  double latch_correction_sec_{0.0};
};

}  // namespace aim_camera_driver
