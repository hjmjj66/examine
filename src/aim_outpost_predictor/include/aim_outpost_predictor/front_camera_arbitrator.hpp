#pragma once

#include <chrono>

namespace aim_outpost_predictor
{

class FrontCameraArbitrator
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  explicit FrontCameraArbitrator(
    double fallback_timeout_sec = 0.2,
    TimePoint last_front_0_observation_time = std::chrono::steady_clock::now())
  : fallback_timeout_(std::chrono::duration<double>(fallback_timeout_sec)),
    last_front_0_observation_time_(last_front_0_observation_time)
  {
  }

  void setFallbackTimeout(double fallback_timeout_sec)
  {
    fallback_timeout_ = std::chrono::duration<double>(fallback_timeout_sec);
  }

  void setFallbackEnabled(bool enabled)
  {
    fallback_enabled_ = enabled;
  }

  bool shouldProcessFront0(bool has_outpost_observation, TimePoint now)
  {
    if (has_outpost_observation) {
      last_front_0_observation_time_ = now;
      return true;
    }
    return now - last_front_0_observation_time_ < fallback_timeout_;
  }

  bool shouldProcessFront1(TimePoint now) const
  {
    return fallback_enabled_ &&
           now - last_front_0_observation_time_ >= fallback_timeout_;
  }

private:
  std::chrono::duration<double> fallback_timeout_;
  TimePoint last_front_0_observation_time_;
  bool fallback_enabled_{true};
};

}  // namespace aim_outpost_predictor
