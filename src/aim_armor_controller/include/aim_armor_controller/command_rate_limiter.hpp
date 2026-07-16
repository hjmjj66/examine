#ifndef AIM_ARMOR_CONTROLLER__COMMAND_RATE_LIMITER_HPP_
#define AIM_ARMOR_CONTROLLER__COMMAND_RATE_LIMITER_HPP_

#include <algorithm>
#include <cmath>

namespace aim_armor_controller
{

struct CommandAngles
{
  double yaw_deg{0.0};
  double pitch_deg{0.0};
};

class CommandRateLimiter
{
public:
  void setMaxRateDegPerSec(double max_rate_deg_per_sec)
  {
    max_rate_deg_per_sec_ = std::max(0.0, max_rate_deg_per_sec);
  }

  void reset(double yaw_deg, double pitch_deg)
  {
    last_angles_ = CommandAngles{yaw_deg, pitch_deg};
    initialized_ = true;
  }

  CommandAngles update(double desired_yaw_deg, double desired_pitch_deg, double dt_sec)
  {
    if (!initialized_) {
      reset(desired_yaw_deg, desired_pitch_deg);
      return last_angles_;
    }

    if (max_rate_deg_per_sec_ <= 0.0) {
      last_angles_ = CommandAngles{desired_yaw_deg, desired_pitch_deg};
      return last_angles_;
    }

    const double max_delta_deg = max_rate_deg_per_sec_ * std::max(0.0, dt_sec);
    const double yaw_delta_deg = std::remainder(
      desired_yaw_deg - last_angles_.yaw_deg, 360.0);
    const double pitch_delta_deg = desired_pitch_deg - last_angles_.pitch_deg;

    last_angles_.yaw_deg += std::clamp(yaw_delta_deg, -max_delta_deg, max_delta_deg);
    last_angles_.pitch_deg += std::clamp(pitch_delta_deg, -max_delta_deg, max_delta_deg);
    return last_angles_;
  }

private:
  double max_rate_deg_per_sec_{0.0};
  CommandAngles last_angles_;
  bool initialized_{false};
};

}  // namespace aim_armor_controller

#endif  // AIM_ARMOR_CONTROLLER__COMMAND_RATE_LIMITER_HPP_
