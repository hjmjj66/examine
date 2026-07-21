#pragma once

#include "sentry_msgs/msg/aim_result.hpp"

namespace aim_armor_controller
{

inline void setAimResultKinematics(
  sentry_msgs::msg::AimResult & result,
  double yaw_deg,
  double pitch_deg,
  double yaw_omega_deg_s,
  double pitch_omega_deg_s,
  double yaw_alpha_deg_s2,
  double pitch_alpha_deg_s2)
{
  result.yaw = static_cast<float>(yaw_deg);
  result.pitch = static_cast<float>(pitch_deg);
  result.yaw_omega = static_cast<float>(yaw_omega_deg_s);
  result.pitch_omega = static_cast<float>(pitch_omega_deg_s);
  result.yaw_alpha = static_cast<float>(yaw_alpha_deg_s2);
  result.pitch_alpha = static_cast<float>(pitch_alpha_deg_s2);
}

}  // namespace aim_armor_controller
