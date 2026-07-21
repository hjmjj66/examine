#pragma once

#include <gimbal_driver/msg/gimbal_trajectory.hpp>

namespace aim_armor_controller
{

using ControllerTrajectoryMessage = gimbal_driver::msg::GimbalTrajectory;

inline ControllerTrajectoryMessage makeControllerTrajectoryMessage(
  double yaw_deg,
  double yaw_omega_deg_s,
  double yaw_alpha_deg_s2,
  double pitch_deg,
  double pitch_omega_deg_s,
  double pitch_alpha_deg_s2)
{
  ControllerTrajectoryMessage message;
  message.yaw = static_cast<float>(yaw_deg);
  message.yaw_omega = static_cast<float>(yaw_omega_deg_s);
  message.yaw_alpha = static_cast<float>(yaw_alpha_deg_s2);
  message.pitch = static_cast<float>(pitch_deg);
  message.pitch_omega = static_cast<float>(pitch_omega_deg_s);
  message.pitch_alpha = static_cast<float>(pitch_alpha_deg_s2);
  return message;
}

}  // namespace aim_armor_controller
