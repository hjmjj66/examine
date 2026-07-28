#pragma once

#include <cmath>

namespace aim_armor_controller
{

constexpr double kPi = 3.14159265358979323846;

inline double limitRad(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

inline double outpostArmorFacingError(double center_x, double center_y, double armor_yaw)
{
  const double gun_angle = std::atan2(center_y, center_x);
  return limitRad(armor_yaw - gun_angle);
}

inline bool allowOutpostFireByFacingGate(
  double center_x,
  double center_y,
  double armor_yaw,
  double gate_rad)
{
  if (gate_rad <= 0.0) {
    return true;
  }
  return std::abs(outpostArmorFacingError(center_x, center_y, armor_yaw)) <= gate_rad;
}

}  // namespace aim_armor_controller
