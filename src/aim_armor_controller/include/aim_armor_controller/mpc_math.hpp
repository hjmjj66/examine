#ifndef AIM_ARMOR_CONTROLLER__MPC_MATH_HPP_
#define AIM_ARMOR_CONTROLLER__MPC_MATH_HPP_

#include <cmath>
#include <vector>

namespace aim_armor_controller
{

constexpr double kMpcPi = 3.14159265358979323846;

inline double normalizeMpcAngle(double angle)
{
  while (angle > kMpcPi) {
    angle -= 2.0 * kMpcPi;
  }
  while (angle <= -kMpcPi) {
    angle += 2.0 * kMpcPi;
  }
  return angle;
}

inline double shortestAngleDelta(double target, double current)
{
  return normalizeMpcAngle(target - current);
}

inline double unwrapAngleNear(double angle, double reference)
{
  return reference + shortestAngleDelta(angle, reference);
}

inline std::vector<double> buildCenteredDelaySamples(
  double base_delay, double dt, int horizon)
{
  std::vector<double> samples;
  if (horizon < 0) {
    return samples;
  }

  samples.reserve(static_cast<std::size_t>(horizon + 1));
  const int half_horizon = horizon / 2;
  for (int i = 0; i <= horizon; ++i) {
    samples.push_back(base_delay + static_cast<double>(i - half_horizon) * dt);
  }
  return samples;
}

inline double absoluteYawFromWorldPoint(
  double target_x, double target_y, double target_z,
  double gun_x, double gun_y, double gun_z)
{
  (void)target_z;
  (void)gun_z;
  return std::atan2(target_y - gun_y, target_x - gun_x);
}

}  // namespace aim_armor_controller

#endif  // AIM_ARMOR_CONTROLLER__MPC_MATH_HPP_
