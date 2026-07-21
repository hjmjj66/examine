#include "aim_armor_controller/mpc_trajectory_solver.hpp"

#include <algorithm>
#include <cmath>

namespace aim_armor_controller
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.794;
constexpr double kNoAirGravity = 9.7833;
constexpr double kDragCoefficient = 0.47;
constexpr double kAirDensity = 1.204;
constexpr double kBulletMass = 3.2e-3;
constexpr double kBulletDiameter = 16.8e-3;
constexpr int kMaxIterations = 100;
constexpr double kTolerance = 1e-6;

MpcTrajectorySolution solveNoAirTrajectory(
  double bullet_speed,
  double target_x,
  double target_y,
  double target_z)
{
  MpcTrajectorySolution result;
  const double distance = std::hypot(target_x, target_y);
  const double a = kNoAirGravity * distance * distance /
    (2.0 * bullet_speed * bullet_speed);
  const double b = -distance;
  const double c = a + target_z;
  const double discriminant = b * b - 4.0 * a * c;
  if (distance < 1e-6) {
    result.failure_reason = "no_air_distance_too_small";
    result.unsolvable = true;
    return result;
  }
  if (std::abs(a) < 1e-12) {
    result.failure_reason = "no_air_degenerate_coefficient";
    result.unsolvable = true;
    return result;
  }
  if (discriminant < 0.0) {
    result.failure_reason = "no_air_discriminant_negative";
    result.unsolvable = true;
    return result;
  }

  const double sqrt_discriminant = std::sqrt(discriminant);
  const double tan_pitch_1 = (-b + sqrt_discriminant) / (2.0 * a);
  const double tan_pitch_2 = (-b - sqrt_discriminant) / (2.0 * a);
  const double pitch_1 = std::atan(tan_pitch_1);
  const double pitch_2 = std::atan(tan_pitch_2);
  const double time_1 = distance / (bullet_speed * std::cos(pitch_1));
  const double time_2 = distance / (bullet_speed * std::cos(pitch_2));

  result.yaw = std::atan2(target_y, target_x);
  result.pitch = time_1 < time_2 ? pitch_1 : pitch_2;
  result.fly_time = time_1 < time_2 ? time_1 : time_2;
  return result;
}

}  // namespace

MpcTrajectorySolution solveMpcTrajectory(
  double bullet_speed,
  double target_x,
  double target_y,
  double target_z,
  bool use_air_resistance)
{
  MpcTrajectorySolution result;
  if (bullet_speed <= 0.0) {
    result.failure_reason = "bullet_speed_non_positive";
    result.unsolvable = true;
    return result;
  }
  if (!use_air_resistance) {
    return solveNoAirTrajectory(bullet_speed, target_x, target_y, target_z);
  }

  const double distance = std::hypot(target_x, target_y);
  double pitch = std::atan2(target_z, std::max(distance, 1e-6));
  const double drag_factor =
    kDragCoefficient * kAirDensity * (kPi * kBulletDiameter * kBulletDiameter) /
    (8.0 * kBulletMass);

  for (int i = 0; i < kMaxIterations; ++i) {
    const double cos_pitch = std::cos(pitch);
    if (std::abs(cos_pitch) < 1e-6) {
      result.failure_reason = "air_pitch_cosine_too_small";
      result.unsolvable = true;
      return result;
    }

    const double exp_term = std::exp(drag_factor * distance) - 1.0;
    const double fly_time = exp_term / (drag_factor * bullet_speed * cos_pitch);
    const double delta_z =
      target_z - bullet_speed * std::sin(pitch) * fly_time +
      0.5 * kGravity * fly_time * fly_time;

    if (std::abs(delta_z) < kTolerance) {
      result.yaw = std::atan2(target_y, target_x);
      result.pitch = pitch;
      result.fly_time = fly_time;
      return result;
    }

    const double dt_dpitch =
      exp_term * std::sin(pitch) /
      (drag_factor * bullet_speed * cos_pitch * cos_pitch);
    const double derivative =
      -bullet_speed * (cos_pitch * fly_time + std::sin(pitch) * dt_dpitch) +
      kGravity * fly_time * dt_dpitch;
    if (std::abs(derivative) < 1e-9) {
      result.failure_reason = "air_newton_derivative_too_small";
      break;
    }
    pitch -= delta_z / derivative;
  }

  if (result.failure_reason.empty()) {
    result.failure_reason = "air_iteration_limit";
  }
  result.unsolvable = true;
  return result;
}

}  // namespace aim_armor_controller
