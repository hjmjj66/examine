#include "aim_armor_controller/armor_selection_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aim_armor_controller
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

bool candidateAllowed(
  const ArmorSelectionInput & input,
  const ArmorSelectionConfig & config,
  std::size_t index)
{
  const auto & candidate = input.candidates[index];
  if (!input.is_outpost) {
    return true;
  }

  if (
    !config.enable_smart_selector ||
    !config.allow_predicted_outpost_slots ||
    !input.tracking_converged)
  {
    return candidate.observed;
  }

  return candidate.observed || std::abs(candidate.facing_error) <= config.coming_angle_rad;
}

bool candidateInComingWindow(
  const ArmorSelectionInput & input,
  const ArmorSelectionConfig & config,
  std::size_t index)
{
  const double coming_angle = config.coming_angle_rad > 0.0 ?
    config.coming_angle_rad : kPi / 3.0;
  const double leaving_angle = std::max(0.0, config.leaving_angle_rad);
  const double error = input.candidates[index].facing_error;
  if (std::abs(error) > coming_angle) {
    return false;
  }

  if (input.angular_velocity > 0.0) {
    return error < leaving_angle;
  }
  return error > -leaving_angle;
}

int nearestAllowedCandidate(
  const ArmorSelectionInput & input,
  const ArmorSelectionConfig & config)
{
  int best_index = -1;
  double best_error = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < input.candidates.size(); ++i) {
    if (!candidateAllowed(input, config, i)) {
      continue;
    }
    const double error = std::abs(input.candidates[i].facing_error);
    if (error < best_error) {
      best_error = error;
      best_index = static_cast<int>(i);
    }
  }
  return best_index;
}

}  // namespace

double computeArmorJumpAngle(
  double distance,
  double radius,
  double angular_velocity,
  const ArmorSelectionConfig & config)
{
  double jump_angle = kPi / 4.0;
  if (
    std::abs(angular_velocity) > 1e-6 &&
    radius > 1e-6 &&
    distance > 0.0 &&
    config.response_speed_rad_s > 0.0)
  {
    const double compensation =
      1.0 / (
      1.0 + 1.414 * distance * config.response_speed_rad_s /
      radius / std::abs(angular_velocity));
    jump_angle -= compensation;
  }

  return std::max(jump_angle, config.min_switch_angle_rad);
}

ArmorSelectionResult selectArmorIndex(
  const ArmorSelectionInput & input,
  const ArmorSelectionConfig & config)
{
  ArmorSelectionResult result;
  if (input.candidates.empty()) {
    return result;
  }

  const int nearest_index = nearestAllowedCandidate(input, config);
  if (nearest_index < 0) {
    return result;
  }

  const bool locked_index_valid =
    input.locked_index >= 0 &&
    input.locked_index < static_cast<int>(input.candidates.size());
  if (!config.enable_smart_selector || !locked_index_valid) {
    result.valid = true;
    result.index = nearest_index;
    result.switched = locked_index_valid && result.index != input.locked_index;
    return result;
  }

  const bool locked_allowed =
    candidateAllowed(input, config, static_cast<std::size_t>(input.locked_index));
  if (!locked_allowed) {
    result.valid = true;
    result.index = nearest_index;
    result.switched = input.locked_index != nearest_index;
    return result;
  }

  if (std::abs(input.angular_velocity) <= config.min_angular_velocity_rad_s) {
    result.valid = true;
    result.index = input.locked_index;
    return result;
  }

  const double jump_angle = computeArmorJumpAngle(
    input.distance, input.radius, input.angular_velocity, config);
  const double locked_error =
    input.candidates[static_cast<std::size_t>(input.locked_index)].facing_error;
  const int candidate_count = static_cast<int>(input.candidates.size());
  const int next_index =
    input.angular_velocity > 0.0 ?
    (input.locked_index + candidate_count - 1) % candidate_count :
    (input.locked_index + 1) % candidate_count;
  const bool crossed_switch_angle =
    input.angular_velocity > 0.0 ?
    locked_error > jump_angle :
    locked_error < -jump_angle;

  if (
    crossed_switch_angle &&
    candidateAllowed(input, config, static_cast<std::size_t>(next_index)) &&
    candidateInComingWindow(input, config, static_cast<std::size_t>(next_index)))
  {
    result.valid = true;
    result.switched = next_index != input.locked_index;
    result.index = next_index;
    result.switch_angle_rad = jump_angle;
    return result;
  }

  result.valid = true;
  result.index = input.locked_index;
  result.switch_angle_rad = jump_angle;
  return result;
}

}  // namespace aim_armor_controller
