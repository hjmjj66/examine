#pragma once

#include <vector>

namespace aim_armor_controller
{

struct ArmorSelectionCandidate
{
  double facing_error{0.0};
  bool observed{true};
};

struct ArmorSelectionConfig
{
  bool enable_smart_selector{true};
  double coming_angle_rad{60.0 * 3.14159265358979323846 / 180.0};
  double leaving_angle_rad{20.0 * 3.14159265358979323846 / 180.0};
  double response_speed_rad_s{0.01};
  double min_angular_velocity_rad_s{0.6};
  double min_switch_angle_rad{30.0 * 3.14159265358979323846 / 180.0};
  bool allow_predicted_outpost_slots{true};
};

struct ArmorSelectionInput
{
  std::vector<ArmorSelectionCandidate> candidates;
  int locked_index{-1};
  double angular_velocity{0.0};
  double radius{0.0};
  double distance{0.0};
  bool is_outpost{false};
  bool tracking_converged{true};
};

struct ArmorSelectionResult
{
  bool valid{false};
  bool switched{false};
  int index{-1};
  double switch_angle_rad{0.0};
};

double computeArmorJumpAngle(
  double distance,
  double radius,
  double angular_velocity,
  const ArmorSelectionConfig & config);

ArmorSelectionResult selectArmorIndex(
  const ArmorSelectionInput & input,
  const ArmorSelectionConfig & config);

}  // namespace aim_armor_controller
