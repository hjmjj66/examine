#include "aim_armor_controller/mpc_target_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "aim_armor_controller/armor_selection_policy.hpp"

namespace aim_armor_controller
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

ArmorSelectionConfig defaultSelectionConfig(bool allow_predicted_outpost_slots)
{
  ArmorSelectionConfig config;
  config.allow_predicted_outpost_slots = allow_predicted_outpost_slots;
  return config;
}

}  // namespace

double mpcLimitRad(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double mpcRadToDeg(double angle)
{
  return angle * 180.0 / kPi;
}

double mpcDegToRad(double angle)
{
  return angle * kPi / 180.0;
}

TargetModel makeTargetModel(const aim_msgs::msg::TargetState & target)
{
  TargetModel model;
  model.header = target.header;
  model.id = target.id;
  model.tracking = target.tracking;
  model.converged = target.converged;
  model.jumped = target.jumped;
  model.center = target.center;
  model.velocity = target.velocity;
  model.yaw = target.yaw;
  model.angular_velocity = target.angular_velocity;
  model.radius = target.radius;
  model.radius_offset = target.radius_offset;
  model.height_offset = target.height_offset;
  model.armor_count = 4;
  return model;
}

TargetModel makeTargetModel(const aim_msgs::msg::OutpostState & target)
{
  TargetModel model;
  model.header = target.header;
  model.id = target.id;
  model.tracking = target.tracking;
  model.converged = target.converged;
  model.jumped = target.jumped;
  model.center = target.center;
  model.yaw = target.yaw;
  model.angular_velocity = target.angular_velocity;
  model.radius = target.radius;
  model.low_height_offset = target.low_height_offset;
  model.high_height_offset = target.high_height_offset;
  model.armor_count = 3;
  model.has_primary_armor = target.has_primary_armor;
  model.primary_slot = target.primary_slot;
  model.visible_slots = target.visible_slots;
  return model;
}

void predictTarget(TargetModel & target, double dt)
{
  target.center.x += target.velocity.x * dt;
  target.center.y += target.velocity.y * dt;
  target.center.z += target.velocity.z * dt;
  target.yaw = mpcLimitRad(target.yaw + target.angular_velocity * dt);
}

std::vector<ArmorState> buildArmors(const TargetModel & target)
{
  std::vector<ArmorState> armors;
  const int armor_count = std::max(1, target.armor_count);
  armors.reserve(static_cast<std::size_t>(armor_count));
  for (int i = 0; i < armor_count; ++i) {
    const double yaw = mpcLimitRad(target.yaw + static_cast<double>(i) * 2.0 * kPi / armor_count);
    double radius = target.radius;
    double z = target.center.z;
    if (armor_count == 4) {
      const bool use_offset = i == 1 || i == 3;
      radius = use_offset ? target.radius + target.radius_offset : target.radius;
      z = use_offset ? target.center.z + target.height_offset : target.center.z;
    } else if (armor_count == 3) {
      if (i == 0) {
        z += target.low_height_offset;
      } else if (i == 2) {
        z += target.high_height_offset;
      }
    }

    ArmorState armor;
    armor.yaw = yaw;
    armor.position = makePoint(
      target.center.x - radius * std::cos(yaw), target.center.y - radius * std::sin(yaw), z);
    armors.push_back(armor);
  }
  return armors;
}

double armorFacingError(const TargetModel & target, const ArmorState & armor)
{
  const double center_yaw = std::atan2(target.center.y, target.center.x);
  return mpcLimitRad(armor.yaw - center_yaw);
}

namespace
{

ArmorSelectionInput makeSelectionInput(const TargetModel & target, int locked_index)
{
  const auto armors = buildArmors(target);
  ArmorSelectionInput input;
  input.locked_index = locked_index;
  input.angular_velocity = target.angular_velocity;
  input.radius = target.radius;
  input.distance = std::hypot(target.center.x, target.center.y);
  input.is_outpost = target.armor_count == 3;
  input.tracking_converged = target.converged;
  input.candidates.reserve(armors.size());
  for (std::size_t i = 0; i < armors.size(); ++i) {
    ArmorSelectionCandidate candidate;
    candidate.facing_error = armorFacingError(target, armors[i]);
    candidate.observed =
      !input.is_outpost ||
      (i < target.visible_slots.size() && target.visible_slots[i]);
    input.candidates.push_back(candidate);
  }
  return input;
}

}  // namespace

std::optional<int> chooseOutpostArmorIndex(const TargetModel & target)
{
  const ArmorSelectionConfig config = defaultSelectionConfig(true);
  const auto result = selectArmorIndex(makeSelectionInput(target, -1), config);
  if (!result.valid) {
    return std::nullopt;
  }
  return result.index;
}

std::optional<int> chooseLowSpeedArmorIndex(const TargetModel & target, double & lock_index)
{
  const ArmorSelectionConfig config = defaultSelectionConfig(true);
  const auto result = selectArmorIndex(
    makeSelectionInput(target, static_cast<int>(lock_index)), config);
  if (!result.valid) {
    return std::nullopt;
  }
  lock_index = static_cast<double>(result.index);
  return result.index;
}

AimCandidate makeAimCandidate(const TargetModel & target, int armor_index)
{
  AimCandidate candidate;
  const std::vector<ArmorState> armors = buildArmors(target);
  if (armor_index < 0 || armor_index >= static_cast<int>(armors.size())) {
    return candidate;
  }

  candidate.valid = true;
  candidate.point_world = armors[static_cast<std::size_t>(armor_index)].position;
  candidate.armor_yaw_world = armors[static_cast<std::size_t>(armor_index)].yaw;
  candidate.armor_index = armor_index;
  return candidate;
}

AimCandidate chooseAimPoint(
  const TargetModel & target,
  double & lock_index,
  const AimSelectionConfig & config,
  const TargetModel * low_speed_selection_target)
{
  (void)low_speed_selection_target;
  if (buildArmors(target).empty()) {
    AimCandidate invalid_candidate;
    return invalid_candidate;
  }

  ArmorSelectionConfig selection_config;
  selection_config.enable_smart_selector = config.enable_smart_selector;
  selection_config.coming_angle_rad = config.coming_angle_rad;
  selection_config.leaving_angle_rad = config.leaving_angle_rad;
  selection_config.response_speed_rad_s = config.selector_response_speed_rad_s;
  selection_config.min_angular_velocity_rad_s = config.selector_min_angular_velocity_rad_s;
  selection_config.min_switch_angle_rad = config.selector_min_switch_angle_rad;
  selection_config.allow_predicted_outpost_slots = config.allow_predicted_outpost_slots;

  const auto result = selectArmorIndex(
    makeSelectionInput(target, static_cast<int>(lock_index)), selection_config);
  if (!result.valid) {
    AimCandidate invalid_candidate;
    return invalid_candidate;
  }

  lock_index = static_cast<double>(result.index);
  return makeAimCandidate(target, result.index);
}

}  // namespace aim_armor_controller
