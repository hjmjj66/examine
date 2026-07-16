#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>

#include "aim_msgs/msg/armor_set_with_state.hpp"
#include "aim_msgs/msg/outpost_state.hpp"
#include "aim_msgs/msg/target_state.hpp"
#include "aim_armor_controller/outpost_fire_gate.hpp"

namespace aim_armor_controller
{

struct LegacyArmorState
{
  geometry_msgs::msg::Point position;
  double yaw{0.0};
};

struct LegacyTargetModel
{
  std_msgs::msg::Header header;
  std::uint8_t id{0};
  bool jumped{false};
  geometry_msgs::msg::Point center;
  geometry_msgs::msg::Vector3 velocity;
  double yaw{0.0};
  double angular_velocity{0.0};
  double radius{0.0};
  double low_height_offset{0.0};
  double high_height_offset{0.0};
  double radius_offset{0.0};
  double height_offset{0.0};
  int armor_count{4};
  bool has_primary_armor{false};
  int primary_slot{-1};
  std::array<bool, 3> visible_slots{{false, false, false}};
};

struct LegacyAimCandidate
{
  bool valid{false};
  geometry_msgs::msg::Point point_world;
  double armor_yaw_world{0.0};
  int armor_index{0};
};

inline geometry_msgs::msg::Point makeLegacyPoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

inline LegacyTargetModel legacyTargetModelFromArmorSet(
  const aim_msgs::msg::ArmorSetWithState & target)
{
  LegacyTargetModel model;
  model.header = target.header;
  model.id = target.id;
  model.jumped = target.jumped;
  model.center = makeLegacyPoint(target.center_x, target.center_y, target.center_z);
  model.velocity.x = target.velocity_x;
  model.velocity.y = target.velocity_y;
  model.velocity.z = target.velocity_z;
  model.yaw = target.yaw;
  model.angular_velocity = target.angular_velocity;
  model.radius = target.radius;
  model.radius_offset = target.radius_offset;
  model.height_offset = target.height_offset;
  model.armor_count = target.id == 6 ? 3 : std::max<int>(1, static_cast<int>(target.armors.size()));
  model.has_primary_armor = false;
  model.primary_slot = -1;
  if (model.armor_count == 3) {
    model.low_height_offset = -target.height_offset;
    model.high_height_offset = target.height_offset;
  }
  return model;
}

inline LegacyTargetModel legacyTargetModelFromTargetState(
  const std_msgs::msg::Header & header,
  const aim_msgs::msg::TargetState & target)
{
  LegacyTargetModel model;
  model.header = header;
  model.id = target.id;
  model.jumped = target.jumped;
  model.center = target.center;
  model.velocity = target.velocity;
  model.yaw = target.yaw;
  model.angular_velocity = target.angular_velocity;
  model.radius = target.radius;
  model.radius_offset = target.radius_offset;
  model.height_offset = target.height_offset;
  model.armor_count = std::max<int>(1, static_cast<int>(target.predicted_armors.size()));
  model.has_primary_armor = false;
  model.primary_slot = -1;
  return model;
}

inline LegacyTargetModel legacyTargetModelFromOutpostState(const aim_msgs::msg::OutpostState & target)
{
  LegacyTargetModel model;
  model.header = target.header;
  model.id = target.id;
  model.jumped = target.jumped;
  model.center = target.center;
  model.velocity = target.velocity;
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

inline void predictLegacyTarget(LegacyTargetModel & target, double dt)
{
  target.center.x += target.velocity.x * dt;
  target.center.y += target.velocity.y * dt;
  target.center.z += target.velocity.z * dt;
  target.yaw = limitRad(target.yaw + target.angular_velocity * dt);
}

inline std::vector<LegacyArmorState> buildLegacyArmors(const LegacyTargetModel & target)
{
  std::vector<LegacyArmorState> armors;
  const int armor_count = std::max(1, target.armor_count);
  armors.reserve(static_cast<std::size_t>(armor_count));
  for (int i = 0; i < armor_count; ++i) {
    const double yaw = limitRad(target.yaw + static_cast<double>(i) * 2.0 * kPi / armor_count);
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

    LegacyArmorState armor;
    armor.yaw = yaw;
    armor.position = makeLegacyPoint(
      target.center.x - radius * std::cos(yaw),
      target.center.y - radius * std::sin(yaw),
      z);
    armors.push_back(armor);
  }
  return armors;
}

inline double legacyArmorFacingError(const LegacyTargetModel & target, const LegacyArmorState & armor)
{
  const double center_yaw = std::atan2(target.center.y, target.center.x);
  return limitRad(armor.yaw - center_yaw);
}

inline double legacyTimeUntilFace(double facing_error, double angular_velocity)
{
  if (std::abs(angular_velocity) < 1e-6) {
    return std::numeric_limits<double>::infinity();
  }

  if (angular_velocity > 0.0) {
    const double angle_to_face = facing_error <= 0.0 ? -facing_error : 2.0 * kPi - facing_error;
    return angle_to_face / angular_velocity;
  }

  const double angle_to_face = facing_error >= 0.0 ? facing_error : 2.0 * kPi + facing_error;
  return angle_to_face / -angular_velocity;
}

inline LegacyAimCandidate makeLegacyAimCandidate(const LegacyTargetModel & target, int armor_index)
{
  LegacyAimCandidate candidate;
  const auto armors = buildLegacyArmors(target);
  if (armor_index < 0 || armor_index >= static_cast<int>(armors.size())) {
    return candidate;
  }

  candidate.valid = true;
  candidate.point_world = armors[static_cast<std::size_t>(armor_index)].position;
  candidate.armor_yaw_world = armors[static_cast<std::size_t>(armor_index)].yaw;
  candidate.armor_index = armor_index;
  return candidate;
}

inline std::optional<int> chooseLegacyOutpostArmorIndex(
  const LegacyTargetModel & target, double facing_gate_rad = 0.0)
{
  const auto armors = buildLegacyArmors(target);
  if (armors.empty()) {
    return std::nullopt;
  }

  int best_index = 0;
  double best_yaw_error = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < armors.size(); ++i) {
    if (i >= target.visible_slots.size() || !target.visible_slots[i]) {
      continue;
    }
    const double yaw_error = std::abs(legacyArmorFacingError(target, armors[i]));
    if (yaw_error < best_yaw_error) {
      best_yaw_error = yaw_error;
      best_index = static_cast<int>(i);
    }
  }
  if (!std::isfinite(best_yaw_error) ||
      (facing_gate_rad > 0.0 && best_yaw_error > facing_gate_rad)) {
    return std::nullopt;
  }
  return best_index;
}

inline std::optional<int> chooseLegacyLowSpeedArmorIndex(
  const LegacyTargetModel & target,
  double & lock_index)
{
  const auto armors = buildLegacyArmors(target);
  if (armors.empty()) {
    return std::nullopt;
  }

  const double center_yaw = std::atan2(target.center.y, target.center.x);
  std::vector<double> delta_angle_list;
  delta_angle_list.reserve(armors.size());
  for (const auto & armor : armors) {
    delta_angle_list.push_back(limitRad(armor.yaw - center_yaw));
  }

  std::vector<int> id_list;
  for (std::size_t i = 0; i < armors.size(); ++i) {
    if (std::abs(delta_angle_list[i]) > 60.0 * kPi / 180.0) {
      continue;
    }
    id_list.push_back(static_cast<int>(i));
  }

  if (id_list.empty()) {
    return std::nullopt;
  }

  int selected_index = id_list.front();
  if (id_list.size() > 1U) {
    const int id0 = id_list[0];
    const int id1 = id_list[1];
    if (lock_index != id0 && lock_index != id1) {
      lock_index =
        std::abs(delta_angle_list[static_cast<std::size_t>(id0)]) <
        std::abs(delta_angle_list[static_cast<std::size_t>(id1)]) ? id0 : id1;
    }
    selected_index = static_cast<int>(lock_index);
  } else {
    lock_index = -1.0;
  }

  return selected_index;
}

inline LegacyAimCandidate chooseLegacyAimPoint(
  const LegacyTargetModel & target,
  const LegacyTargetModel * low_speed_selection_target,
  double & lock_index,
  bool enable_smart_selector,
  double smart_selector_max_angular_velocity,
  double comming_angle_rad,
  double leaving_angle_rad)
{
  LegacyAimCandidate candidate;
  const auto armors = buildLegacyArmors(target);
  if (armors.empty()) {
    return candidate;
  }

  if (target.armor_count == 3) {
    const LegacyTargetModel & selection_target =
      low_speed_selection_target != nullptr ? *low_speed_selection_target : target;
    const auto selected_index = chooseLegacyOutpostArmorIndex(selection_target);
    if (!selected_index.has_value()) {
      return candidate;
    }
    return makeLegacyAimCandidate(target, *selected_index);
  }

  if (!target.jumped) {
    candidate.valid = true;
    candidate.point_world = armors.front().position;
    candidate.armor_yaw_world = armors.front().yaw;
    candidate.armor_index = 0;
    return candidate;
  }

  if (!enable_smart_selector ||
    std::abs(target.angular_velocity) <= std::abs(smart_selector_max_angular_velocity))
  {
    const LegacyTargetModel & selection_target =
      low_speed_selection_target != nullptr ? *low_speed_selection_target : target;
    const auto selected_index = chooseLegacyLowSpeedArmorIndex(selection_target, lock_index);
    if (!selected_index.has_value()) {
      return candidate;
    }
    return makeLegacyAimCandidate(target, *selected_index);
  }

  const double center_yaw = std::atan2(target.center.y, target.center.x);
  std::vector<double> delta_angle_list;
  delta_angle_list.reserve(armors.size());
  for (const auto & armor : armors) {
    delta_angle_list.push_back(limitRad(armor.yaw - center_yaw));
  }

  struct SpinningCandidate
  {
    std::size_t index;
    double delta_angle;
    bool coming_side;
  };

  std::vector<SpinningCandidate> spinning_candidates;
  spinning_candidates.reserve(armors.size());
  for (std::size_t i = 0; i < armors.size(); ++i) {
    if (std::abs(delta_angle_list[i]) > comming_angle_rad) {
      continue;
    }

    const double delta_angle = delta_angle_list[i];
    const bool inside_leaving_window = target.angular_velocity > 0.0 ?
      delta_angle < leaving_angle_rad :
      delta_angle > -leaving_angle_rad;
    if (!inside_leaving_window) {
      continue;
    }

    spinning_candidates.push_back(
      SpinningCandidate{
        i,
        delta_angle,
        delta_angle * target.angular_velocity < 0.0});
  }

  if (spinning_candidates.empty()) {
    return candidate;
  }

  const auto best_spinning_candidate = std::min_element(
    spinning_candidates.begin(), spinning_candidates.end(),
    [](const SpinningCandidate & lhs, const SpinningCandidate & rhs) {
      if (lhs.coming_side != rhs.coming_side) {
        return lhs.coming_side;
      }
      return std::abs(lhs.delta_angle) < std::abs(rhs.delta_angle);
    });

  return makeLegacyAimCandidate(target, static_cast<int>(best_spinning_candidate->index));
}

inline LegacyAimCandidate chooseLegacySpinCenterAimPoint(
  const LegacyTargetModel & target,
  double shoot_face_tolerance_rad)
{
  LegacyAimCandidate candidate;
  const auto armors = buildLegacyArmors(target);
  if (armors.empty()) {
    return candidate;
  }

  std::size_t selected_index = 0U;
  double best_time_until_face = std::numeric_limits<double>::infinity();
  double best_facing_error = std::numeric_limits<double>::infinity();
  bool found_inside_face_window = false;
  for (std::size_t i = 0; i < armors.size(); ++i) {
    const double facing_error = legacyArmorFacingError(target, armors[i]);
    if (std::abs(facing_error) < shoot_face_tolerance_rad) {
      if (!found_inside_face_window || std::abs(facing_error) < std::abs(best_facing_error)) {
        selected_index = i;
        best_facing_error = facing_error;
        found_inside_face_window = true;
      }
      continue;
    }
    if (found_inside_face_window) {
      continue;
    }

    const double time_until_face = legacyTimeUntilFace(facing_error, target.angular_velocity);
    if (time_until_face < best_time_until_face ||
      (std::abs(time_until_face - best_time_until_face) < 1e-6 &&
      std::abs(facing_error) < std::abs(best_facing_error)))
    {
      selected_index = i;
      best_time_until_face = time_until_face;
      best_facing_error = facing_error;
    }
  }

  candidate.valid = true;
  candidate.point_world =
    makeLegacyPoint(target.center.x, target.center.y, armors[selected_index].position.z);
  candidate.armor_yaw_world = armors[selected_index].yaw;
  candidate.armor_index = static_cast<int>(selected_index);
  return candidate;
}

}  // namespace aim_armor_controller
