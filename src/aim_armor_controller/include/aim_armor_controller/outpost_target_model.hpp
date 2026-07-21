#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

#include "aim_msgs/msg/armor.hpp"
#include "aim_msgs/msg/armor_set_with_state.hpp"
#include "aim_msgs/msg/outpost_state.hpp"
#include "aim_armor_controller/outpost_fire_gate.hpp"

namespace aim_armor_controller
{

struct OutpostArmorState
{
  geometry_msgs::msg::Point position;
  double yaw{0.0};
};

struct OutpostTargetModel
{
  geometry_msgs::msg::Point center;
  double yaw{0.0};
  double angular_velocity{0.0};
  double radius{0.0};
  double low_height_offset{0.0};
  double high_height_offset{0.0};
  int armor_count{3};
};

inline geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

inline geometry_msgs::msg::Point armorCenter(const aim_msgs::msg::Armor & armor)
{
  geometry_msgs::msg::Point center;
  for (const auto & pt : armor.corners) {
    center.x += pt.x;
    center.y += pt.y;
    center.z += pt.z;
  }
  center.x *= 0.25;
  center.y *= 0.25;
  center.z *= 0.25;
  return center;
}

inline OutpostTargetModel outpostTargetModelFromArmorSet(const aim_msgs::msg::ArmorSetWithState & armor_set)
{
  OutpostTargetModel model;
  model.center = makePoint(armor_set.center_x, armor_set.center_y, armor_set.center_z);
  model.yaw = armor_set.yaw;
  model.angular_velocity = armor_set.angular_velocity;
  model.radius = armor_set.radius;
  model.low_height_offset = -armor_set.height_offset;
  model.high_height_offset = armor_set.height_offset;

  if (armor_set.armors.size() >= 3) {
    double avg_radius = 0.0;
    std::array<double, 3> z_values{};
    for (std::size_t i = 0; i < 3; ++i) {
      const auto center = armorCenter(armor_set.armors[i]);
      const double dx = center.x - armor_set.center_x;
      const double dy = center.y - armor_set.center_y;
      avg_radius += std::hypot(dx, dy);
      z_values[i] = center.z;
    }
    model.radius = avg_radius / 3.0;
    model.low_height_offset = z_values[0] - armor_set.center_z;
    model.high_height_offset = z_values[2] - armor_set.center_z;
  }

  return model;
}

inline OutpostTargetModel outpostTargetModelFromState(const aim_msgs::msg::OutpostState & state)
{
  OutpostTargetModel model;
  model.center = state.center;
  model.yaw = state.yaw;
  model.angular_velocity = state.angular_velocity;
  model.radius = state.radius;
  model.low_height_offset = state.low_height_offset;
  model.high_height_offset = state.high_height_offset;
  model.armor_count = 3;
  return model;
}

inline void predictOutpostTarget(OutpostTargetModel & target, double dt)
{
  target.yaw = limitRad(target.yaw + target.angular_velocity * dt);
}

inline std::vector<OutpostArmorState> buildOutpostArmors(const OutpostTargetModel & target)
{
  std::vector<OutpostArmorState> armors;
  const int armor_count = std::max(1, target.armor_count);
  armors.reserve(static_cast<std::size_t>(armor_count));
  for (int i = 0; i < armor_count; ++i) {
    const double yaw = limitRad(target.yaw + static_cast<double>(i) * 2.0 * kPi / armor_count);
    double z = target.center.z;
    if (armor_count == 3) {
      if (i == 0) {
        z += target.low_height_offset;
      } else if (i == 2) {
        z += target.high_height_offset;
      }
    }

    OutpostArmorState armor;
    armor.yaw = yaw;
    armor.position = makePoint(
      target.center.x - target.radius * std::cos(yaw),
      target.center.y - target.radius * std::sin(yaw),
      z);
    armors.push_back(armor);
  }
  return armors;
}

inline double outpostArmorFacingError(
  const OutpostTargetModel & target,
  const OutpostArmorState & armor)
{
  const double center_yaw = std::atan2(target.center.y, target.center.x);
  return limitRad(armor.yaw - center_yaw);
}

inline std::optional<int> chooseOutpostArmorIndexAtImpact(const OutpostTargetModel & target)
{
  const std::vector<OutpostArmorState> armors = buildOutpostArmors(target);
  if (armors.empty()) {
    return std::nullopt;
  }

  int best_index = 0;
  double best_yaw_error = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < armors.size(); ++i) {
    const double yaw_error = std::abs(outpostArmorFacingError(target, armors[i]));
    if (yaw_error < best_yaw_error) {
      best_yaw_error = yaw_error;
      best_index = static_cast<int>(i);
    }
  }
  return best_index;
}

}  // namespace aim_armor_controller
