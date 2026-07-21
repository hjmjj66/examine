#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>

#include "aim_msgs/msg/outpost_state.hpp"
#include "aim_msgs/msg/target_state.hpp"

namespace aim_armor_controller
{

struct ArmorState
{
  geometry_msgs::msg::Point position;
  double yaw{0.0};
};

struct TargetModel
{
  std_msgs::msg::Header header;
  std::uint8_t id{0};
  bool tracking{true};
  bool converged{false};
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

struct AimCandidate
{
  bool valid{false};
  bool fire_allowed{true};
  geometry_msgs::msg::Point point_world;
  double armor_yaw_world{0.0};
  int armor_index{0};
};

struct AimSelectionConfig
{
  bool enable_smart_selector{true};
  double low_speed_angular_velocity_threshold{2.0};
  double coming_angle_rad{60.0 * 3.14159265358979323846 / 180.0};
  double leaving_angle_rad{20.0 * 3.14159265358979323846 / 180.0};
  double selector_response_speed_rad_s{0.01};
  double selector_min_angular_velocity_rad_s{0.6};
  double selector_min_switch_angle_rad{30.0 * 3.14159265358979323846 / 180.0};
  bool allow_predicted_outpost_slots{true};

};

TargetModel makeTargetModel(const aim_msgs::msg::TargetState & target);
TargetModel makeTargetModel(const aim_msgs::msg::OutpostState & target);

void predictTarget(TargetModel & target, double dt);
std::vector<ArmorState> buildArmors(const TargetModel & target);
double armorFacingError(const TargetModel & target, const ArmorState & armor);
std::optional<int> chooseOutpostArmorIndex(const TargetModel & target);
std::optional<int> chooseLowSpeedArmorIndex(const TargetModel & target, double & lock_index);
AimCandidate makeAimCandidate(const TargetModel & target, int armor_index);
AimCandidate chooseAimPoint(
  const TargetModel & target,
  double & lock_index,
  const AimSelectionConfig & config,
  const TargetModel * low_speed_selection_target = nullptr);

double mpcLimitRad(double angle);
double mpcRadToDeg(double angle);
double mpcDegToRad(double angle);

}  // namespace aim_armor_controller
