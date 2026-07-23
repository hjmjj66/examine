#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>

#include "aim_msgs/msg/target_state.hpp"
#include "aim_msgs/msg/target_state_array.hpp"

namespace aim_armor_controller
{

struct TargetStateMatch
{
  std_msgs::msg::Header header;
  aim_msgs::msg::TargetState target;
  bool valid{false};
};

inline bool isZeroStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}

inline bool isTargetStateReadyForControl(const aim_msgs::msg::TargetState & target)
{
  return target.tracking && target.converged;
}

inline bool isNewerStamp(
  const builtin_interfaces::msg::Time & lhs,
  const builtin_interfaces::msg::Time & rhs)
{
  if (lhs.sec != rhs.sec) {
    return lhs.sec > rhs.sec;
  }
  return lhs.nanosec > rhs.nanosec;
}

inline double centerDistanceSq(const geometry_msgs::msg::Point & center)
{
  return center.x * center.x + center.y * center.y + center.z * center.z;
}

inline double measurementAgeSec(
  const builtin_interfaces::msg::Time & stamp_sec,
  double now_sec)
{
  if (isZeroStamp(stamp_sec)) {
    return 0.0;
  }
  return std::max(0.0, now_sec - static_cast<double>(stamp_sec.sec) -
    static_cast<double>(stamp_sec.nanosec) * 1e-9);
}

inline bool preferTargetStateMatch(
  const TargetStateMatch & candidate,
  const TargetStateMatch & current_best)
{
  if (!current_best.valid) {
    return candidate.valid;
  }
  if (!candidate.valid) {
    return false;
  }
  if (candidate.target.tracking != current_best.target.tracking) {
    return candidate.target.tracking;
  }
  if (candidate.target.converged != current_best.target.converged) {
    return candidate.target.converged;
  }
  if (isNewerStamp(candidate.header.stamp, current_best.header.stamp)) {
    return true;
  }
  if (isNewerStamp(current_best.header.stamp, candidate.header.stamp)) {
    return false;
  }
  return centerDistanceSq(candidate.target.center) < centerDistanceSq(current_best.target.center);
}

inline void updateBestTargetStateMatch(
  const aim_msgs::msg::TargetStateArray & array,
  std::uint8_t selected_id,
  TargetStateMatch & best_match)
{
  for (const auto & target : array.targets) {
    if (target.id != selected_id) {
      continue;
    }
    TargetStateMatch candidate;
    candidate.header = array.header;
    candidate.target = target;
    candidate.valid = true;
    if (preferTargetStateMatch(candidate, best_match)) {
      best_match = candidate;
    }
  }
}

inline std::optional<TargetStateMatch> selectBestTargetStateMatch(
  std::uint8_t selected_id,
  const aim_msgs::msg::TargetStateArray * front_0,
  const aim_msgs::msg::TargetStateArray * front_1,
  const aim_msgs::msg::TargetStateArray * back)
{
  TargetStateMatch best_match;
  if (front_0 != nullptr) {
    updateBestTargetStateMatch(*front_0, selected_id, best_match);
  }
  if (front_1 != nullptr) {
    updateBestTargetStateMatch(*front_1, selected_id, best_match);
  }
  if (back != nullptr) {
    updateBestTargetStateMatch(*back, selected_id, best_match);
  }
  if (!best_match.valid) {
    return std::nullopt;
  }
  return best_match;
}

}  // namespace aim_armor_controller
