#ifndef AIM_ARMOR_CONTROLLER__CONTROL_TIME_ALIGNMENT_HPP_
#define AIM_ARMOR_CONTROLLER__CONTROL_TIME_ALIGNMENT_HPP_

#include <algorithm>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>

namespace aim_armor_controller
{

inline bool isZeroControlStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0U;
}

inline rclcpp::Time resolveControlStamp(
  const builtin_interfaces::msg::Time & gimbal_stamp,
  const rclcpp::Time & fallback_now)
{
  return isZeroControlStamp(gimbal_stamp) ? fallback_now : rclcpp::Time(gimbal_stamp);
}

inline double ageAtControlStamp(
  const builtin_interfaces::msg::Time & measurement_stamp,
  const rclcpp::Time & control_stamp)
{
  if (isZeroControlStamp(measurement_stamp)) {
    return 0.0;
  }
  return std::max(0.0, (control_stamp - rclcpp::Time(measurement_stamp)).seconds());
}

}  // namespace aim_armor_controller

#endif  // AIM_ARMOR_CONTROLLER__CONTROL_TIME_ALIGNMENT_HPP_
