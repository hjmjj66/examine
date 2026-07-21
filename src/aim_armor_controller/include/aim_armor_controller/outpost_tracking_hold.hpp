#ifndef AIM_ARMOR_CONTROLLER__OUTPOST_TRACKING_HOLD_HPP_
#define AIM_ARMOR_CONTROLLER__OUTPOST_TRACKING_HOLD_HPP_

namespace aim_armor_controller
{

inline bool shouldHoldOutpostTarget(
  bool selected_outpost, bool has_cached_target, double elapsed_sec, double hold_timeout_sec)
{
  return selected_outpost && has_cached_target && hold_timeout_sec > 0.0 &&
         elapsed_sec >= 0.0 && elapsed_sec <= hold_timeout_sec;
}

}  // namespace aim_armor_controller

#endif  // AIM_ARMOR_CONTROLLER__OUTPOST_TRACKING_HOLD_HPP_
