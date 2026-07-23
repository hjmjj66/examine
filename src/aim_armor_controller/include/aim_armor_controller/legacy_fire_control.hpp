#pragma once

#include <array>
#include <cstddef>

namespace aim_armor_controller
{

struct LegacyFireControlInput
{
  int target_id{-1};
  double yaw_error_deg{0.0};
  double pitch_error_deg{0.0};
  double shoot_yaw_tolerance_deg{0.0};
  double shoot_pitch_tolerance_deg{0.0};
  bool inside_face_window{false};
  bool spin_center_aim{false};
  std::size_t armor_index{0};
};

struct LegacyFireControlState
{
  int last_fire_target_id{-1};
  std::array<bool, 4> armor_fired_in_face_window{{false, false, false, false}};
};

struct LegacyFireControlOutput
{
  bool gimbal_ready{false};
  bool shoot_flag{false};
};

inline LegacyFireControlOutput evaluateLegacyFireControl(
  const LegacyFireControlInput & input,
  LegacyFireControlState & state)
{
  LegacyFireControlOutput output;
  output.gimbal_ready =
    input.yaw_error_deg < input.shoot_yaw_tolerance_deg &&
    input.pitch_error_deg < input.shoot_pitch_tolerance_deg;
  output.shoot_flag = output.gimbal_ready;

  if (!input.spin_center_aim) {
    state.armor_fired_in_face_window.fill(false);
    return output;
  }

  if (state.last_fire_target_id != input.target_id) {
    state.armor_fired_in_face_window.fill(false);
    state.last_fire_target_id = input.target_id;
  }

  output.shoot_flag = false;
  if (!input.inside_face_window) {
    state.armor_fired_in_face_window.fill(false);
    return output;
  }

  if (!output.gimbal_ready) {
    return output;
  }

  if (input.armor_index >= state.armor_fired_in_face_window.size()) {
    return output;
  }

  if (!state.armor_fired_in_face_window[input.armor_index]) {
    state.armor_fired_in_face_window[input.armor_index] = true;
    output.shoot_flag = true;
  }
  return output;
}

}  // namespace aim_armor_controller
