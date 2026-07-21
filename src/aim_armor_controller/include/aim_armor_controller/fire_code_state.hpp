#ifndef AIM_ARMOR_CONTROLLER__FIRE_CODE_STATE_HPP_
#define AIM_ARMOR_CONTROLLER__FIRE_CODE_STATE_HPP_

#include <cstdint>

namespace aim_armor_controller
{

constexpr std::uint8_t kFireStatusIdle = 0U;
constexpr std::uint8_t kFireStatusFire = 3U;

struct FireCodeState
{
  std::uint8_t fire_status{kFireStatusIdle};
  std::uint8_t cap_state{0U};
  bool follow_mode{false};
  bool aim_mode{false};
  std::uint8_t rotate{0U};
};

inline void toggleFireStatus(FireCodeState & state)
{
  state.fire_status = state.fire_status == kFireStatusFire ?
    kFireStatusIdle : kFireStatusFire;
}

inline std::uint8_t encodeFireCodeRaw(const FireCodeState & state)
{
  return static_cast<std::uint8_t>(
    (state.fire_status & 0x03U) |
    ((state.cap_state & 0x03U) << 2U) |
    (state.follow_mode ? 0x10U : 0U) |
    (state.aim_mode ? 0x20U : 0U) |
    ((state.rotate & 0x03U) << 6U));
}

}  // namespace aim_armor_controller

#endif  // AIM_ARMOR_CONTROLLER__FIRE_CODE_STATE_HPP_
