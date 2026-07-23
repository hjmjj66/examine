#include <cstdint>

#include <gtest/gtest.h>

#include "aim_armor_controller/fire_code_state.hpp"
#include "aim_armor_controller/legacy_fire_control.hpp"

namespace
{

TEST(FireCodeState, TogglesFireStatusBetweenZeroAndThree)
{
  aim_armor_controller::FireCodeState state;

  EXPECT_EQ(state.fire_status, 0U);
  aim_armor_controller::toggleFireStatus(state);
  EXPECT_EQ(state.fire_status, 3U);
  aim_armor_controller::toggleFireStatus(state);
  EXPECT_EQ(state.fire_status, 0U);
}

TEST(FireCodeState, ToggleDoesNotChangeOtherSemanticFields)
{
  aim_armor_controller::FireCodeState state;
  state.cap_state = 2U;
  state.follow_mode = true;
  state.aim_mode = true;
  state.rotate = 3U;

  aim_armor_controller::toggleFireStatus(state);

  EXPECT_EQ(state.fire_status, 3U);
  EXPECT_EQ(state.cap_state, 2U);
  EXPECT_TRUE(state.follow_mode);
  EXPECT_TRUE(state.aim_mode);
  EXPECT_EQ(state.rotate, 3U);
}

TEST(FireCodeState, RawEncodingKeepsFireStatusAndOtherFieldsSeparate)
{
  aim_armor_controller::FireCodeState state;
  state.fire_status = 3U;
  state.cap_state = 0U;
  state.follow_mode = false;
  state.aim_mode = true;
  state.rotate = 1U;

  EXPECT_EQ(aim_armor_controller::encodeFireCodeRaw(state), 0x63U);

  aim_armor_controller::toggleFireStatus(state);
  EXPECT_EQ(aim_armor_controller::encodeFireCodeRaw(state), 0x60U);
}

TEST(LegacyFireControl, ResetsArmorAfterLeavingFaceWindow)
{
  aim_armor_controller::LegacyFireControlState state;
  const aim_armor_controller::LegacyFireControlInput input{
    1, 0.0, 0.0, 1.0, 1.0, true, true, 0U};

  EXPECT_TRUE(aim_armor_controller::evaluateLegacyFireControl(input, state).shoot_flag);
  EXPECT_FALSE(aim_armor_controller::evaluateLegacyFireControl(input, state).shoot_flag);

  auto outside_window = input;
  outside_window.inside_face_window = false;
  EXPECT_FALSE(
    aim_armor_controller::evaluateLegacyFireControl(outside_window, state).shoot_flag);

  EXPECT_TRUE(aim_armor_controller::evaluateLegacyFireControl(input, state).shoot_flag);
}
}  // namespace
