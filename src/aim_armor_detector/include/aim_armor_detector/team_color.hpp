#pragma once

namespace aim_armor_detector
{

inline constexpr int kUnknownDetectColor = -1;

inline constexpr int detectColorForTeam(bool is_team_red) noexcept
{
  // Model color ids: 0 is blue and 1 is red.
  return is_team_red ? 0 : 1;
}

}  // namespace aim_armor_detector
