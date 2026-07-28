#include <gtest/gtest.h>

#include "aim_armor_detector/team_color.hpp"

namespace
{

TEST(TeamColor, RedTeamTargetsBlue)
{
  EXPECT_EQ(aim_armor_detector::detectColorForTeam(true), 0);
}

TEST(TeamColor, BlueTeamTargetsRed)
{
  EXPECT_EQ(aim_armor_detector::detectColorForTeam(false), 1);
}

}  // namespace
