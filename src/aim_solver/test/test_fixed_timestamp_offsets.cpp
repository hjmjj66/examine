#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "aim_solver/fixed_timestamp_offsets.hpp"

namespace
{

TEST(FixedTimestampOffsets, LoadsAllCameraOffsetsByParameterName)
{
  const auto offsets = aim_solver::loadFixedTimestampOffsets(
    [](const std::string & name) -> std::optional<double> {
      if (name == "front_tf_timestamp_offset_sec") {
        return -0.024;
      }
      if (name == "front_1_tf_timestamp_offset_sec") {
        return -0.026;
      }
      if (name == "back_tf_timestamp_offset_sec") {
        return -0.024;
      }
      return std::nullopt;
    });

  EXPECT_DOUBLE_EQ(offsets.front_sec, -0.024);
  EXPECT_DOUBLE_EQ(offsets.front_1_sec, -0.026);
  EXPECT_DOUBLE_EQ(offsets.back_sec, -0.024);
}

TEST(FixedTimestampOffsets, RejectsMissingCameraOffset)
{
  EXPECT_THROW(
    aim_solver::loadFixedTimestampOffsets(
      [](const std::string &) -> std::optional<double> {
        return std::nullopt;
      }),
    std::invalid_argument);
}

TEST(FixedTimestampOffsets, RejectsNonFiniteCameraOffset)
{
  EXPECT_THROW(
    aim_solver::loadFixedTimestampOffsets(
      [](const std::string &) -> std::optional<double> {
        return std::numeric_limits<double>::quiet_NaN();
      }),
    std::invalid_argument);
}

}  // namespace
