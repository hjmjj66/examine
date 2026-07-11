#include <gtest/gtest.h>

#include <chrono>

#include "aim_outpost_predictor/front_camera_arbitrator.hpp"

namespace aim_outpost_predictor
{
namespace
{

TEST(FrontCameraArbitrator, FrontZeroPriorityFallbackAndRecovery)
{
  const auto start = std::chrono::steady_clock::time_point{};
  FrontCameraArbitrator arbitrator(0.2, start);

  EXPECT_TRUE(arbitrator.shouldProcessFront0(true, start));
  EXPECT_FALSE(arbitrator.shouldProcessFront1(start + std::chrono::milliseconds(199)));
  EXPECT_FALSE(arbitrator.shouldProcessFront0(false, start + std::chrono::milliseconds(200)));
  EXPECT_TRUE(arbitrator.shouldProcessFront1(start + std::chrono::milliseconds(200)));

  EXPECT_TRUE(arbitrator.shouldProcessFront0(true, start + std::chrono::milliseconds(201)));
  EXPECT_FALSE(arbitrator.shouldProcessFront1(start + std::chrono::milliseconds(201)));
}

}  // namespace
}  // namespace aim_outpost_predictor
