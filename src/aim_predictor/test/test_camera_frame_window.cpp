#include "aim_predictor/camera_frame_window.hpp"

#include <chrono>
#include <string>

#include <gtest/gtest.h>

namespace aim_predictor
{

namespace
{

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

struct TestFrame
{
  std::string source;
  std::int64_t stamp{0};
  std::uint64_t arrival_sequence{0};
  Clock::time_point arrival_time{};
};

TEST(CameraFrameWindow, SortsFramesByStampAndUsesArrivalSequenceAsTieBreaker)
{
  CameraFrameWindow<TestFrame> window(1ms, 16);
  const auto start = Clock::time_point{};

  window.enqueue(TestFrame{"front_0", 200, 0, start});
  window.enqueue(TestFrame{"front_1", 100, 1, start + 500us});
  window.enqueue(TestFrame{"back", 100, 2, start + 700us});

  EXPECT_FALSE(window.takeReady(start + 999us).has_value());
  const auto batch = window.takeReady(start + 1ms);
  ASSERT_TRUE(batch.has_value());
  ASSERT_EQ(batch->size(), 3U);
  EXPECT_EQ(batch->at(0).source, "front_1");
  EXPECT_EQ(batch->at(1).source, "back");
  EXPECT_EQ(batch->at(2).source, "front_0");
}

TEST(CameraFrameWindow, DropsOldestPendingFrameWhenCapacityIsFull)
{
  CameraFrameWindow<TestFrame> window(1ms, 2);
  const auto start = Clock::time_point{};

  EXPECT_FALSE(window.enqueue(TestFrame{"a", 1, 0, start}));
  EXPECT_FALSE(window.enqueue(TestFrame{"b", 2, 1, start + 100us}));
  EXPECT_TRUE(window.enqueue(TestFrame{"c", 3, 2, start + 200us}));

  const auto batch = window.takeReady(start + 1ms);
  ASSERT_TRUE(batch.has_value());
  ASSERT_EQ(batch->size(), 2U);
  EXPECT_EQ(batch->at(0).source, "b");
  EXPECT_EQ(batch->at(1).source, "c");
}

TEST(CameraFrameWindow, KeepsFramesAfterTheFirstWindowForTheNextBatch)
{
  CameraFrameWindow<TestFrame> window(1ms, 16);
  const auto start = Clock::time_point{};

  window.enqueue(TestFrame{"a", 100, 0, start});
  window.enqueue(TestFrame{"b", 200, 1, start + 500us});
  window.enqueue(TestFrame{"c", 300, 2, start + 1500us});

  const auto first_batch = window.takeReady(start + 1ms);
  ASSERT_TRUE(first_batch.has_value());
  ASSERT_EQ(first_batch->size(), 2U);
  EXPECT_EQ(first_batch->at(0).source, "a");
  EXPECT_EQ(first_batch->at(1).source, "b");

  EXPECT_FALSE(window.takeReady(start + 2ms).has_value());
  const auto second_batch = window.takeReady(start + 2500us);
  ASSERT_TRUE(second_batch.has_value());
  ASSERT_EQ(second_batch->size(), 1U);
  EXPECT_EQ(second_batch->at(0).source, "c");
}

TEST(CameraFrameWindow, EmptyWindowDoesNotProduceAReadyBatch)
{
  CameraFrameWindow<TestFrame> window(1ms, 16);
  EXPECT_FALSE(window.takeReady(Clock::time_point{} + 10ms).has_value());
  EXPECT_FALSE(window.nextDeadline().has_value());
}

}  // namespace
}  // namespace aim_predictor
