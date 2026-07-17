#include <cmath>
#include <optional>

#include <gtest/gtest.h>

#include "aim_solver/time_alignment_estimator.hpp"

namespace
{

double gimbalYaw(double stamp_sec)
{
  return 0.35 * std::sin(3.0 * stamp_sec) + 0.08 * stamp_sec;
}

TEST(TimeAlignmentEstimator, FindsOffsetFromChangingGimbalMotion)
{
  aim_solver::TimeAlignmentEstimator::Config config;
  config.initial_offset_sec = 0.0;
  config.min_offset_sec = -0.30;
  config.max_offset_sec = 0.10;
  config.search_step_sec = 0.002;
  config.window_size = 32U;
  config.min_samples = 10U;
  config.min_motion_rad = 0.02;
  config.update_period_sec = 0.0;
  config.max_offset_step_sec = 0.30;

  aim_solver::TimeAlignmentEstimator estimator(config);
  constexpr double true_offset = -0.12;
  for (int i = 0; i < 32; ++i) {
    const double stamp = 2.0 + static_cast<double>(i) * 0.03;
    const double camera_bearing = -gimbalYaw(stamp + true_offset);
    estimator.addSample(
      {
        stamp,
        {std::cos(camera_bearing), std::sin(camera_bearing), 0.0}});
  }

  const auto status = estimator.update(
    3.0,
    0.4,
    [](double offset, const aim_solver::TimeAlignmentEstimator::Sample & sample)
    -> std::optional<double> {
      return std::atan2(sample.point[1], sample.point[0]) +
      gimbalYaw(sample.stamp_sec + offset);
    });

  EXPECT_TRUE(status.updated);
  EXPECT_NEAR(status.offset_sec, true_offset, 0.015);
  EXPECT_LT(status.score_rad2, status.score_at_zero_rad2);
}

TEST(TimeAlignmentEstimator, FreezesWhenGimbalDoesNotMove)
{
  aim_solver::TimeAlignmentEstimator estimator;
  for (int i = 0; i < 12; ++i) {
    estimator.addSample({1.0 + i * 0.02, {1.0, 0.0, 0.0}});
  }

  const auto status = estimator.update(
    2.0,
    0.0,
    [](double, const aim_solver::TimeAlignmentEstimator::Sample &)
    -> std::optional<double> {return 0.2;});

  EXPECT_FALSE(status.updated);
  EXPECT_TRUE(status.frozen);
  EXPECT_EQ(status.reason, "insufficient_gimbal_motion");
}

TEST(TimeAlignmentEstimator, ClearsSamplesWhenTargetChanges)
{
  aim_solver::TimeAlignmentEstimator estimator;
  estimator.addSample({1.0, {1.0, 0.0, 0.0}});
  const double initial_offset = estimator.offsetSec();

  estimator.resetSamples();

  EXPECT_TRUE(estimator.samples().empty());
  EXPECT_DOUBLE_EQ(estimator.offsetSec(), initial_offset);
  EXPECT_EQ(estimator.status().reason, "target_changed_waiting_for_samples");
}

TEST(TimeAlignmentEstimator, DoesNotDriftWhenFrozenWithoutSamples)
{
  aim_solver::TimeAlignmentEstimator::Config config;
  config.update_period_sec = 0.0;
  config.min_samples = 4U;
  config.window_size = 8U;
  config.max_offset_step_sec = 0.3;
  config.min_offset_sec = -0.3;
  config.max_offset_sec = 0.1;
  config.search_step_sec = 0.002;
  aim_solver::TimeAlignmentEstimator estimator(config);

  for (int i = 0; i < 8; ++i) {
    const double stamp = 2.0 + i * 0.03;
    const double bearing = -gimbalYaw(stamp - 0.12);
    estimator.addSample({stamp, {std::cos(bearing), std::sin(bearing), 0.0}});
  }
  const auto updated = estimator.update(
    3.0,
    0.4,
    [](double offset, const aim_solver::TimeAlignmentEstimator::Sample & sample)
    -> std::optional<double> {
      return std::atan2(sample.point[1], sample.point[0]) +
        gimbalYaw(sample.stamp_sec + offset);
    });
  ASSERT_TRUE(updated.updated);
  const double offset_before = updated.offset_sec;
  const double drift_before = updated.offset_drift_sec_per_sec;

  estimator.resetSamples();
  const auto frozen = estimator.update(
    5.0,
    0.0,
    [](double, const aim_solver::TimeAlignmentEstimator::Sample &)
    -> std::optional<double> {return 0.0;});

  EXPECT_TRUE(frozen.frozen);
  EXPECT_DOUBLE_EQ(frozen.offset_sec, offset_before);
  EXPECT_DOUBLE_EQ(frozen.offset_drift_sec_per_sec, drift_before);
  EXPECT_EQ(frozen.reason, "waiting_for_samples");
}

}  // namespace
