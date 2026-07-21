#include <gtest/gtest.h>

#include "aim_predictor/measurement_noise.hpp"

TEST(MeasurementNoise, UnitScalePreservesLegacyCovariance)
{
  const aim_predictor::MeasurementNoiseConfig config;
  const Eigen::Vector4d diagonal =
    aim_predictor::measurementNoiseDiagonal(0.5, 3.0, config);

  EXPECT_DOUBLE_EQ(diagonal[0], 4e-3);
  EXPECT_DOUBLE_EQ(diagonal[1], 4e-3);
  EXPECT_DOUBLE_EQ(diagonal[2], std::log(1.5) + 0.1);
  EXPECT_DOUBLE_EQ(diagonal[3], std::log(4.0) / 200.0 + 9e-2);
}

TEST(MeasurementNoise, PerCameraScaleChangesOnlyItsConfiguredObservation)
{
  aim_predictor::MeasurementNoiseConfig config;
  config.yaw_variance_scale = 2.0;
  config.distance_variance_scale = 0.5;

  const Eigen::Vector4d diagonal =
    aim_predictor::measurementNoiseDiagonal(0.5, 3.0, config);

  EXPECT_DOUBLE_EQ(diagonal[0], 8e-3);
  EXPECT_DOUBLE_EQ(diagonal[1], 4e-3);
  EXPECT_DOUBLE_EQ(diagonal[2], (std::log(1.5) + 0.1) * 0.5);
  EXPECT_DOUBLE_EQ(diagonal[3], std::log(4.0) / 200.0 + 9e-2);
}
