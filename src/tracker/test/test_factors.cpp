#include "tracker/factors.hpp"
#include "tracker/measurement.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/noiseModel/Isotropic.h>

#include <opencv2/core.hpp>

namespace
{

constexpr double kPi = 3.14159265358979323846;

gtsam::SharedNoiseModel unitNoise(std::size_t dimension)
{
  return gtsam::noiseModel::Isotropic::Sigma(static_cast<int>(dimension), 1.0);
}

}  // namespace

TEST(TrackerFactors, TranslationFactorHasZeroConstantVelocityResidual)
{
  const auto factor = tracker::TranslationFactor(
    gtsam::Symbol('x', 0), gtsam::Symbol('v', 0), gtsam::Symbol('x', 1),
    0.5, unitNoise(3));

  const gtsam::Vector3 residual = factor.evaluateError(
    gtsam::Point3(1.0, 2.0, 3.0),
    gtsam::Vector3(2.0, -1.0, 0.5),
    gtsam::Point3(2.0, 1.5, 3.25));

  EXPECT_TRUE(residual.isApprox(gtsam::Vector3::Zero()));
}

TEST(TrackerFactors, VelocityFactorHasZeroResidualForUnchangedVelocity)
{
  const auto factor = tracker::VelocityFactor(
    gtsam::Symbol('v', 0), gtsam::Symbol('v', 1), unitNoise(3));

  const gtsam::Vector3 residual = factor.evaluateError(
    gtsam::Vector3(2.0, -1.0, 0.5),
    gtsam::Vector3(2.0, -1.0, 0.5));

  EXPECT_TRUE(residual.isApprox(gtsam::Vector3::Zero()));
}

TEST(TrackerFactors, YawFactorWrapsAcrossNegativePi)
{
  const auto factor = tracker::YawFactor(
    gtsam::Symbol('r', 0), gtsam::Symbol('w', 0), gtsam::Symbol('r', 1),
    0.0, unitNoise(1));

  const gtsam::Vector residual = factor.evaluateError(
    gtsam::Rot2::fromAngle(kPi - 0.01), 0.0, gtsam::Rot2::fromAngle(-kPi + 0.01));

  ASSERT_EQ(residual.size(), 1);
  EXPECT_NEAR(residual[0], 0.02, 1e-12);
}

TEST(TrackerFactors, GeometryFactorUsesBaseRadiusForIndexZero)
{
  const auto factor = tracker::ArmorGeometryFactor(
    gtsam::Symbol('p', 0), gtsam::Symbol('q', 0), gtsam::Symbol('d', 0),
    gtsam::Symbol('h', 0), gtsam::Symbol('r', 0), gtsam::Symbol('x', 0),
    gtsam::Pose3(), 0, unitNoise(4));

  const gtsam::Vector residual = factor.evaluateError(
    gtsam::Pose3(gtsam::Rot3::Ypr(0.0, 0.0, 0.0), gtsam::Point3(1.78, 3.0, 1.0)),
    0.22, -0.01, 0.02, gtsam::Rot2::fromAngle(0.0), gtsam::Point3(2.0, 3.0, 1.0));

  EXPECT_TRUE(residual.isApprox(gtsam::Vector::Zero(4)));
}

TEST(TrackerFactors, GeometryFactorUsesRadiusAndHeightOffsetsForIndexOne)
{
  const auto factor = tracker::ArmorGeometryFactor(
    gtsam::Symbol('p', 0), gtsam::Symbol('q', 0), gtsam::Symbol('d', 0),
    gtsam::Symbol('h', 0), gtsam::Symbol('r', 0), gtsam::Symbol('x', 0),
    gtsam::Pose3(), 1, unitNoise(4));

  const gtsam::Vector residual = factor.evaluateError(
    gtsam::Pose3(
      gtsam::Rot3::Ypr(kPi / 2.0, 0.0, 0.0),
      gtsam::Point3(2.0, 2.79, 1.02)),
    0.22, -0.01, 0.02, gtsam::Rot2::fromAngle(0.0), gtsam::Point3(2.0, 3.0, 1.0));

  EXPECT_TRUE(residual.isApprox(gtsam::Vector::Zero(4)));
}

TEST(TrackerFactors, ReprojectionFactorHasZeroResidualForKnownPinholeProjection)
{
  const cv::Mat camera_matrix =
    (cv::Mat_<double>(3, 3) << 400.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0);
  const cv::Mat distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);
  const gtsam::Cal3DS2 calibration(400.0, 500.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0);
  const std::array<gtsam::Point3, 4> armor_points{
    gtsam::Point3(-0.1, -0.05, 0.0),
    gtsam::Point3(-0.1, 0.05, 0.0),
    gtsam::Point3(0.1, 0.05, 0.0),
    gtsam::Point3(0.1, -0.05, 0.0)};
  const gtsam::Pose3 pose(gtsam::Rot3::Identity(), gtsam::Point3(0.0, 0.0, 2.0));
  std::array<gtsam::Point2, 4> corners{};
  for (std::size_t i = 0; i < armor_points.size(); ++i) {
    corners[i] = calibration.uncalibrate(
      gtsam::PinholeCamera<gtsam::Cal3DS2>::Project(pose.transformFrom(armor_points[i])));
  }

  const auto factor = tracker::ArmorReprojFactor(
    gtsam::Symbol('p', 0), camera_matrix, distortion_coefficients,
    armor_points, corners, unitNoise(8));
  const gtsam::Vector residual = factor.evaluateError(pose);

  EXPECT_TRUE(residual.isApprox(gtsam::Vector::Zero(8)));
}

TEST(TrackerMeasurement, ConvertsObservationFieldsWithoutLoss)
{
  aim_msgs::msg::ArmorPoseObservation message;
  message.pose.position.x = 1.0;
  message.camera_pose.position.z = 2.0;
  message.corners[2].x = 123.0;
  message.armor_class.class_id = 3;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 7;
  stamp.nanosec = 11;

  const tracker::ArmorObservation observation = tracker::fromRosObservation(
    4, tracker::CameraSource::Back, stamp, message);

  EXPECT_EQ(observation.target_id, 4);
  EXPECT_EQ(observation.source, tracker::CameraSource::Back);
  EXPECT_EQ(observation.stamp.sec, 7);
  EXPECT_EQ(observation.stamp.nanosec, 11u);
  EXPECT_DOUBLE_EQ(observation.world_pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(observation.camera_pose.position.z, 2.0);
  EXPECT_DOUBLE_EQ(observation.corners[2].x, 123.0);
  EXPECT_EQ(observation.armor_class.class_id, 3);
}
