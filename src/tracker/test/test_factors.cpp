#include "tracker/factors.hpp"
#include "tracker/measurement.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/noiseModel/Isotropic.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kJacobianStep = 1e-6;

gtsam::SharedNoiseModel unitNoise(std::size_t dimension)
{
  return gtsam::noiseModel::Isotropic::Sigma(static_cast<int>(dimension), 1.0);
}

void alignWrappedComponent(gtsam::Vector & value, const gtsam::Vector & reference, int index)
{
  while (value[index] - reference[index] > kPi) {
    value[index] -= 2.0 * kPi;
  }
  while (value[index] - reference[index] <= -kPi) {
    value[index] += 2.0 * kPi;
  }
}

template<typename Function>
gtsam::Matrix numericalPose3Jacobian(
  Function function, const gtsam::Pose3 & value, int wrapped_component = -1)
{
  const gtsam::Vector reference = function(value);
  gtsam::Matrix jacobian(reference.size(), 6);
  for (int column = 0; column < 6; ++column) {
    gtsam::Vector6 delta = gtsam::Vector6::Zero();
    delta[column] = kJacobianStep;
    gtsam::Vector plus = function(value.retract(delta));
    delta[column] = -kJacobianStep;
    gtsam::Vector minus = function(value.retract(delta));
    if (wrapped_component >= 0) {
      alignWrappedComponent(plus, reference, wrapped_component);
      alignWrappedComponent(minus, reference, wrapped_component);
    }
    jacobian.col(column) = (plus - minus) / (2.0 * kJacobianStep);
  }
  return jacobian;
}

template<typename Function>
gtsam::Matrix numericalPoint3Jacobian(Function function, const gtsam::Point3 & value)
{
  const gtsam::Vector base = function(value);
  gtsam::Matrix jacobian(base.size(), 3);
  for (int column = 0; column < 3; ++column) {
    gtsam::Point3 plus = value;
    gtsam::Point3 minus = value;
    plus[column] += kJacobianStep;
    minus[column] -= kJacobianStep;
    jacobian.col(column) = (function(plus) - function(minus)) / (2.0 * kJacobianStep);
  }
  return jacobian;
}

template<typename Function>
gtsam::Matrix numericalScalarJacobian(Function function, double value)
{
  const gtsam::Vector plus = function(value + kJacobianStep);
  const gtsam::Vector minus = function(value - kJacobianStep);
  gtsam::Matrix jacobian(plus.size(), 1);
  jacobian.col(0) = (plus - minus) / (2.0 * kJacobianStep);
  return jacobian;
}

template<typename Function>
gtsam::Matrix numericalRot2Jacobian(
  Function function, const gtsam::Rot2 & value, int wrapped_component = -1)
{
  const gtsam::Vector reference = function(value);
  gtsam::Vector plus = function(gtsam::Rot2::fromAngle(value.theta() + kJacobianStep));
  gtsam::Vector minus = function(gtsam::Rot2::fromAngle(value.theta() - kJacobianStep));
  if (wrapped_component >= 0) {
    alignWrappedComponent(plus, reference, wrapped_component);
    alignWrappedComponent(minus, reference, wrapped_component);
  }
  gtsam::Matrix jacobian(plus.size(), 1);
  jacobian.col(0) = (plus - minus) / (2.0 * kJacobianStep);
  return jacobian;
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
  const cv::Mat distortion_coefficients =
    (cv::Mat_<double>(5, 1) << 0.01, -0.001, 0.0005, -0.0003, 0.0001);
  const cv::Mat row_distortion_coefficients =
    (cv::Mat_<double>(1, 5) << 0.01, -0.001, 0.0005, -0.0003, 0.0001);
  const std::array<gtsam::Point3, 4> armor_points{
    gtsam::Point3(-0.1, -0.05, 0.0),
    gtsam::Point3(-0.1, 0.05, 0.0),
    gtsam::Point3(0.1, 0.05, 0.0),
    gtsam::Point3(0.1, -0.05, 0.0)};
  const gtsam::Pose3 pose(gtsam::Rot3::Identity(), gtsam::Point3(0.0, 0.0, 2.0));
  const cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  const cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 2.0);
  std::vector<cv::Point3d> object_points;
  object_points.reserve(armor_points.size());
  for (const auto & point : armor_points) {
    object_points.emplace_back(point.x(), point.y(), point.z());
  }
  std::vector<cv::Point2d> expected_corners;
  cv::projectPoints(
    object_points, rvec, tvec, camera_matrix, distortion_coefficients, expected_corners);
  std::array<gtsam::Point2, 4> corners{};
  for (std::size_t i = 0; i < armor_points.size(); ++i) {
    corners[i] = gtsam::Point2(expected_corners[i].x, expected_corners[i].y);
  }

  const auto factor = tracker::ArmorReprojFactor(
    gtsam::Symbol('p', 0), camera_matrix, distortion_coefficients,
    armor_points, corners, unitNoise(8));
  const gtsam::Vector residual = factor.evaluateError(pose);

  EXPECT_TRUE(residual.isApprox(gtsam::Vector::Zero(8)));

  const auto row_factor = tracker::ArmorReprojFactor(
    gtsam::Symbol('p', 1), camera_matrix, row_distortion_coefficients,
    armor_points, corners, unitNoise(8));
  const gtsam::Vector row_residual = row_factor.evaluateError(pose);

  EXPECT_TRUE(row_residual.isApprox(gtsam::Vector::Zero(8)));
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

TEST(TrackerFactors, TranslationFactorProvidesExpectedJacobians)
{
  const auto factor = tracker::TranslationFactor(
    gtsam::Symbol('x', 0), gtsam::Symbol('v', 0), gtsam::Symbol('x', 1),
    0.5, unitNoise(3));
  gtsam::Matrix H1;
  gtsam::Matrix H2;
  gtsam::Matrix H3;
  factor.evaluateError(
    gtsam::Point3(1.0, 2.0, 3.0), gtsam::Vector3(2.0, -1.0, 0.5),
    gtsam::Point3(2.0, 1.5, 3.25), H1, H2, H3);

  EXPECT_TRUE(H1.isApprox(-gtsam::Matrix3::Identity()));
  EXPECT_TRUE(H2.isApprox(-0.5 * gtsam::Matrix3::Identity()));
  EXPECT_TRUE(H3.isApprox(gtsam::Matrix3::Identity()));
}

TEST(TrackerFactors, VelocityFactorProvidesExpectedJacobians)
{
  const auto factor = tracker::VelocityFactor(
    gtsam::Symbol('v', 0), gtsam::Symbol('v', 1), unitNoise(3));
  gtsam::Matrix H1;
  gtsam::Matrix H2;
  factor.evaluateError(gtsam::Vector3(2.0, -1.0, 0.5), gtsam::Vector3(1.0, 0.0, 0.25), H1, H2);

  EXPECT_TRUE(H1.isApprox(-gtsam::Matrix3::Identity()));
  EXPECT_TRUE(H2.isApprox(gtsam::Matrix3::Identity()));
}

TEST(TrackerFactors, YawFactorProvidesExpectedJacobians)
{
  const auto factor = tracker::YawFactor(
    gtsam::Symbol('r', 0), gtsam::Symbol('w', 0), gtsam::Symbol('r', 1),
    0.5, unitNoise(1));
  gtsam::Matrix H1;
  gtsam::Matrix H2;
  gtsam::Matrix H3;
  factor.evaluateError(
    gtsam::Rot2::fromAngle(0.2), 0.3, gtsam::Rot2::fromAngle(0.35), H1, H2, H3);

  EXPECT_TRUE(H1.isApprox(gtsam::Matrix::Constant(1, 1, -1.0)));
  EXPECT_TRUE(H2.isApprox(gtsam::Matrix::Constant(1, 1, -0.5)));
  EXPECT_TRUE(H3.isApprox(gtsam::Matrix::Constant(1, 1, 1.0)));
}

TEST(TrackerFactors, VyawFactorProvidesExpectedJacobians)
{
  const auto factor = tracker::VyawFactor(
    gtsam::Symbol('w', 0), gtsam::Symbol('w', 1), unitNoise(1));
  gtsam::Matrix H1;
  gtsam::Matrix H2;
  factor.evaluateError(0.3, -0.2, H1, H2);

  EXPECT_TRUE(H1.isApprox(gtsam::Matrix::Constant(1, 1, -1.0)));
  EXPECT_TRUE(H2.isApprox(gtsam::Matrix::Constant(1, 1, 1.0)));
}

TEST(TrackerFactors, GeometryFactorJacobiansMatchIndependentManifoldDifference)
{
  const auto factor = tracker::ArmorGeometryFactor(
    gtsam::Symbol('p', 0), gtsam::Symbol('q', 0), gtsam::Symbol('d', 0),
    gtsam::Symbol('h', 0), gtsam::Symbol('r', 0), gtsam::Symbol('x', 0),
    gtsam::Pose3(), 1, unitNoise(4));
  const gtsam::Pose3 p_camera(
    gtsam::Rot3::Ypr(0.4, 0.0, 0.0), gtsam::Point3(2.0, 2.79, 1.02));
  const double radius = 0.22;
  const double radius_offset = -0.01;
  const double height_offset = 0.02;
  const gtsam::Rot2 center_yaw = gtsam::Rot2::fromAngle(0.15);
  const gtsam::Point3 center_position(2.0, 3.0, 1.0);
  gtsam::Matrix H1;
  gtsam::Matrix H2;
  gtsam::Matrix H3;
  gtsam::Matrix H4;
  gtsam::Matrix H5;
  gtsam::Matrix H6;
  factor.evaluateError(
    p_camera, radius, radius_offset, height_offset, center_yaw, center_position,
    H1, H2, H3, H4, H5, H6);

  const auto residual = [&](const gtsam::Pose3 & pose) {
      return factor.evaluateError(
        pose, radius, radius_offset, height_offset, center_yaw, center_position);
    };
  const auto center_yaw_residual = [&](const gtsam::Rot2 & yaw) {
      return factor.evaluateError(
        p_camera, radius, radius_offset, height_offset, yaw, center_position);
    };
  EXPECT_TRUE(H1.isApprox(numericalPose3Jacobian(residual, p_camera), 1e-5));
  EXPECT_TRUE(H2.isApprox(numericalScalarJacobian(
      [&](double value) {
        return factor.evaluateError(
          p_camera, value, radius_offset, height_offset, center_yaw, center_position);
      }, radius), 1e-5));
  EXPECT_TRUE(H3.isApprox(numericalScalarJacobian(
      [&](double value) {
        return factor.evaluateError(
          p_camera, radius, value, height_offset, center_yaw, center_position);
      }, radius_offset), 1e-5));
  EXPECT_TRUE(H4.isApprox(numericalScalarJacobian(
      [&](double value) {
        return factor.evaluateError(
          p_camera, radius, radius_offset, value, center_yaw, center_position);
      }, height_offset), 1e-5));
  EXPECT_TRUE(H5.isApprox(numericalRot2Jacobian(center_yaw_residual, center_yaw), 1e-5));
  EXPECT_TRUE(H6.isApprox(numericalPoint3Jacobian(
      [&](const gtsam::Point3 & value) {
        return factor.evaluateError(
          p_camera, radius, radius_offset, height_offset, center_yaw, value);
      }, center_position), 1e-5));
}

TEST(TrackerFactors, GeometryYawJacobianRemainsFiniteAcrossWrapBranch)
{
  const auto factor = tracker::ArmorGeometryFactor(
    gtsam::Symbol('p', 0), gtsam::Symbol('q', 0), gtsam::Symbol('d', 0),
    gtsam::Symbol('h', 0), gtsam::Symbol('r', 0), gtsam::Symbol('x', 0),
    gtsam::Pose3(), 0, unitNoise(4));
  const gtsam::Pose3 p_camera(
    gtsam::Rot3::Ypr(kPi - 1e-7, 0.0, 0.0), gtsam::Point3(0.78, 0.0, 0.0));
  const gtsam::Rot2 center_yaw = gtsam::Rot2::fromAngle(0.0);
  const gtsam::Point3 center_position(1.0, 0.0, 0.0);
  gtsam::Matrix H1;
  gtsam::Matrix H2;
  gtsam::Matrix H3;
  gtsam::Matrix H4;
  gtsam::Matrix H5;
  gtsam::Matrix H6;
  factor.evaluateError(
    p_camera, 0.22, 0.0, 0.0, center_yaw, center_position,
    H1, H2, H3, H4, H5, H6);

  const auto residual = [&](const gtsam::Pose3 & pose) {
      return factor.evaluateError(pose, 0.22, 0.0, 0.0, center_yaw, center_position);
    };
  const auto center_yaw_residual = [&](const gtsam::Rot2 & yaw) {
      return factor.evaluateError(p_camera, 0.22, 0.0, 0.0, yaw, center_position);
    };
  const gtsam::Matrix expected_h1 = numericalPose3Jacobian(residual, p_camera, 3);
  const gtsam::Matrix expected_h5 = numericalRot2Jacobian(center_yaw_residual, center_yaw, 3);
  EXPECT_TRUE(H1.isApprox(expected_h1, 1e-5));
  EXPECT_TRUE(H5.isApprox(expected_h5, 1e-5));
  EXPECT_LT(std::abs(H1(3, 2)), 2.0);
  EXPECT_LT(std::abs(H5(3, 0)), 2.0);
  EXPECT_NEAR(H1(3, 2), -1.0, 1e-5);
  EXPECT_NEAR(H5(3, 0), 1.0, 1e-5);
}

TEST(TrackerFactors, ReprojectionFactorJacobianMatchesIndependentManifoldDifference)
{
  const cv::Mat camera_matrix =
    (cv::Mat_<double>(3, 3) << 400.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0);
  const cv::Mat distortion_coefficients =
    (cv::Mat_<double>(5, 1) << 0.01, -0.001, 0.0005, -0.0003, 0.0001);
  const std::array<gtsam::Point3, 4> armor_points{
    gtsam::Point3(-0.1, -0.05, 0.0),
    gtsam::Point3(-0.1, 0.05, 0.0),
    gtsam::Point3(0.1, 0.05, 0.0),
    gtsam::Point3(0.1, -0.05, 0.0)};
  const gtsam::Pose3 pose(gtsam::Rot3::Ypr(0.1, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 2.0));
  const cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.1);
  const cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 2.0);
  std::vector<cv::Point3d> object_points;
  object_points.reserve(armor_points.size());
  for (const auto & point : armor_points) {
    object_points.emplace_back(point.x(), point.y(), point.z());
  }
  std::vector<cv::Point2d> projected_points;
  cv::projectPoints(
    object_points, rvec, tvec, camera_matrix, distortion_coefficients, projected_points);
  std::array<gtsam::Point2, 4> corners{};
  for (std::size_t i = 0; i < corners.size(); ++i) {
    corners[i] = gtsam::Point2(projected_points[i].x, projected_points[i].y);
  }
  const auto factor = tracker::ArmorReprojFactor(
    gtsam::Symbol('p', 0), camera_matrix, distortion_coefficients,
    armor_points, corners, unitNoise(8));
  gtsam::Matrix H;
  factor.evaluateError(pose, H);
  const auto residual = [&](const gtsam::Pose3 & value) {
      return factor.evaluateError(value);
    };

  EXPECT_TRUE(H.isApprox(numericalPose3Jacobian(residual, pose), 1e-5));
}
