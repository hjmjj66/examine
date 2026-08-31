#include "tracker/factors.hpp"

#include <cmath>

#include <gtsam/geometry/PinholeCamera.h>

namespace tracker
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kFiniteDifferenceStep = 1e-6;

double wrapAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

template<typename Function>
gtsam::Matrix finiteDifferencePose3(Function function, const gtsam::Pose3 & value)
{
  gtsam::Matrix jacobian;
  const gtsam::Vector base = function(value);
  jacobian.resize(base.size(), 6);
  for (int column = 0; column < 6; ++column) {
    gtsam::Vector6 delta = gtsam::Vector6::Zero();
    delta[column] = kFiniteDifferenceStep;
    const gtsam::Vector plus = function(value.retract(delta));
    delta[column] = -kFiniteDifferenceStep;
    const gtsam::Vector minus = function(value.retract(delta));
    jacobian.col(column) = (plus - minus) / (2.0 * kFiniteDifferenceStep);
  }
  return jacobian;
}

template<typename Function>
gtsam::Matrix finiteDifferencePoint3(Function function, const gtsam::Point3 & value)
{
  gtsam::Matrix jacobian;
  const gtsam::Vector base = function(value);
  jacobian.resize(base.size(), 3);
  for (int column = 0; column < 3; ++column) {
    gtsam::Point3 plus = value;
    gtsam::Point3 minus = value;
    plus[column] += kFiniteDifferenceStep;
    minus[column] -= kFiniteDifferenceStep;
    jacobian.col(column) =
      (function(plus) - function(minus)) / (2.0 * kFiniteDifferenceStep);
  }
  return jacobian;
}

template<typename Function>
gtsam::Matrix finiteDifferenceScalar(Function function, double value)
{
  const gtsam::Vector plus = function(value + kFiniteDifferenceStep);
  const gtsam::Vector minus = function(value - kFiniteDifferenceStep);
  gtsam::Matrix jacobian(plus.size(), 1);
  jacobian.col(0) = (plus - minus) / (2.0 * kFiniteDifferenceStep);
  return jacobian;
}

template<typename Function>
gtsam::Matrix finiteDifferenceRot2(Function function, const gtsam::Rot2 & value)
{
  const gtsam::Vector plus = function(gtsam::Rot2::fromAngle(value.theta() + kFiniteDifferenceStep));
  const gtsam::Vector minus = function(gtsam::Rot2::fromAngle(value.theta() - kFiniteDifferenceStep));
  gtsam::Matrix jacobian(plus.size(), 1);
  jacobian.col(0) = (plus - minus) / (2.0 * kFiniteDifferenceStep);
  return jacobian;
}

gtsam::Vector scalarResidual(double value)
{
  gtsam::Vector residual(1);
  residual[0] = value;
  return residual;
}

}  // namespace

TranslationFactor::TranslationFactor(
  gtsam::Key x_pre,
  gtsam::Key v_pre,
  gtsam::Key x_cur,
  double dt,
  const gtsam::SharedNoiseModel & noise_model)
: gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Vector3, gtsam::Point3>(
    noise_model, x_pre, v_pre, x_cur), dt_(dt)
{
}

gtsam::Vector TranslationFactor::evaluateError(
  const gtsam::Point3 & x_pre,
  const gtsam::Vector3 & v_pre,
  const gtsam::Point3 & x_cur,
  gtsam::OptionalMatrixType H1,
  gtsam::OptionalMatrixType H2,
  gtsam::OptionalMatrixType H3) const
{
  if (H1) {
    *H1 = -gtsam::Matrix3::Identity();
  }
  if (H2) {
    *H2 = -dt_ * gtsam::Matrix3::Identity();
  }
  if (H3) {
    *H3 = gtsam::Matrix3::Identity();
  }
  return x_cur - x_pre - dt_ * v_pre;
}

VelocityFactor::VelocityFactor(
  gtsam::Key v_pre,
  gtsam::Key v_cur,
  const gtsam::SharedNoiseModel & noise_model)
: gtsam::NoiseModelFactorN<gtsam::Vector3, gtsam::Vector3>(noise_model, v_pre, v_cur)
{
}

gtsam::Vector VelocityFactor::evaluateError(
  const gtsam::Vector3 & v_pre,
  const gtsam::Vector3 & v_cur,
  gtsam::OptionalMatrixType H1,
  gtsam::OptionalMatrixType H2) const
{
  if (H1) {
    *H1 = -gtsam::Matrix3::Identity();
  }
  if (H2) {
    *H2 = gtsam::Matrix3::Identity();
  }
  return v_cur - v_pre;
}

YawFactor::YawFactor(
  gtsam::Key r_pre,
  gtsam::Key w_pre,
  gtsam::Key r_cur,
  double dt,
  const gtsam::SharedNoiseModel & noise_model)
: gtsam::NoiseModelFactorN<gtsam::Rot2, double, gtsam::Rot2>(
    noise_model, r_pre, w_pre, r_cur), dt_(dt)
{
}

gtsam::Vector YawFactor::evaluateError(
  const gtsam::Rot2 & r_pre,
  const double & w_pre,
  const gtsam::Rot2 & r_cur,
  gtsam::OptionalMatrixType H1,
  gtsam::OptionalMatrixType H2,
  gtsam::OptionalMatrixType H3) const
{
  if (H1) {
    *H1 = gtsam::Matrix::Constant(1, 1, -1.0);
  }
  if (H2) {
    *H2 = gtsam::Matrix::Constant(1, 1, -dt_);
  }
  if (H3) {
    *H3 = gtsam::Matrix::Constant(1, 1, 1.0);
  }
  return r_pre.compose(gtsam::Rot2::fromAngle(dt_ * w_pre)).localCoordinates(r_cur);
}

VyawFactor::VyawFactor(
  gtsam::Key w_pre,
  gtsam::Key w_cur,
  const gtsam::SharedNoiseModel & noise_model)
: gtsam::NoiseModelFactorN<double, double>(noise_model, w_pre, w_cur)
{
}

gtsam::Vector VyawFactor::evaluateError(
  const double & w_pre,
  const double & w_cur,
  gtsam::OptionalMatrixType H1,
  gtsam::OptionalMatrixType H2) const
{
  if (H1) {
    *H1 = gtsam::Matrix::Constant(1, 1, -1.0);
  }
  if (H2) {
    *H2 = gtsam::Matrix::Constant(1, 1, 1.0);
  }
  return scalarResidual(w_cur - w_pre);
}

ArmorGeometryFactor::ArmorGeometryFactor(
  gtsam::Key p_camera,
  gtsam::Key radius,
  gtsam::Key radius_offset,
  gtsam::Key height_offset,
  gtsam::Key center_yaw,
  gtsam::Key center_position,
  const gtsam::Pose3 & camera_to_world,
  std::size_t armor_index,
  const gtsam::SharedNoiseModel & noise_model)
: gtsam::NoiseModelFactorN<gtsam::Pose3, double, double, double, double, gtsam::Point3>(
    noise_model, p_camera, radius, radius_offset, height_offset, center_yaw, center_position),
  camera_to_world_(camera_to_world), armor_index_(armor_index)
{
}

gtsam::Vector ArmorGeometryFactor::geometryResidual(
  const gtsam::Pose3 & p_camera,
  double radius,
  double radius_offset,
  double height_offset,
  const gtsam::Rot2 & center_yaw,
  const gtsam::Point3 & center_position) const
{
  const double armor_yaw = center_yaw.theta() + static_cast<double>(armor_index_) * kPi / 2.0;
  const bool has_offset = armor_index_ == 1 || armor_index_ == 3;
  const double armor_radius = radius + (has_offset ? radius_offset : 0.0);
  const double armor_height = has_offset ? height_offset : 0.0;
  const gtsam::Point3 observed_position = camera_to_world_.transformFrom(p_camera.translation());
  const double cosine = std::cos(armor_yaw);
  const double sine = std::sin(armor_yaw);
  const gtsam::Vector3 tangential(-sine, cosine, 0.0);
  const gtsam::Vector3 radial(cosine, sine, 0.0);
  const double dx = center_position.x() - observed_position.x();
  const double dy = center_position.y() - observed_position.y();
  gtsam::Vector residual(4);
  residual <<
    tangential.x() * dx + tangential.y() * dy,
    radial.x() * dx + radial.y() * dy - armor_radius,
    center_position.z() + armor_height - observed_position.z(),
    wrapAngle(armor_yaw - camera_to_world_.compose(p_camera).rotation().ypr()(0));
  return residual;
}

gtsam::Vector ArmorGeometryFactor::evaluateError(
  const gtsam::Pose3 & p_camera,
  const double & radius,
  const double & radius_offset,
  const double & height_offset,
  const gtsam::Rot2 & center_yaw,
  const gtsam::Point3 & center_position,
  gtsam::OptionalMatrixType H1,
  gtsam::OptionalMatrixType H2,
  gtsam::OptionalMatrixType H3,
  gtsam::OptionalMatrixType H4,
  gtsam::OptionalMatrixType H5,
  gtsam::OptionalMatrixType H6) const
{
  if (H1) {
    *H1 = finiteDifferencePose3(
      [this, radius, radius_offset, height_offset, center_yaw, &center_position](
        const gtsam::Pose3 & value) {
        return geometryResidual(value, radius, radius_offset, height_offset, center_yaw, center_position);
      }, p_camera);
  }
  if (H2) {
    *H2 = finiteDifferenceScalar(
      [this, &p_camera, radius_offset, height_offset, center_yaw, &center_position](double value) {
        return geometryResidual(p_camera, value, radius_offset, height_offset, center_yaw, center_position);
      }, radius);
  }
  if (H3) {
    *H3 = finiteDifferenceScalar(
      [this, &p_camera, radius, height_offset, center_yaw, &center_position](double value) {
        return geometryResidual(p_camera, radius, value, height_offset, center_yaw, center_position);
      }, radius_offset);
  }
  if (H4) {
    *H4 = finiteDifferenceScalar(
      [this, &p_camera, radius, radius_offset, center_yaw, &center_position](double value) {
        return geometryResidual(p_camera, radius, radius_offset, value, center_yaw, center_position);
      }, height_offset);
  }
  if (H5) {
    *H5 = finiteDifferenceRot2(
      [this, &p_camera, radius, radius_offset, height_offset, &center_position](
        const gtsam::Rot2 & value) {
        return geometryResidual(p_camera, radius, radius_offset, height_offset, value, center_position);
      }, center_yaw);
  }
  if (H6) {
    *H6 = finiteDifferencePoint3(
      [this, &p_camera, radius, radius_offset, height_offset, center_yaw](
        const gtsam::Point3 & value) {
        return geometryResidual(p_camera, radius, radius_offset, height_offset, center_yaw, value);
      }, center_position);
  }
  return geometryResidual(p_camera, radius, radius_offset, height_offset, center_yaw, center_position);
}

ArmorReprojFactor::ArmorReprojFactor(
  gtsam::Key p_camera,
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion_coefficients,
  const std::array<gtsam::Point3, 4> & armor_points,
  const std::array<gtsam::Point2, 4> & corners,
  const gtsam::SharedNoiseModel & noise_model)
: gtsam::NoiseModelFactorN<gtsam::Pose3>(noise_model, p_camera),
  calibration_(
    camera_matrix.at<double>(0, 0), camera_matrix.at<double>(1, 1),
    camera_matrix.at<double>(0, 1), camera_matrix.at<double>(0, 2),
    camera_matrix.at<double>(1, 2), distortion_coefficients.at<double>(0, 0),
    distortion_coefficients.at<double>(0, 1), distortion_coefficients.at<double>(0, 2),
    distortion_coefficients.at<double>(0, 3)),
  armor_points_(armor_points), corners_(corners)
{
}

gtsam::Vector ArmorReprojFactor::reprojectionResidual(const gtsam::Pose3 & p_camera) const
{
  gtsam::Vector residual(8);
  for (std::size_t i = 0; i < armor_points_.size(); ++i) {
    const gtsam::Point2 projected = calibration_.uncalibrate(
      gtsam::PinholeCamera<gtsam::Cal3DS2>::Project(
        p_camera.transformFrom(armor_points_[i])));
    residual[2 * i] = projected.x() - corners_[i].x();
    residual[2 * i + 1] = projected.y() - corners_[i].y();
  }
  return residual;
}

gtsam::Vector ArmorReprojFactor::evaluateError(
  const gtsam::Pose3 & p_camera,
  gtsam::OptionalMatrixType H1) const
{
  if (H1) {
    *H1 = finiteDifferencePose3(
      [this](const gtsam::Pose3 & value) { return reprojectionResidual(value); }, p_camera);
  }
  return reprojectionResidual(p_camera);
}

}  // namespace tracker
