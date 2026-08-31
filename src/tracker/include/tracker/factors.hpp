#pragma once

#include <array>
#include <cstddef>

#include <boost/optional/optional.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <opencv2/core.hpp>

namespace tracker
{

class TranslationFactor : public gtsam::NoiseModelFactorN<
    gtsam::Point3, gtsam::Vector3, gtsam::Point3>
{
public:
  TranslationFactor(
    gtsam::Key x_pre,
    gtsam::Key v_pre,
    gtsam::Key x_cur,
    double dt,
    const gtsam::SharedNoiseModel & noise_model);

  gtsam::Vector evaluateError(
    const gtsam::Point3 & x_pre,
    const gtsam::Vector3 & v_pre,
    const gtsam::Point3 & x_cur,
    gtsam::OptionalMatrixType H1 = boost::none,
    gtsam::OptionalMatrixType H2 = boost::none,
    gtsam::OptionalMatrixType H3 = boost::none) const override;

private:
  double dt_;
};

class VelocityFactor : public gtsam::NoiseModelFactorN<gtsam::Vector3, gtsam::Vector3>
{
public:
  VelocityFactor(
    gtsam::Key v_pre,
    gtsam::Key v_cur,
    const gtsam::SharedNoiseModel & noise_model);

  gtsam::Vector evaluateError(
    const gtsam::Vector3 & v_pre,
    const gtsam::Vector3 & v_cur,
    gtsam::OptionalMatrixType H1 = boost::none,
    gtsam::OptionalMatrixType H2 = boost::none) const override;
};

class YawFactor : public gtsam::NoiseModelFactorN<gtsam::Rot2, double, gtsam::Rot2>
{
public:
  YawFactor(
    gtsam::Key r_pre,
    gtsam::Key w_pre,
    gtsam::Key r_cur,
    double dt,
    const gtsam::SharedNoiseModel & noise_model);

  gtsam::Vector evaluateError(
    const gtsam::Rot2 & r_pre,
    const double & w_pre,
    const gtsam::Rot2 & r_cur,
    gtsam::OptionalMatrixType H1 = boost::none,
    gtsam::OptionalMatrixType H2 = boost::none,
    gtsam::OptionalMatrixType H3 = boost::none) const override;

private:
  double dt_;
};

class VyawFactor : public gtsam::NoiseModelFactorN<double, double>
{
public:
  VyawFactor(
    gtsam::Key w_pre,
    gtsam::Key w_cur,
    const gtsam::SharedNoiseModel & noise_model);

  gtsam::Vector evaluateError(
    const double & w_pre,
    const double & w_cur,
    gtsam::OptionalMatrixType H1 = boost::none,
    gtsam::OptionalMatrixType H2 = boost::none) const override;
};

class ArmorGeometryFactor : public gtsam::NoiseModelFactorN<
    gtsam::Pose3, double, double, double, double, gtsam::Point3>
{
public:
  ArmorGeometryFactor(
    gtsam::Key p_camera,
    gtsam::Key radius,
    gtsam::Key radius_offset,
    gtsam::Key height_offset,
    gtsam::Key center_yaw,
    gtsam::Key center_position,
    const gtsam::Pose3 & camera_to_world,
    std::size_t armor_index,
    const gtsam::SharedNoiseModel & noise_model);

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & p_camera,
    const double & radius,
    const double & radius_offset,
    const double & height_offset,
    const gtsam::Rot2 & center_yaw,
    const gtsam::Point3 & center_position,
    gtsam::OptionalMatrixType H1 = boost::none,
    gtsam::OptionalMatrixType H2 = boost::none,
    gtsam::OptionalMatrixType H3 = boost::none,
    gtsam::OptionalMatrixType H4 = boost::none,
    gtsam::OptionalMatrixType H5 = boost::none,
    gtsam::OptionalMatrixType H6 = boost::none) const override;

private:
  gtsam::Vector geometryResidual(
    const gtsam::Pose3 & p_camera,
    double radius,
    double radius_offset,
    double height_offset,
    const gtsam::Rot2 & center_yaw,
    const gtsam::Point3 & center_position) const;

  gtsam::Pose3 camera_to_world_;
  std::size_t armor_index_;
};

class ArmorReprojFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
public:
  ArmorReprojFactor(
    gtsam::Key p_camera,
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    const std::array<gtsam::Point3, 4> & armor_points,
    const std::array<gtsam::Point2, 4> & corners,
    const gtsam::SharedNoiseModel & noise_model);

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & p_camera,
    gtsam::OptionalMatrixType H1 = boost::none) const override;

private:
  gtsam::Vector reprojectionResidual(const gtsam::Pose3 & p_camera) const;

  gtsam::Cal3DS2 calibration_;
  std::array<gtsam::Point3, 4> armor_points_;
  std::array<gtsam::Point2, 4> corners_;
};

}  // namespace tracker
