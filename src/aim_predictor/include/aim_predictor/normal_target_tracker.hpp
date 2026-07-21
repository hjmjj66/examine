#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/time.hpp>

#include "aim_predictor/extended_kalman_filter.hpp"
#include "aim_predictor/measurement_noise.hpp"

namespace aim_predictor
{

struct ArmorMeasurement
{
  std::uint8_t id{0};
  geometry_msgs::msg::Pose pose;
  Eigen::Vector3d xyz{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ypr{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ypd{Eigen::Vector3d::Zero()};
  CameraSource source{CameraSource::Front0};
};

struct ProcessNoiseConfig
{
  double low_speed_process_noise_xy{1600.0};
  double low_speed_process_noise_z{1600.0};
  double low_speed_process_noise_yaw{400.0};
  double middle_speed_process_noise_xy{1600.0};
  double middle_speed_process_noise_z{1600.0};
  double middle_speed_process_noise_yaw{400.0};
  double high_speed_process_noise_xy{1600.0};
  double high_speed_process_noise_z{1600.0};
  double high_speed_process_noise_yaw{400.0};
  double middle_speed_angular_velocity_threshold{2.0};
  double high_speed_angular_velocity_threshold{6.0};
};

class NormalTargetTracker
{
public:
  NormalTargetTracker() = default;

  void setProcessNoiseConfig(const ProcessNoiseConfig & process_noise_config);
  void initialize(const ArmorMeasurement & armor, const rclcpp::Time & stamp, double radius);
  void predict(const rclcpp::Time & stamp);
  void update(
    const ArmorMeasurement & armor,
    const MeasurementNoiseConfig & measurement_noise_config);

  [[nodiscard]] bool active() const;
  [[nodiscard]] bool converged() const;
  [[nodiscard]] bool diverged() const;
  [[nodiscard]] bool jumped() const;
  [[nodiscard]] std::uint8_t target_id() const;
  [[nodiscard]] const Eigen::VectorXd & state() const;
  [[nodiscard]] std::vector<geometry_msgs::msg::Pose> predictedArmors() const;

  int lost_count{0};
  int update_count{0};

private:
  Eigen::Vector3d armorPosition(const Eigen::VectorXd & x, int armor_index) const;
  Eigen::MatrixXd observationJacobian(const Eigen::VectorXd & x, int armor_index) const;
  std::vector<Eigen::Vector4d> predictedArmorStates() const;
  int matchArmorIndex(const ArmorMeasurement & armor) const;

  bool initialized_{false};
  std::uint8_t target_id_{0};
  int armor_num_{4};
  int last_armor_index_{0};
  bool jumped_{false};
  ProcessNoiseConfig process_noise_config_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  ExtendedKalmanFilter ekf_;
};

}  // namespace aim_predictor
