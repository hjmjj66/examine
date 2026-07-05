#pragma once

#include <Eigen/Dense>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace aim_predictor
{

inline constexpr double kPi = 3.14159265358979323846;

Eigen::Vector3d poseToYpr(const geometry_msgs::msg::Pose & pose);
double quaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion);
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
Eigen::Vector3d xyzToYpd(const Eigen::Vector3d & xyz);
Eigen::MatrixXd xyzToYpdJacobian(const Eigen::Vector3d & xyz);
double limitRad(double angle);
geometry_msgs::msg::Point pointAlongPoseXAxis(
  const geometry_msgs::msg::Pose & pose,
  double distance);

}  // namespace aim_predictor
