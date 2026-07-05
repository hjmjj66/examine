#include "aim_predictor/aim_predictor_utils.hpp"

#include <algorithm>
#include <cmath>

namespace aim_predictor
{

Eigen::Vector3d poseToYpr(const geometry_msgs::msg::Pose & pose)
{
  const double yaw = quaternionToYaw(pose.orientation);
  const double qw = pose.orientation.w;
  const double qx = pose.orientation.x;
  const double qy = pose.orientation.y;
  const double qz = pose.orientation.z;

  const double sinp = 2.0 * (qw * qy - qz * qx);
  const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(kPi / 2.0, sinp) : std::asin(sinp);

  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  return {yaw, pitch, roll};
}

double quaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double qw = quaternion.w;
  const double qx = quaternion.x;
  const double qy = quaternion.y;
  const double qz = quaternion.z;

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw * 0.5);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  return q;
}

Eigen::Vector3d xyzToYpd(const Eigen::Vector3d & xyz)
{
  const double distance = xyz.norm();
  const double yaw = std::atan2(xyz.y(), xyz.x());
  const double pitch = std::atan2(-xyz.z(), std::hypot(xyz.x(), xyz.y()));
  return {yaw, pitch, distance};
}

Eigen::MatrixXd xyzToYpdJacobian(const Eigen::Vector3d & xyz)
{
  const double x = xyz.x();
  const double y = xyz.y();
  const double z = xyz.z();
  const double xy_sq = x * x + y * y;
  const double xy_norm = std::sqrt(std::max(xy_sq, 1e-12));
  const double distance_sq = xy_sq + z * z;
  const double distance = std::sqrt(std::max(distance_sq, 1e-12));

  Eigen::MatrixXd jacobian(3, 3);
  jacobian <<
    -y / std::max(xy_sq, 1e-12), x / std::max(xy_sq, 1e-12), 0.0,
    x * z / (std::max(distance_sq, 1e-12) * xy_norm),
    y * z / (std::max(distance_sq, 1e-12) * xy_norm),
    -xy_norm / std::max(distance_sq, 1e-12),
    x / distance,
    y / distance,
    z / distance;
  return jacobian;
}

double limitRad(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

geometry_msgs::msg::Point pointAlongPoseXAxis(
  const geometry_msgs::msg::Pose & pose,
  double distance)
{
  geometry_msgs::msg::Point point;
  const auto & q = pose.orientation;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;

  const double axis_x = 1.0 - 2.0 * (yy + zz);
  const double axis_y = 2.0 * (xy + wz);
  const double axis_z = 2.0 * (xz - wy);

  point.x = pose.position.x + axis_x * distance;
  point.y = pose.position.y + axis_y * distance;
  point.z = pose.position.z + axis_z * distance;
  return point;
}

}  // namespace aim_predictor
