#include "aim_predictor/normal_target_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "aim_predictor/aim_predictor_utils.hpp"

namespace aim_predictor
{

void NormalTargetTracker::setProcessNoiseConfig(const ProcessNoiseConfig & process_noise_config)
{
  process_noise_config_ = process_noise_config;
}

void NormalTargetTracker::initialize(
  const ArmorMeasurement & armor,
  const rclcpp::Time & stamp,
  double radius)
{
  const double center_x = armor.xyz.x() + radius * std::cos(armor.ypr.x());
  const double center_y = armor.xyz.y() + radius * std::sin(armor.ypr.x());
  const double center_z = armor.xyz.z();

  Eigen::VectorXd x0(11);
  x0 << center_x, 0.0, center_y, 0.0, center_z, 0.0, armor.ypr.x(), 0.0, radius, 0.0, 0.0;
  Eigen::VectorXd p0_diag(11);
  p0_diag << 1.0, 64.0, 1.0, 64.0, 1.0, 64.0, 0.4, 100.0, 1.0, 1.0, 1.0;
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
      Eigen::VectorXd c = a + b;
      c[6] = limitRad(c[6]);
      return c;
    };
  ekf_ = ExtendedKalmanFilter(x0, p0_diag.asDiagonal(), x_add);

  initialized_ = true;
  target_id_ = armor.id;
  last_armor_index_ = 0;
  jumped_ = false;
  last_stamp_ = stamp;
  lost_count = 0;
  update_count = 1;
}

void NormalTargetTracker::predict(const rclcpp::Time & stamp)
{
  if (!initialized_) {
    return;
  }
  if (stamp < last_stamp_) {
    return;
  }
  const double dt = std::max(0.0, (stamp - last_stamp_).seconds());
  last_stamp_ = stamp;

  Eigen::MatrixXd f = Eigen::MatrixXd::Identity(11, 11);
  f(0, 1) = dt;
  f(2, 3) = dt;
  f(4, 5) = dt;
  f(6, 7) = dt;

  const double a = dt * dt * dt * dt / 4.0;
  const double b = dt * dt * dt / 2.0;
  const double c = dt * dt;
  const double abs_angular_velocity = std::abs(ekf_.x[7]);
  const double middle_threshold = std::min(
    std::abs(process_noise_config_.middle_speed_angular_velocity_threshold),
    std::abs(process_noise_config_.high_speed_angular_velocity_threshold));
  const double high_threshold = std::max(
    std::abs(process_noise_config_.middle_speed_angular_velocity_threshold),
    std::abs(process_noise_config_.high_speed_angular_velocity_threshold));

  double process_noise_xy = process_noise_config_.high_speed_process_noise_xy;
  double process_noise_z = process_noise_config_.high_speed_process_noise_z;
  double process_noise_yaw = process_noise_config_.high_speed_process_noise_yaw;
  if (abs_angular_velocity < middle_threshold) {
    process_noise_xy = process_noise_config_.low_speed_process_noise_xy;
    process_noise_z = process_noise_config_.low_speed_process_noise_z;
    process_noise_yaw = process_noise_config_.low_speed_process_noise_yaw;
  } else if (abs_angular_velocity < high_threshold) {
    process_noise_xy = process_noise_config_.middle_speed_process_noise_xy;
    process_noise_z = process_noise_config_.middle_speed_process_noise_z;
    process_noise_yaw = process_noise_config_.middle_speed_process_noise_yaw;
  }

  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(11, 11);
  q(0, 0) = a * process_noise_xy;
  q(0, 1) = b * process_noise_xy;
  q(1, 0) = b * process_noise_xy;
  q(1, 1) = c * process_noise_xy;
  q(2, 2) = a * process_noise_xy;
  q(2, 3) = b * process_noise_xy;
  q(3, 2) = b * process_noise_xy;
  q(3, 3) = c * process_noise_xy;
  q(4, 4) = a * process_noise_z;
  q(4, 5) = b * process_noise_z;
  q(5, 4) = b * process_noise_z;
  q(5, 5) = c * process_noise_z;
  q(6, 6) = a * process_noise_yaw;
  q(6, 7) = b * process_noise_yaw;
  q(7, 6) = b * process_noise_yaw;
  q(7, 7) = c * process_noise_yaw;

  auto transition = [&](const Eigen::VectorXd & x) {
      Eigen::VectorXd prior = f * x;
      prior[6] = limitRad(prior[6]);
      return prior;
    };
  ekf_.predict(f, q, transition);
}

void NormalTargetTracker::update(
  const ArmorMeasurement & armor,
  const MeasurementNoiseConfig & measurement_noise_config)
{
  if (!initialized_) {
    return;
  }

  const int armor_index = matchArmorIndex(armor);
  if (armor_index != 0) {
    jumped_ = true;
  }
  last_armor_index_ = armor_index;
  ++update_count;
  lost_count = 0;

  Eigen::MatrixXd h = observationJacobian(ekf_.x, armor_index);
  const double center_yaw = std::atan2(armor.xyz.y(), armor.xyz.x());
  const double delta_angle = limitRad(armor.ypr.x() - center_yaw);

  const Eigen::Vector4d r_diag = measurementNoiseDiagonal(
    delta_angle, armor.ypd.z(), measurement_noise_config);
  const Eigen::MatrixXd r = r_diag.asDiagonal();

  auto observation = [&](const Eigen::VectorXd & x) {
      const Eigen::Vector3d xyz = armorPosition(x, armor_index);
      const Eigen::Vector3d ypd = xyzToYpd(xyz);
      Eigen::Vector4d z;
      z << ypd.x(), ypd.y(), ypd.z(), limitRad(x[6] + armor_index * kPi / 2.0);
      return z;
    };
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
      Eigen::VectorXd c = a - b;
      c[0] = limitRad(c[0]);
      c[1] = limitRad(c[1]);
      c[3] = limitRad(c[3]);
      return c;
    };

  Eigen::Vector4d z;
  z << armor.ypd.x(), armor.ypd.y(), armor.ypd.z(), armor.ypr.x();
  ekf_.update(z, h, r, observation, z_subtract);
}

bool NormalTargetTracker::active() const
{
  return initialized_;
}

bool NormalTargetTracker::converged() const
{
  return initialized_ && update_count > 3 && !diverged();
}

bool NormalTargetTracker::diverged() const
{
  const bool r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  const bool l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;
  return !(r_ok && l_ok);
}

bool NormalTargetTracker::jumped() const
{
  return jumped_;
}

std::uint8_t NormalTargetTracker::target_id() const
{
  return target_id_;
}

const Eigen::VectorXd & NormalTargetTracker::state() const
{
  return ekf_.x;
}

std::vector<geometry_msgs::msg::Pose> NormalTargetTracker::predictedArmors() const
{
  std::vector<geometry_msgs::msg::Pose> poses;
  if (!initialized_) {
    return poses;
  }

  poses.reserve(static_cast<std::size_t>(armor_num_));
  for (int i = 0; i < armor_num_; ++i) {
    const Eigen::Vector3d xyz = armorPosition(ekf_.x, i);
    const double yaw = limitRad(ekf_.x[6] + i * kPi / 2.0);
    geometry_msgs::msg::Pose pose;
    pose.position.x = xyz.x();
    pose.position.y = xyz.y();
    pose.position.z = xyz.z();
    pose.orientation = yawToQuaternion(yaw);
    poses.push_back(std::move(pose));
  }
  return poses;
}

Eigen::Vector3d NormalTargetTracker::armorPosition(const Eigen::VectorXd & x, int armor_index) const
{
  const double angle = limitRad(x[6] + armor_index * kPi / 2.0);
  const bool use_l_h = armor_index == 1 || armor_index == 3;
  const double r = use_l_h ? x[8] + x[9] : x[8];
  const double armor_x = x[0] - r * std::cos(angle);
  const double armor_y = x[2] - r * std::sin(angle);
  const double armor_z = use_l_h ? x[4] + x[10] : x[4];
  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd NormalTargetTracker::observationJacobian(
  const Eigen::VectorXd & x,
  int armor_index) const
{
  const double angle = limitRad(x[6] + armor_index * kPi / 2.0);
  const bool use_l_h = armor_index == 1 || armor_index == 3;
  const double r = use_l_h ? x[8] + x[9] : x[8];

  const double dx_da = r * std::sin(angle);
  const double dy_da = -r * std::cos(angle);
  const double dx_dr = -std::cos(angle);
  const double dy_dr = -std::sin(angle);
  const double dx_dl = use_l_h ? -std::cos(angle) : 0.0;
  const double dy_dl = use_l_h ? -std::sin(angle) : 0.0;
  const double dz_dh = use_l_h ? 1.0 : 0.0;

  Eigen::MatrixXd h_armor_xyza(4, 11);
  h_armor_xyza <<
    1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl, 0,
    0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dz_dh,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;

  const Eigen::Vector3d armor_xyz = armorPosition(x, armor_index);
  const Eigen::MatrixXd h_armor_ypd = xyzToYpdJacobian(armor_xyz);

  Eigen::MatrixXd h_armor_ypda(4, 4);
  h_armor_ypda <<
    h_armor_ypd(0, 0), h_armor_ypd(0, 1), h_armor_ypd(0, 2), 0,
    h_armor_ypd(1, 0), h_armor_ypd(1, 1), h_armor_ypd(1, 2), 0,
    h_armor_ypd(2, 0), h_armor_ypd(2, 1), h_armor_ypd(2, 2), 0,
    0, 0, 0, 1;

  return h_armor_ypda * h_armor_xyza;
}

std::vector<Eigen::Vector4d> NormalTargetTracker::predictedArmorStates() const
{
  std::vector<Eigen::Vector4d> states;
  states.reserve(static_cast<std::size_t>(armor_num_));
  for (int i = 0; i < armor_num_; ++i) {
    const Eigen::Vector3d xyz = armorPosition(ekf_.x, i);
    states.push_back({xyz.x(), xyz.y(), xyz.z(), limitRad(ekf_.x[6] + i * kPi / 2.0)});
  }
  return states;
}

int NormalTargetTracker::matchArmorIndex(const ArmorMeasurement & armor) const
{
  auto states = predictedArmorStates();
  std::vector<std::pair<Eigen::Vector4d, int>> indexed_states;
  indexed_states.reserve(states.size());
  for (int i = 0; i < armor_num_; ++i) {
    indexed_states.emplace_back(states[static_cast<std::size_t>(i)], i);
  }

  std::sort(
    indexed_states.begin(),
    indexed_states.end(),
    [](const auto & lhs, const auto & rhs) {
      const Eigen::Vector3d lhs_xyz = lhs.first.template head<3>();
      const Eigen::Vector3d rhs_xyz = rhs.first.template head<3>();
      return xyzToYpd(lhs_xyz).z() < xyzToYpd(rhs_xyz).z();
    });

  double min_angle_error = std::numeric_limits<double>::max();
  int best_index = last_armor_index_;
  const int candidate_count = std::min(3, static_cast<int>(indexed_states.size()));
  for (int i = 0; i < candidate_count; ++i) {
    const auto & xyza = indexed_states[static_cast<std::size_t>(i)].first;
    const Eigen::Vector3d ypd = xyzToYpd(xyza.head<3>());
    const double angle_error =
      std::abs(limitRad(armor.ypr.x() - xyza[3])) +
      std::abs(limitRad(armor.ypd.x() - ypd.x()));
    if (angle_error < min_angle_error) {
      min_angle_error = angle_error;
      best_index = indexed_states[static_cast<std::size_t>(i)].second;
    }
  }
  return best_index;
}

}  // namespace aim_predictor
