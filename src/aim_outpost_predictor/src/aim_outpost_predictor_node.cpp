#include "aim_outpost_predictor/aim_outpost_predictor_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.hpp>
#include <tf2_ros/qos.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace aim_outpost_predictor
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

rclcpp::QoS makeHighRateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

rclcpp::QoS makeRealtimeTfQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

}  // namespace

AimOutpostPredictorNode::ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0,
  const Eigen::MatrixXd & p0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), p(p0), identity_(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add_(std::move(x_add))
{
}

Eigen::VectorXd AimOutpostPredictorNode::ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & f,
  const Eigen::MatrixXd & q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> transition)
{
  p = f * p * f.transpose() + q;
  x = transition(x);
  return x;
}

Eigen::VectorXd AimOutpostPredictorNode::ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z,
  const Eigen::MatrixXd & h,
  const Eigen::MatrixXd & r,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> observation,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  const Eigen::MatrixXd innovation_cov = h * p * h.transpose() + r;
  const Eigen::MatrixXd k = p * h.transpose() * innovation_cov.inverse();
  p = (identity_ - k * h) * p * (identity_ - k * h).transpose() + k * r * k.transpose();

  const Eigen::VectorXd innovation = z_subtract(z, observation(x));
  x = x_add_(x, k * innovation);

  const double nis = innovation.transpose() * innovation_cov.inverse() * innovation;
  recent_nis_failures.push_back(nis > 9.4877 ? 1 : 0);
  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }
  return x;
}

void AimOutpostPredictorNode::OutpostTracker::setConfig(const TrackerConfig & config)
{
  config_ = config;
}

void AimOutpostPredictorNode::OutpostTracker::initialize(
  const ArmorMeasurement & armor,
  const rclcpp::Time & stamp)
{
  const double center_x = armor.xyz.x() + config_.init_radius * std::cos(armor.ypr.x());
  const double center_y = armor.xyz.y() + config_.init_radius * std::sin(armor.ypr.x());
  const double center_z = armor.xyz.z();

  Eigen::VectorXd x0(11);
  x0 << center_x, 0.0, center_y, 0.0, center_z, 0.0, armor.ypr.x(), 0.0, config_.init_radius,
    config_.initial_low_height_offset, config_.initial_high_height_offset;
  Eigen::VectorXd p0_diag(11);
  p0_diag << 1.0, 64.0, 1.0, 64.0, 1.0, 64.0, 1.0, 100.0, 1.0, 0.5, 0.5;

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
      Eigen::VectorXd c = a + b;
      c[6] = AimOutpostPredictorNode::limitRad(c[6]);
      return c;
    };
  ekf_ = ExtendedKalmanFilter(x0, p0_diag.asDiagonal(), x_add);

  initialized_ = true;
  jumped_ = false;
  has_primary_slot_ = false;
  primary_slot_ = 0;
  mismatch_streak_ = 0;
  update_count_ = 0;
  lost_count = 0;
  last_stamp_ = stamp;
}

void AimOutpostPredictorNode::OutpostTracker::reset()
{
  initialized_ = false;
  jumped_ = false;
  has_primary_slot_ = false;
  primary_slot_ = 0;
  mismatch_streak_ = 0;
  update_count_ = 0;
  lost_count = 0;
  last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  ekf_ = ExtendedKalmanFilter();
}

void AimOutpostPredictorNode::OutpostTracker::predict(const rclcpp::Time & stamp)
{
  if (!initialized_) {
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

  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(11, 11);
  q(0, 0) = a * config_.process_noise_xy;
  q(0, 1) = b * config_.process_noise_xy;
  q(1, 0) = b * config_.process_noise_xy;
  q(1, 1) = c * config_.process_noise_xy;
  q(2, 2) = a * config_.process_noise_xy;
  q(2, 3) = b * config_.process_noise_xy;
  q(3, 2) = b * config_.process_noise_xy;
  q(3, 3) = c * config_.process_noise_xy;
  q(4, 4) = a * config_.process_noise_z;
  q(4, 5) = b * config_.process_noise_z;
  q(5, 4) = b * config_.process_noise_z;
  q(5, 5) = c * config_.process_noise_z;
  q(6, 6) = a * config_.process_noise_yaw;
  q(6, 7) = b * config_.process_noise_yaw;
  q(7, 6) = b * config_.process_noise_yaw;
  q(7, 7) = c * config_.process_noise_yaw;

  auto transition = [&](const Eigen::VectorXd & x) {
      Eigen::VectorXd prior = f * x;
      prior[6] = AimOutpostPredictorNode::limitRad(prior[6]);
      return prior;
    };

  if (
    converged() && config_.angular_velocity_clamp > 0.0 &&
    std::abs(ekf_.x[7]) > config_.angular_velocity_clamp)
  {
    ekf_.x[7] = ekf_.x[7] > 0.0 ? config_.angular_velocity_clamp : -config_.angular_velocity_clamp;
  }

  jumped_ = false;
  ekf_.predict(f, q, transition);
}

bool AimOutpostPredictorNode::OutpostTracker::update(const std::vector<ArmorMeasurement> & armors)
{
  if (!initialized_ || armors.empty()) {
    mismatch_streak_ = 0;
    return false;
  }

  std::vector<ArmorMeasurement> observations = armors;
  std::sort(
    observations.begin(), observations.end(),
    [](const ArmorMeasurement & lhs, const ArmorMeasurement & rhs) {
      return lhs.ypd.z() < rhs.ypd.z();
    });
  if (observations.size() > static_cast<std::size_t>(kOutpostSlots)) {
    observations.resize(kOutpostSlots);
  }

  const auto slots_xyza = predictedArmorStates();
  const auto assignment = findBestAssignment(observations, slots_xyza);
  if (!assignment.valid) {
    mismatch_streak_ = 0;
    return false;
  }

  const int obs_count = assignment.obs_count;
  const int measurement_dim = obs_count * 4;
  Eigen::VectorXd z = Eigen::VectorXd::Zero(measurement_dim);
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(measurement_dim, ekf_.x.size());
  Eigen::MatrixXd r = Eigen::MatrixXd::Zero(measurement_dim, measurement_dim);
  std::array<int, kOutpostSlots> segment_slots{{0, 0, 0}};

  for (int i = 0; i < obs_count; ++i) {
    const ArmorMeasurement & armor = observations[static_cast<std::size_t>(i)];
    const int slot = assignment.obs_to_slot[static_cast<std::size_t>(i)];
    segment_slots[static_cast<std::size_t>(i)] = slot;

    const int row = 4 * i;
    z.segment<4>(row) =
      Eigen::Vector4d{armor.ypd.x(), armor.ypd.y(), armor.ypd.z(), armor.ypr.x()};
    h.block(row, 0, 4, ekf_.x.size()) = observationJacobian(ekf_.x, slot);

    const double center_yaw = std::atan2(armor.xyz.y(), armor.xyz.x());
    const double delta_angle = AimOutpostPredictorNode::limitRad(armor.ypr.x() - center_yaw);
    Eigen::VectorXd r_diag(4);
    r_diag << 4e-3,
      4e-3,
      std::log(std::abs(delta_angle) + 1.0) + 1.0,
      std::log(std::abs(armor.ypd.z()) + 1.0) / 200.0 + 9e-2;
    r.block(row, row, 4, 4) = r_diag.asDiagonal();
  }

  auto observation = [&](const Eigen::VectorXd & x) {
      Eigen::VectorXd hx = Eigen::VectorXd::Zero(measurement_dim);
      for (int i = 0; i < obs_count; ++i) {
        const int slot = segment_slots[static_cast<std::size_t>(i)];
        const int row = 4 * i;
        const Eigen::Vector3d xyz = armorPosition(x, slot);
        const Eigen::Vector3d ypd = AimOutpostPredictorNode::xyzToYpd(xyz);
        hx.segment<4>(row) = Eigen::Vector4d{
          ypd.x(),
          ypd.y(),
          ypd.z(),
          AimOutpostPredictorNode::limitRad(x[6] + slot * 2.0 * kPi / kOutpostSlots)};
      }
      return hx;
    };
  auto z_subtract = [measurement_dim](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
      Eigen::VectorXd c = a - b;
      const int segment_count = measurement_dim / 4;
      for (int i = 0; i < segment_count; ++i) {
        const int row = 4 * i;
        c[row + 0] = AimOutpostPredictorNode::limitRad(c[row + 0]);
        c[row + 1] = AimOutpostPredictorNode::limitRad(c[row + 1]);
        c[row + 3] = AimOutpostPredictorNode::limitRad(c[row + 3]);
      }
      return c;
    };

  const bool primary_visible =
    has_primary_slot_ && findObservationIndexForSlot(assignment, primary_slot_) >= 0;
  const bool fast_reanchor_candidate =
    has_primary_slot_ && obs_count >= 2 && !primary_visible &&
    std::isfinite(assignment.total_cost) && assignment.total_cost >= config_.fast_reanchor_cost;
  if (fast_reanchor_candidate) {
    mismatch_streak_ = std::min(mismatch_streak_ + 1, std::max(1, config_.fast_reanchor_frames));
  } else {
    mismatch_streak_ = 0;
  }

  auto primary_decision = choosePrimarySlot(assignment);
  if (fast_reanchor_candidate && mismatch_streak_ >= std::max(1, config_.fast_reanchor_frames)) {
    const int reanchor_slot = findMinCostSlot(assignment);
    if (reanchor_slot >= 0) {
      primary_decision.slot = reanchor_slot;
      mismatch_streak_ = 0;
    }
  }

  ekf_.update(z, h, r, observation, z_subtract);

  if (primary_decision.slot >= 0) {
    if (has_primary_slot_ && primary_decision.slot != primary_slot_) {
      jumped_ = true;
    }
    primary_slot_ = primary_decision.slot;
    has_primary_slot_ = true;
  }

  ++update_count_;
  lost_count = 0;
  return true;
}

bool AimOutpostPredictorNode::OutpostTracker::active() const
{
  return initialized_;
}

bool AimOutpostPredictorNode::OutpostTracker::converged() const
{
  return initialized_ && update_count_ > config_.converged_min_updates && !diverged();
}

bool AimOutpostPredictorNode::OutpostTracker::diverged() const
{
  if (!initialized_) {
    return false;
  }
  return !(ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5);
}

bool AimOutpostPredictorNode::OutpostTracker::jumped() const
{
  return jumped_;
}

bool AimOutpostPredictorNode::OutpostTracker::hasPrimaryArmor() const
{
  return initialized_ && has_primary_slot_;
}

int AimOutpostPredictorNode::OutpostTracker::primarySlot() const
{
  return has_primary_slot_ ? primary_slot_ : -1;
}

double AimOutpostPredictorNode::OutpostTracker::nisFailureRatio() const
{
  if (ekf_.recent_nis_failures.empty()) {
    return 0.0;
  }
  const int failures = std::accumulate(
    ekf_.recent_nis_failures.begin(), ekf_.recent_nis_failures.end(), 0);
  return static_cast<double>(failures) /
         static_cast<double>(ekf_.recent_nis_failures.size());
}

const Eigen::VectorXd & AimOutpostPredictorNode::OutpostTracker::state() const
{
  return ekf_.x;
}

std::vector<geometry_msgs::msg::Pose> AimOutpostPredictorNode::OutpostTracker::predictedArmors() const
{
  std::vector<geometry_msgs::msg::Pose> poses;
  if (!initialized_) {
    return poses;
  }

  poses.reserve(kOutpostSlots);
  for (int i = 0; i < kOutpostSlots; ++i) {
    const Eigen::Vector3d xyz = armorPosition(ekf_.x, i);
    const double yaw = AimOutpostPredictorNode::limitRad(ekf_.x[6] + i * 2.0 * kPi / kOutpostSlots);
    geometry_msgs::msg::Pose pose;
    pose.position.x = xyz.x();
    pose.position.y = xyz.y();
    pose.position.z = xyz.z();
    pose.orientation = AimOutpostPredictorNode::yawToQuaternion(yaw);
    poses.push_back(pose);
  }
  return poses;
}

geometry_msgs::msg::Pose AimOutpostPredictorNode::OutpostTracker::primaryArmorPose() const
{
  const auto poses = predictedArmors();
  if (poses.empty()) {
    return geometry_msgs::msg::Pose{};
  }
  if (!has_primary_slot_) {
    return poses.front();
  }
  const int slot = std::clamp(primary_slot_, 0, kOutpostSlots - 1);
  return poses[static_cast<std::size_t>(slot)];
}

double AimOutpostPredictorNode::OutpostTracker::outpostMatchCost(
  const ArmorMeasurement & armor,
  const Eigen::Vector4d & slot_xyza) const
{
  const Eigen::Vector3d slot_ypd = AimOutpostPredictorNode::xyzToYpd(slot_xyza.head<3>());
  const double yaw_error = std::abs(AimOutpostPredictorNode::limitRad(armor.ypd.x() - slot_ypd.x()));
  const double pitch_error = std::abs(AimOutpostPredictorNode::limitRad(armor.ypd.y() - slot_ypd.y()));
  const double distance_error = std::abs(armor.ypd.z() - slot_ypd.z());
  const double angle_error = std::abs(AimOutpostPredictorNode::limitRad(armor.ypr.x() - slot_xyza[3]));
  const double z_error = std::abs(armor.xyz.z() - slot_xyza[2]);

  if (
    yaw_error > config_.match_yaw_gate || pitch_error > config_.match_pitch_gate ||
    distance_error > config_.match_distance_gate || angle_error > config_.match_angle_gate ||
    z_error > config_.match_z_gate)
  {
    return std::numeric_limits<double>::infinity();
  }

  return yaw_error / config_.match_yaw_sigma + pitch_error / config_.match_pitch_sigma +
         distance_error / config_.match_distance_sigma + angle_error / config_.match_angle_sigma +
         z_error / config_.match_z_sigma;
}

AimOutpostPredictorNode::OutpostTracker::OutpostAssignment
AimOutpostPredictorNode::OutpostTracker::findBestAssignment(
  const std::vector<ArmorMeasurement> & armors,
  const std::vector<Eigen::Vector4d> & slot_xyza) const
{
  OutpostAssignment result;
  result.obs_count = std::min(static_cast<int>(armors.size()), kOutpostSlots);
  if (result.obs_count <= 0 || static_cast<int>(slot_xyza.size()) < kOutpostSlots) {
    return result;
  }

  std::array<std::array<double, kOutpostSlots>, kOutpostSlots> cost_matrix{};
  for (int i = 0; i < result.obs_count; ++i) {
    for (int slot = 0; slot < kOutpostSlots; ++slot) {
      double cost = outpostMatchCost(armors[static_cast<std::size_t>(i)], slot_xyza[slot]);
      if (has_primary_slot_ && std::abs(ekf_.x[7]) > 0.4 && slot != primary_slot_) {
        if (ekf_.x[7] > 0.0) {
          const int forward_steps = (slot - primary_slot_ + kOutpostSlots) % kOutpostSlots;
          if (forward_steps == 2) {
            cost += config_.reverse_direction_penalty;
          }
        } else {
          const int backward_steps = (primary_slot_ - slot + kOutpostSlots) % kOutpostSlots;
          if (backward_steps == 2) {
            cost += config_.reverse_direction_penalty;
          }
        }
      }
      if (cost > config_.max_pair_cost) {
        cost = std::numeric_limits<double>::infinity();
      }
      cost_matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(slot)] = cost;
    }
  }

  std::array<int, kOutpostSlots> current_slots{{-1, -1, -1}};
  std::array<double, kOutpostSlots> current_cost{{0.0, 0.0, 0.0}};

  std::function<void(int, int, double)> dfs = [&](int obs_idx, int used_slots, double total_cost) {
      if (obs_idx >= result.obs_count) {
        if (total_cost < result.total_cost) {
          result.valid = true;
          result.total_cost = total_cost;
          result.obs_to_slot = current_slots;
          result.obs_cost = current_cost;
        }
        return;
      }

      for (int slot = 0; slot < kOutpostSlots; ++slot) {
        if ((used_slots & (1 << slot)) != 0) {
          continue;
        }
        const double pair_cost = cost_matrix[static_cast<std::size_t>(obs_idx)][static_cast<std::size_t>(slot)];
        if (!std::isfinite(pair_cost)) {
          continue;
        }
        const double next_total = total_cost + pair_cost;
        if (next_total >= result.total_cost) {
          continue;
        }
        current_slots[static_cast<std::size_t>(obs_idx)] = slot;
        current_cost[static_cast<std::size_t>(obs_idx)] = pair_cost;
        dfs(obs_idx + 1, used_slots | (1 << slot), next_total);
        current_slots[static_cast<std::size_t>(obs_idx)] = -1;
        current_cost[static_cast<std::size_t>(obs_idx)] = 0.0;
      }
    };

  dfs(0, 0, 0.0);
  return result;
}

AimOutpostPredictorNode::OutpostTracker::OutpostPrimaryDecision
AimOutpostPredictorNode::OutpostTracker::choosePrimarySlot(const OutpostAssignment & assignment) const
{
  OutpostPrimaryDecision decision;

  if (!has_primary_slot_) {
    if (assignment.obs_count < 2) {
      return decision;
    }
    decision.slot = findMinCostSlot(assignment);
    return decision;
  }

  decision.slot = primary_slot_;
  if (assignment.obs_count < 2) {
    return decision;
  }

  if (findObservationIndexForSlot(assignment, primary_slot_) >= 0) {
    return decision;
  }

  if (std::abs(ekf_.x[7]) <= 0.4) {
    return decision;
  }

  const int preferred_slot =
    (primary_slot_ + (ekf_.x[7] > 0.0 ? 1 : -1) + kOutpostSlots) % kOutpostSlots;
  if (findObservationIndexForSlot(assignment, preferred_slot) >= 0) {
    decision.slot = preferred_slot;
  }
  return decision;
}

int AimOutpostPredictorNode::OutpostTracker::findObservationIndexForSlot(
  const OutpostAssignment & assignment,
  int slot) const
{
  for (int i = 0; i < assignment.obs_count; ++i) {
    if (assignment.obs_to_slot[static_cast<std::size_t>(i)] == slot) {
      return i;
    }
  }
  return -1;
}

int AimOutpostPredictorNode::OutpostTracker::findMinCostObservationIndex(
  const OutpostAssignment & assignment) const
{
  int best_obs = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int i = 0; i < assignment.obs_count; ++i) {
    const double cost = assignment.obs_cost[static_cast<std::size_t>(i)];
    if (std::isfinite(cost) && cost < best_cost) {
      best_cost = cost;
      best_obs = i;
    }
  }
  return best_obs;
}

int AimOutpostPredictorNode::OutpostTracker::findMinCostSlot(const OutpostAssignment & assignment) const
{
  const int best_obs = findMinCostObservationIndex(assignment);
  return best_obs >= 0 ? assignment.obs_to_slot[static_cast<std::size_t>(best_obs)] : -1;
}

Eigen::Vector3d AimOutpostPredictorNode::OutpostTracker::armorPosition(
  const Eigen::VectorXd & x,
  int slot) const
{
  const double angle = AimOutpostPredictorNode::limitRad(x[6] + slot * 2.0 * kPi / kOutpostSlots);
  double armor_z = x[4];
  if (slot == 0) {
    armor_z += x[9];
  } else if (slot == 2) {
    armor_z += x[10];
  }
  return {x[0] - x[8] * std::cos(angle), x[2] - x[8] * std::sin(angle), armor_z};
}

Eigen::MatrixXd AimOutpostPredictorNode::OutpostTracker::observationJacobian(
  const Eigen::VectorXd & x,
  int slot) const
{
  const double angle = AimOutpostPredictorNode::limitRad(x[6] + slot * 2.0 * kPi / kOutpostSlots);
  const double radius = x[8];
  const double dx_da = radius * std::sin(angle);
  const double dy_da = -radius * std::cos(angle);
  const double dx_dr = -std::cos(angle);
  const double dy_dr = -std::sin(angle);
  const double dz_dlow = slot == 0 ? 1.0 : 0.0;
  const double dz_dhigh = slot == 2 ? 1.0 : 0.0;

  Eigen::MatrixXd h_armor_xyza(4, 11);
  h_armor_xyza <<
    1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, 0, 0,
    0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, dz_dlow, dz_dhigh,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;

  const Eigen::Vector3d armor_xyz = armorPosition(x, slot);
  const Eigen::MatrixXd h_armor_ypd = AimOutpostPredictorNode::xyzToYpdJacobian(armor_xyz);

  Eigen::MatrixXd h_armor_ypda(4, 4);
  h_armor_ypda <<
    h_armor_ypd(0, 0), h_armor_ypd(0, 1), h_armor_ypd(0, 2), 0,
    h_armor_ypd(1, 0), h_armor_ypd(1, 1), h_armor_ypd(1, 2), 0,
    h_armor_ypd(2, 0), h_armor_ypd(2, 1), h_armor_ypd(2, 2), 0,
    0, 0, 0, 1;

  return h_armor_ypda * h_armor_xyza;
}

std::vector<Eigen::Vector4d> AimOutpostPredictorNode::OutpostTracker::predictedArmorStates() const
{
  std::vector<Eigen::Vector4d> states;
  if (!initialized_) {
    return states;
  }
  states.reserve(kOutpostSlots);
  for (int slot = 0; slot < kOutpostSlots; ++slot) {
    const Eigen::Vector3d xyz = armorPosition(ekf_.x, slot);
    states.push_back({
      xyz.x(),
      xyz.y(),
      xyz.z(),
      AimOutpostPredictorNode::limitRad(ekf_.x[6] + slot * 2.0 * kPi / kOutpostSlots)});
  }
  return states;
}

AimOutpostPredictorNode::AimOutpostPredictorNode(const rclcpp::NodeOptions & options)
: Node("aim_outpost_predictor_node", options)
{
  declare_parameter<std::string>("front_0_armor_pose_set_topic", "/aim_solver/front_0/armor_pose_sets");
  declare_parameter<std::string>("front_1_armor_pose_set_topic", "/aim_solver/front_1/armor_pose_sets");
  declare_parameter<double>("front_0_fallback_timeout_sec", 0.2);
  declare_parameter<std::string>("outpost_state_topic", "/aim_outpost_predictor/outpost_state");
  declare_parameter<bool>("enable_visualization", false);
  declare_parameter<std::string>("visualization_topic", "/aim_outpost_predictor/visualization");
  declare_parameter<std::string>("world_frame_id", "world");
  declare_parameter<std::string>("gimbal_frame_id", "gimbal_link");
  declare_parameter<double>("tf_lookup_timeout_sec", 0.02);
  declare_parameter<double>("armor_gimbal_yaw_gate_deg", 60.0);
  declare_parameter<int>("outpost_id", 6);
  declare_parameter<int>("max_lost_count", 75);
  declare_parameter<int>("min_consecutive_detections_to_track", 1);
  declare_parameter<double>("max_nis_failure_ratio", 0.4);
  declare_parameter<double>("init_radius", tracker_config_.init_radius);
  declare_parameter<double>("initial_low_height_offset", tracker_config_.initial_low_height_offset);
  declare_parameter<double>("initial_high_height_offset", tracker_config_.initial_high_height_offset);
  declare_parameter<double>("process_noise_xy", tracker_config_.process_noise_xy);
  declare_parameter<double>("process_noise_z", tracker_config_.process_noise_z);
  declare_parameter<double>("process_noise_yaw", tracker_config_.process_noise_yaw);
  declare_parameter<double>("angular_velocity_clamp", tracker_config_.angular_velocity_clamp);
  declare_parameter<int>("converged_min_updates", tracker_config_.converged_min_updates);
  declare_parameter<double>("match_yaw_sigma", tracker_config_.match_yaw_sigma);
  declare_parameter<double>("match_pitch_sigma", tracker_config_.match_pitch_sigma);
  declare_parameter<double>("match_distance_sigma", tracker_config_.match_distance_sigma);
  declare_parameter<double>("match_angle_sigma", tracker_config_.match_angle_sigma);
  declare_parameter<double>("match_z_sigma", tracker_config_.match_z_sigma);
  declare_parameter<double>("match_yaw_gate", tracker_config_.match_yaw_gate);
  declare_parameter<double>("match_pitch_gate", tracker_config_.match_pitch_gate);
  declare_parameter<double>("match_distance_gate", tracker_config_.match_distance_gate);
  declare_parameter<double>("match_angle_gate", tracker_config_.match_angle_gate);
  declare_parameter<double>("match_z_gate", tracker_config_.match_z_gate);
  declare_parameter<double>("reverse_direction_penalty", tracker_config_.reverse_direction_penalty);
  declare_parameter<double>("max_pair_cost", tracker_config_.max_pair_cost);
  declare_parameter<double>("fast_reanchor_cost", tracker_config_.fast_reanchor_cost);
  declare_parameter<int>("fast_reanchor_frames", tracker_config_.fast_reanchor_frames);

  front_0_armor_pose_set_topic_ =
    get_parameter("front_0_armor_pose_set_topic").as_string();
  front_1_armor_pose_set_topic_ =
    get_parameter("front_1_armor_pose_set_topic").as_string();
  front_0_fallback_timeout_sec_ = std::max(
    0.0, get_parameter("front_0_fallback_timeout_sec").as_double());
  outpost_state_topic_ = get_parameter("outpost_state_topic").as_string();
  enable_visualization_ = get_parameter("enable_visualization").as_bool();
  visualization_topic_ = get_parameter("visualization_topic").as_string();
  world_frame_id_ = get_parameter("world_frame_id").as_string();
  gimbal_frame_id_ = get_parameter("gimbal_frame_id").as_string();
  tf_lookup_timeout_sec_ = get_parameter("tf_lookup_timeout_sec").as_double();
  armor_gimbal_yaw_gate_rad_ =
    get_parameter("armor_gimbal_yaw_gate_deg").as_double() * kPi / 180.0;
  outpost_id_ = static_cast<std::uint8_t>(
    std::clamp(get_parameter("outpost_id").as_int(), 0L, 255L));
  max_lost_count_ = get_parameter("max_lost_count").as_int();
  min_consecutive_detections_to_track_ = std::max(
    1, static_cast<int>(get_parameter("min_consecutive_detections_to_track").as_int()));
  max_nis_failure_ratio_ = get_parameter("max_nis_failure_ratio").as_double();
  tracker_config_.init_radius = get_parameter("init_radius").as_double();
  tracker_config_.initial_low_height_offset = get_parameter("initial_low_height_offset").as_double();
  tracker_config_.initial_high_height_offset = get_parameter("initial_high_height_offset").as_double();
  tracker_config_.process_noise_xy = get_parameter("process_noise_xy").as_double();
  tracker_config_.process_noise_z = get_parameter("process_noise_z").as_double();
  tracker_config_.process_noise_yaw = get_parameter("process_noise_yaw").as_double();
  tracker_config_.angular_velocity_clamp = get_parameter("angular_velocity_clamp").as_double();
  tracker_config_.converged_min_updates = get_parameter("converged_min_updates").as_int();
  tracker_config_.match_yaw_sigma = get_parameter("match_yaw_sigma").as_double();
  tracker_config_.match_pitch_sigma = get_parameter("match_pitch_sigma").as_double();
  tracker_config_.match_distance_sigma = get_parameter("match_distance_sigma").as_double();
  tracker_config_.match_angle_sigma = get_parameter("match_angle_sigma").as_double();
  tracker_config_.match_z_sigma = get_parameter("match_z_sigma").as_double();
  tracker_config_.match_yaw_gate = get_parameter("match_yaw_gate").as_double();
  tracker_config_.match_pitch_gate = get_parameter("match_pitch_gate").as_double();
  tracker_config_.match_distance_gate = get_parameter("match_distance_gate").as_double();
  tracker_config_.match_angle_gate = get_parameter("match_angle_gate").as_double();
  tracker_config_.match_z_gate = get_parameter("match_z_gate").as_double();
  tracker_config_.reverse_direction_penalty = get_parameter("reverse_direction_penalty").as_double();
  tracker_config_.max_pair_cost = get_parameter("max_pair_cost").as_double();
  tracker_config_.fast_reanchor_cost = get_parameter("fast_reanchor_cost").as_double();
  tracker_config_.fast_reanchor_frames = get_parameter("fast_reanchor_frames").as_int();

  tracker_.setConfig(tracker_config_);
  front_camera_arbitrator_.setFallbackTimeout(front_0_fallback_timeout_sec_);

  const auto high_rate_qos = makeHighRateQos();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_, this, true, makeRealtimeTfQos(), tf2_ros::StaticListenerQoS());
  outpost_state_pub_ =
    create_publisher<aim_msgs::msg::OutpostState>(outpost_state_topic_, high_rate_qos);
  if (enable_visualization_) {
    visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      visualization_topic_, rclcpp::SensorDataQoS());
  }
  front_0_armor_pose_sub_ = create_subscription<aim_msgs::msg::ArmorPoseSetArray>(
    front_0_armor_pose_set_topic_,
    high_rate_qos,
    [this](const aim_msgs::msg::ArmorPoseSetArray::ConstSharedPtr msg) {
      onArmorPoseSets(true, msg);
    });
  front_1_armor_pose_sub_ = create_subscription<aim_msgs::msg::ArmorPoseSetArray>(
    front_1_armor_pose_set_topic_,
    high_rate_qos,
    [this](const aim_msgs::msg::ArmorPoseSetArray::ConstSharedPtr msg) {
      onArmorPoseSets(false, msg);
    });
}

void AimOutpostPredictorNode::onArmorPoseSets(
  bool from_front_0,
  const aim_msgs::msg::ArmorPoseSetArray::ConstSharedPtr msg)
{
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(tracker_mutex_);
  std::vector<ArmorMeasurement> measurements = extractMeasurements(msg);

  if (from_front_0) {
    if (!front_camera_arbitrator_.shouldProcessFront0(!measurements.empty(), now)) {
      return;
    }
  } else if (!front_camera_arbitrator_.shouldProcessFront1(now)) {
    return;
  }

  processArmorPoseSets(msg, std::move(measurements));
}

std::vector<AimOutpostPredictorNode::ArmorMeasurement>
AimOutpostPredictorNode::extractMeasurements(
  const aim_msgs::msg::ArmorPoseSetArray::ConstSharedPtr & msg)
{
  std::vector<ArmorMeasurement> measurements;
  std::vector<ArmorMeasurement> gated_measurements;
  const bool apply_yaw_gate =
    tracker_.active() && tracker_.converged() && !tracker_.diverged() && tracker_.lost_count == 0;
  const std::optional<double> gimbal_yaw = lookupGimbalYaw(msg->header);
  for (const auto & armor_pose_set : msg->armor_pose_sets) {
    if (armor_pose_set.id != outpost_id_) {
      continue;
    }
    for (const auto & pose : armor_pose_set.armor_poses) {
      ArmorMeasurement measurement;
      measurement.id = armor_pose_set.id;
      measurement.pose = pose;
      measurement.xyz = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
      measurement.ypr = poseToYpr(pose);
      measurement.ypd = xyzToYpd(measurement.xyz);
      measurements.push_back(measurement);

      if (
        !apply_yaw_gate || !gimbal_yaw.has_value() ||
        std::abs(limitRad(measurement.ypr.x() - *gimbal_yaw)) <= armor_gimbal_yaw_gate_rad_)
      {
        gated_measurements.push_back(measurement);
      }
    }
  }

  if (!apply_yaw_gate || !gated_measurements.empty()) {
    return gated_measurements;
  }

  if (!measurements.empty() && gimbal_yaw.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "outpost yaw gate rejected all observations at stamp %.9f, falling back to ungated measurements",
      rclcpp::Time(msg->header.stamp).seconds());
  }

  return measurements;
}

void AimOutpostPredictorNode::processArmorPoseSets(
  const aim_msgs::msg::ArmorPoseSetArray::ConstSharedPtr & msg,
  std::vector<ArmorMeasurement> measurements)
{

  const rclcpp::Time stamp(msg->header.stamp);
  tracker_.setConfig(tracker_config_);
  selected_armor_poses_.clear();

  if (tracker_.active()) {
    tracker_.predict(stamp);
    ++tracker_.lost_count;
  }

  if (measurements.empty()) {
    consecutive_detection_count_ = 0;
  } else {
    std::sort(
      measurements.begin(), measurements.end(),
      [](const ArmorMeasurement & lhs, const ArmorMeasurement & rhs) {
        return lhs.ypd.z() < rhs.ypd.z();
      });
    if (measurements.size() > 3U) {
      measurements.resize(3U);
    }
    ++consecutive_detection_count_;
  }

  const bool needs_initialize =
    !tracker_.active() || tracker_.lost_count > max_lost_count_ || tracker_.diverged() ||
    tracker_.nisFailureRatio() > max_nis_failure_ratio_;

  bool updated = false;
  if (!measurements.empty()) {
    if (needs_initialize) {
      if (consecutive_detection_count_ >= min_consecutive_detections_to_track_) {
        tracker_.initialize(measurements.front(), stamp);
        updated = tracker_.update(measurements);
      }
    } else {
      updated = tracker_.update(measurements);
    }

    if (updated) {
      selected_armor_poses_.reserve(measurements.size());
      for (const auto & measurement : measurements) {
        selected_armor_poses_.push_back(measurement.pose);
      }
    }
  }

  if (tracker_.active() &&
      (tracker_.lost_count > max_lost_count_ || tracker_.diverged() ||
      tracker_.nisFailureRatio() > max_nis_failure_ratio_))
  {
    tracker_.reset();
  }

  aim_msgs::msg::OutpostState state_msg;
  state_msg.header = msg->header;
  state_msg.id = outpost_id_;
  state_msg.tracking = tracker_.active();
  state_msg.converged = tracker_.converged();
  state_msg.jumped = tracker_.jumped();
  state_msg.primary_slot = -1;

  if (tracker_.active()) {
    const Eigen::VectorXd & x = tracker_.state();
    state_msg.center.x = x[0];
    state_msg.center.y = x[2];
    state_msg.center.z = x[4];
    state_msg.velocity.x = x[1];
    state_msg.velocity.y = x[3];
    state_msg.velocity.z = x[5];
    state_msg.yaw = x[6];
    state_msg.angular_velocity = x[7];
    state_msg.radius = x[8];
    state_msg.low_height_offset = x[9];
    state_msg.high_height_offset = x[10];
    state_msg.has_primary_armor = tracker_.hasPrimaryArmor();
    state_msg.primary_slot = static_cast<int8_t>(tracker_.primarySlot());
    state_msg.primary_armor = tracker_.primaryArmorPose();
    state_msg.predicted_armors = tracker_.predictedArmors();
  }

  outpost_state_pub_->publish(state_msg);
  if (enable_visualization_ && visualization_pub_) {
    publishVisualization(msg->header, state_msg);
  }
}

void AimOutpostPredictorNode::publishVisualization(
  const std_msgs::msg::Header & header,
  const aim_msgs::msg::OutpostState & state_msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;

  for (std::size_t armor_index = 0; armor_index < state_msg.predicted_armors.size(); ++armor_index) {
    const auto & pose = state_msg.predicted_armors[armor_index];

    visualization_msgs::msg::Marker cube_marker;
    cube_marker.header = header;
    cube_marker.ns = "outpost_predicted_armor_pose";
    cube_marker.id = marker_id++;
    cube_marker.type = visualization_msgs::msg::Marker::CUBE;
    cube_marker.action = visualization_msgs::msg::Marker::ADD;
    cube_marker.pose = pose;
    cube_marker.scale.x = 0.01;
    cube_marker.scale.y = 0.135;
    cube_marker.scale.z = 0.056;
    cube_marker.color.r = 0.25F;
    cube_marker.color.g = 0.75F;
    cube_marker.color.b = 1.0F;
    cube_marker.color.a = 0.85F;
    cube_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_array.markers.push_back(cube_marker);

    visualization_msgs::msg::Marker axis_marker;
    axis_marker.header = header;
    axis_marker.ns = "outpost_predicted_armor_axis";
    axis_marker.id = marker_id++;
    axis_marker.type = visualization_msgs::msg::Marker::ARROW;
    axis_marker.action = visualization_msgs::msg::Marker::ADD;
    axis_marker.scale.x = 0.01;
    axis_marker.scale.y = 0.02;
    axis_marker.scale.z = 0.03;
    axis_marker.color.r = 1.0F;
    axis_marker.color.g = 0.2F;
    axis_marker.color.b = 0.2F;
    axis_marker.color.a = 0.95F;
    axis_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    axis_marker.points.push_back(pose.position);
    axis_marker.points.push_back(pointAlongPoseXAxis(pose, 0.1));
    marker_array.markers.push_back(axis_marker);
  }

  if (state_msg.tracking && state_msg.has_primary_armor) {
    visualization_msgs::msg::Marker primary_marker;
    primary_marker.header = header;
    primary_marker.ns = "outpost_primary_armor";
    primary_marker.id = marker_id++;
    primary_marker.type = visualization_msgs::msg::Marker::CUBE;
    primary_marker.action = visualization_msgs::msg::Marker::ADD;
    primary_marker.pose = state_msg.primary_armor;
    primary_marker.scale.x = 0.018;
    primary_marker.scale.y = 0.16;
    primary_marker.scale.z = 0.075;
    primary_marker.color.r = 0.1F;
    primary_marker.color.g = 1.0F;
    primary_marker.color.b = 0.35F;
    primary_marker.color.a = 0.95F;
    primary_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_array.markers.push_back(primary_marker);
  }

  int selected_marker_id = 0;
  for (const auto & pose : selected_armor_poses_) {
    visualization_msgs::msg::Marker selected_marker;
    selected_marker.header = header;
    selected_marker.ns = "outpost_selected_measurement";
    selected_marker.id = selected_marker_id++;
    selected_marker.type = visualization_msgs::msg::Marker::CUBE;
    selected_marker.action = visualization_msgs::msg::Marker::ADD;
    selected_marker.pose = pose;
    selected_marker.scale.x = 0.018;
    selected_marker.scale.y = 0.155;
    selected_marker.scale.z = 0.072;
    selected_marker.color.r = 1.0F;
    selected_marker.color.g = 0.9F;
    selected_marker.color.b = 0.1F;
    selected_marker.color.a = 0.95F;
    selected_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker_array.markers.push_back(selected_marker);

    visualization_msgs::msg::Marker axis_marker;
    axis_marker.header = header;
    axis_marker.ns = "outpost_selected_measurement_axis";
    axis_marker.id = selected_marker_id++;
    axis_marker.type = visualization_msgs::msg::Marker::ARROW;
    axis_marker.action = visualization_msgs::msg::Marker::ADD;
    axis_marker.scale.x = 0.014;
    axis_marker.scale.y = 0.03;
    axis_marker.scale.z = 0.04;
    axis_marker.color.r = 1.0F;
    axis_marker.color.g = 0.45F;
    axis_marker.color.b = 0.0F;
    axis_marker.color.a = 1.0F;
    axis_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    axis_marker.points.push_back(pose.position);
    axis_marker.points.push_back(pointAlongPoseXAxis(pose, 0.14));
    marker_array.markers.push_back(axis_marker);
  }

  for (int stale_id = selected_marker_id; stale_id < last_selected_marker_count_; ++stale_id) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns =
      (stale_id % 2 == 0) ? "outpost_selected_measurement" : "outpost_selected_measurement_axis";
    marker.id = stale_id;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(marker);
  }

  for (int stale_id = marker_id; stale_id < last_marker_count_; ++stale_id) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    if (stale_id < 6) {
      marker.ns = (stale_id % 2 == 0) ? "outpost_predicted_armor_pose" : "outpost_predicted_armor_axis";
    } else {
      marker.ns = "outpost_primary_armor";
    }
    marker.id = stale_id;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(marker);
  }

  last_marker_count_ = marker_id;
  last_selected_marker_count_ = selected_marker_id;
  visualization_pub_->publish(marker_array);
}

Eigen::Vector3d AimOutpostPredictorNode::poseToYpr(const geometry_msgs::msg::Pose & pose)
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

double AimOutpostPredictorNode::quaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double qw = quaternion.w;
  const double qx = quaternion.x;
  const double qy = quaternion.y;
  const double qz = quaternion.z;

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::optional<double> AimOutpostPredictorNode::lookupGimbalYaw(const std_msgs::msg::Header & header)
{
  if (!tf_buffer_) {
    return std::nullopt;
  }

  const std::string source_frame = header.frame_id.empty() ? world_frame_id_ : header.frame_id;
  try {
    const auto transform = tf_buffer_->lookupTransform(
      source_frame,
      gimbal_frame_id_,
      tf2_ros::fromMsg(header.stamp),
      tf2::durationFromSec(tf_lookup_timeout_sec_));
    return quaternionToYaw(transform.transform.rotation);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "failed to lookup gimbal yaw for outpost armor gate: target='%s' source='%s': %s",
      source_frame.c_str(), gimbal_frame_id_.c_str(), ex.what());
    return std::nullopt;
  }
}

Eigen::Vector3d AimOutpostPredictorNode::xyzToYpd(const Eigen::Vector3d & xyz)
{
  const double distance = xyz.norm();
  const double yaw = std::atan2(xyz.y(), xyz.x());
  const double pitch = std::atan2(-xyz.z(), std::hypot(xyz.x(), xyz.y()));
  return {yaw, pitch, distance};
}

Eigen::MatrixXd AimOutpostPredictorNode::xyzToYpdJacobian(const Eigen::Vector3d & xyz)
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

double AimOutpostPredictorNode::limitRad(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

geometry_msgs::msg::Quaternion AimOutpostPredictorNode::yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw * 0.5);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  return q;
}

geometry_msgs::msg::Point AimOutpostPredictorNode::pointAlongPoseXAxis(
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

}  // namespace aim_outpost_predictor
