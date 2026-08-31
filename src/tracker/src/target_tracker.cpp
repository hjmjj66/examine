#include "tracker/target_tracker.hpp"

#include "tracker/factors.hpp"

#include <boost/make_shared.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/noiseModel/Isotropic.h>
#include <gtsam/nonlinear/PriorFactor.h>

namespace tracker
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfLightbarLength = 0.028;
constexpr double kSmallArmorHalfWidth = 0.0675;
constexpr double kBigArmorHalfWidth = 0.115;
constexpr double kMinimumSigma = 1e-9;

double positiveSigma(double value)
{
  return std::isfinite(value) && value > kMinimumSigma ? value : 1.0;
}

gtsam::SharedNoiseModel isotropicNoise(std::size_t dimension, double sigma)
{
  return gtsam::noiseModel::Isotropic::Sigma(
    static_cast<int>(dimension), positiveSigma(sigma));
}

}  // namespace

TargetTracker::TargetTracker()
: TargetTracker(TrackerConfig{})
{
}

TargetTracker::TargetTracker(const TrackerConfig & config)
: config_(config)
{
  config_.window_size = std::max<std::size_t>(1, config_.window_size);
}

TargetTracker::TargetTracker(
  const TrackerConfig & config, const std::array<CameraCalibration, 3> & calibrations)
: TargetTracker(config)
{
  for (std::size_t i = 0; i < calibrations_.size(); ++i) {
    calibrations_[i].camera_matrix = calibrations[i].camera_matrix.clone();
    calibrations_[i].distortion_coefficients = calibrations[i].distortion_coefficients.clone();
    calibrations_[i].camera_to_world = calibrations[i].camera_to_world;
  }
}

void TargetTracker::setCameraCalibration(
  CameraSource source, const CameraCalibration & calibration)
{
  const std::size_t index = cameraIndex(source);
  calibrations_[index].camera_matrix = calibration.camera_matrix.clone();
  calibrations_[index].distortion_coefficients = calibration.distortion_coefficients.clone();
  calibrations_[index].camera_to_world = calibration.camera_to_world;
}

bool TargetTracker::initialize(const ArmorObservation & observation)
{
  target_id_ = observation.target_id;
  initialized_ = true;
  diverged_ = false;
  jumped_ = false;
  update_count_ = 1;
  last_armor_index_ = 0;
  last_stamp_ = observation.stamp;
  next_frame_id_ = 1;
  next_pose_id_ = 1;
  last_frame_id_ = 0;
  frames_.clear();
  pending_graph_.resize(0);
  pending_values_.clear();
  factor_keys_.clear();
  needs_rebuild_ = false;
  optimization_failed_ = false;

  initializeState(observation);

  FrameRecord frame;
  frame.stamp = observation.stamp;
  frame.frame_id = 0;
  frame.measurements.push_back(MeasurementRecord{observation, 0, poseKey(0)});
  frames_.push_back(std::move(frame));

  initializeGraph();
  diverged_ = !validGeometry(state_);
  return true;
}

bool TargetTracker::acceptTimestamp(const builtin_interfaces::msg::Time & stamp) const
{
  return !initialized_ || !isOlder(stamp, last_stamp_);
}

bool TargetTracker::predict(const builtin_interfaces::msg::Time & stamp)
{
  if (!initialized_ || !acceptTimestamp(stamp)) {
    return false;
  }

  const double dt = std::max(0.0, secondsBetween(last_stamp_, stamp));
  state_[0] += state_[1] * dt;
  state_[2] += state_[3] * dt;
  state_[4] += state_[5] * dt;
  state_[6] = wrapAngle(state_[6] + state_[7] * dt);
  last_stamp_ = stamp;
  if (!finiteState(state_)) {
    diverged_ = true;
    return false;
  }
  return true;
}

bool TargetTracker::addMeasurement(const ArmorObservation & observation)
{
  if (!initialized_) {
    return initialize(observation);
  }
  if (observation.target_id != target_id_ || !acceptTimestamp(observation.stamp)) {
    return false;
  }

  const std::size_t armor_index = inferArmorIndex(observation);
  if (armor_index != last_armor_index_) {
    jumped_ = true;
  }
  last_armor_index_ = armor_index;
  ++update_count_;
  last_stamp_ = observation.stamp;

  if (!frames_.empty() && !isOlder(observation.stamp, frames_.back().stamp) &&
    !isOlder(frames_.back().stamp, observation.stamp))
  {
    MeasurementRecord measurement{observation, armor_index, poseKey(next_pose_id_++)};
    frames_.back().measurements.push_back(measurement);
    queueMeasurement(frames_.back(), measurement);
    return true;
  }

  FrameRecord frame;
  frame.stamp = observation.stamp;
  frame.frame_id = next_frame_id_++;
  frame.measurements.push_back(
    MeasurementRecord{observation, armor_index, poseKey(next_pose_id_++)});
  frames_.push_back(std::move(frame));

  if (frames_.size() > config_.window_size) {
    frames_.pop_front();
    pending_graph_.resize(0);
    pending_values_.clear();
    needs_rebuild_ = true;
    return true;
  }

  FrameRecord * current = &frames_.back();
  FrameRecord * previous = frames_.size() > 1 ? &frames_[frames_.size() - 2] : nullptr;
  queueFrame(*current, previous);
  return true;
}

bool TargetTracker::optimize()
{
  if (!initialized_ || diverged_ || !isam_ || optimization_failed_) {
    return false;
  }

  const Eigen::VectorXd previous_state = state_;
  const gtsam::Values previous_values = last_values_;
  try {
    bool updated = true;
    if (needs_rebuild_) {
      updated = rebuildGraph();
    } else if (pending_graph_.size() > 0) {
      isam_->update(pending_graph_, pending_values_);
      pending_graph_.resize(0);
      pending_values_.clear();
      const gtsam::Values estimate = isam_->calculateEstimate();
      last_values_ = estimate;
    }
    if (!updated) {
      state_ = previous_state;
      last_values_ = previous_values;
      return false;
    }

    if (frames_.empty() || !setStateFromValues(last_values_, frames_.back().frame_id)) {
      state_ = previous_state;
      last_values_ = previous_values;
      diverged_ = true;
      return false;
    }
    if (!finiteState(state_) || !validGeometry(state_)) {
      state_ = previous_state;
      last_values_ = previous_values;
      diverged_ = true;
      return false;
    }
    return true;
  } catch (...) {
    state_ = previous_state;
    last_values_ = previous_values;
    pending_graph_.resize(0);
    pending_values_.clear();
    needs_rebuild_ = false;
    optimization_failed_ = true;
    return false;
  }
}

bool TargetTracker::active() const
{
  return initialized_;
}

bool TargetTracker::converged() const
{
  return initialized_ && !diverged_ &&
    update_count_ >= config_.lifecycle.confirmation_count;
}

bool TargetTracker::diverged() const
{
  return diverged_;
}

bool TargetTracker::jumped() const
{
  return jumped_;
}

std::uint8_t TargetTracker::target_id() const
{
  return target_id_;
}

const Eigen::VectorXd & TargetTracker::state() const
{
  return state_;
}

aim_msgs::msg::TargetState TargetTracker::toMessage() const
{
  std_msgs::msg::Header header;
  header.stamp = last_stamp_;
  return toMessage(header);
}

aim_msgs::msg::TargetState TargetTracker::toMessage(const std_msgs::msg::Header & header) const
{
  aim_msgs::msg::TargetState message;
  message.header = header;
  message.id = target_id_;
  message.tracking = active() && !diverged_;
  message.converged = converged();
  message.jumped = jumped_;
  if (state_.size() != 11) {
    return message;
  }

  message.center.x = state_[0];
  message.center.y = state_[2];
  message.center.z = state_[4];
  message.velocity.x = state_[1];
  message.velocity.y = state_[3];
  message.velocity.z = state_[5];
  message.yaw = wrapAngle(state_[6]);
  message.angular_velocity = state_[7];
  message.radius = state_[8];
  message.radius_offset = state_[9];
  message.height_offset = state_[10];

  message.predicted_armors.reserve(4);
  for (std::size_t index = 0; index < 4; ++index) {
    const bool has_offset = index == 1 || index == 3;
    const double radius = state_[8] + (has_offset ? state_[9] : 0.0);
    const double yaw = wrapAngle(state_[6] + static_cast<double>(index) * kPi / 2.0);
    geometry_msgs::msg::Pose pose;
    pose.position.x = state_[0] - radius * std::cos(yaw);
    pose.position.y = state_[2] - radius * std::sin(yaw);
    pose.position.z = state_[4] + (has_offset ? state_[10] : 0.0);
    pose.orientation = yawQuaternion(yaw);
    message.predicted_armors.push_back(std::move(pose));
  }
  return message;
}

std::size_t TargetTracker::frameCount() const
{
  return frames_.size();
}

std::size_t TargetTracker::windowSize() const
{
  return config_.window_size;
}

std::optional<builtin_interfaces::msg::Time> TargetTracker::oldestFrameStamp() const
{
  if (frames_.empty()) {
    return std::nullopt;
  }
  return frames_.front().stamp;
}

std::optional<builtin_interfaces::msg::Time> TargetTracker::latestFrameStamp() const
{
  if (frames_.empty()) {
    return std::nullopt;
  }
  return frames_.back().stamp;
}

const gtsam::Values & TargetTracker::values() const
{
  return last_values_;
}

bool TargetTracker::allFactorKeysHaveValues() const
{
  for (const gtsam::Key key : factor_keys_) {
    if (!last_values_.exists(key)) {
      return false;
    }
  }
  return true;
}

std::size_t TargetTracker::cameraIndex(CameraSource source)
{
  switch (source) {
    case CameraSource::Front0:
      return 0;
    case CameraSource::Front1:
      return 1;
    case CameraSource::Back:
      return 2;
  }
  return 0;
}

bool TargetTracker::isOlder(
  const builtin_interfaces::msg::Time & lhs,
  const builtin_interfaces::msg::Time & rhs)
{
  return lhs.sec < rhs.sec || (lhs.sec == rhs.sec && lhs.nanosec < rhs.nanosec);
}

double TargetTracker::secondsBetween(
  const builtin_interfaces::msg::Time & lhs,
  const builtin_interfaces::msg::Time & rhs)
{
  return static_cast<double>(rhs.sec - lhs.sec) +
    1e-9 * static_cast<double>(rhs.nanosec) -
    1e-9 * static_cast<double>(lhs.nanosec);
}

double TargetTracker::wrapAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double TargetTracker::yawFromPose(const geometry_msgs::msg::Pose & pose)
{
  const double sin_yaw = 2.0 * (pose.orientation.w * pose.orientation.z +
    pose.orientation.x * pose.orientation.y);
  const double cos_yaw = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z);
  return wrapAngle(std::atan2(sin_yaw, cos_yaw));
}

gtsam::Pose3 TargetTracker::toPose3(const geometry_msgs::msg::Pose & pose)
{
  return gtsam::Pose3(
    gtsam::Rot3::Ypr(yawFromPose(pose), 0.0, 0.0), toPoint3(pose.position));
}

gtsam::Point3 TargetTracker::toPoint3(const geometry_msgs::msg::Point & point)
{
  return gtsam::Point3(point.x, point.y, point.z);
}

geometry_msgs::msg::Quaternion TargetTracker::yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.w = std::cos(yaw / 2.0);
  quaternion.z = std::sin(yaw / 2.0);
  return quaternion;
}

bool TargetTracker::finiteState(const Eigen::VectorXd & state)
{
  return state.size() == 11 && state.array().isFinite().all();
}

double TargetTracker::cameraScale(CameraSource source) const
{
  double scale = 1.0;
  switch (source) {
    case CameraSource::Front0:
      scale = config_.camera_noise_scales.front_0;
      break;
    case CameraSource::Front1:
      scale = config_.camera_noise_scales.front_1;
      break;
    case CameraSource::Back:
      scale = config_.camera_noise_scales.back;
      break;
  }
  return positiveSigma(scale);
}

std::size_t TargetTracker::inferArmorIndex(const ArmorObservation & observation) const
{
  const gtsam::Point3 measured = toPoint3(observation.world_pose.position);
  const double measured_yaw = yawFromPose(observation.world_pose);
  double best_error = std::numeric_limits<double>::infinity();
  std::size_t best_index = last_armor_index_;
  for (std::size_t index = 0; index < 4; ++index) {
    const bool has_offset = index == 1 || index == 3;
    const double armor_yaw = wrapAngle(state_[6] + static_cast<double>(index) * kPi / 2.0);
    const double radius = state_[8] + (has_offset ? state_[9] : 0.0);
    const gtsam::Point3 expected(
      state_[0] - radius * std::cos(armor_yaw),
      state_[2] - radius * std::sin(armor_yaw),
      state_[4] + (has_offset ? state_[10] : 0.0));
    const double position_error = (measured - expected).norm();
    const double yaw_error = std::abs(wrapAngle(measured_yaw - armor_yaw));
    const double error = position_error + 0.1 * yaw_error;
    if (error < best_error) {
      best_error = error;
      best_index = index;
    }
  }
  return best_index;
}

bool TargetTracker::validGeometry(const Eigen::VectorXd & state) const
{
  if (!finiteState(state)) {
    return false;
  }
  const double radius = state[8];
  const double offset_radius = radius + state[9];
  const double min_radius = config_.lifecycle.min_radius;
  const double max_radius = config_.lifecycle.max_radius;
  return radius > min_radius && radius < max_radius &&
    offset_radius > min_radius && offset_radius < max_radius;
}

void TargetTracker::initializeState(const ArmorObservation & observation)
{
  state_.resize(11);
  const double yaw = yawFromPose(observation.world_pose);
  const double radius = config_.geometry_initialization.radius;
  state_ <<
    observation.world_pose.position.x + radius * std::cos(yaw), 0.0,
    observation.world_pose.position.y + radius * std::sin(yaw), 0.0,
    observation.world_pose.position.z, 0.0, yaw, 0.0, radius,
    config_.geometry_initialization.radius_offset,
    config_.geometry_initialization.height_offset;
}

void TargetTracker::initializeGraph()
{
  isam_ = std::make_unique<gtsam::ISAM2>();
  last_values_.clear();
  pending_graph_.resize(0);
  pending_values_.clear();
  factor_keys_.clear();
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  addSharedValuesAndPriors(graph, values);
  addFrame(graph, values, frames_.front(), nullptr, true);
  last_values_ = values;
  try {
    isam_->update(graph, values);
    last_values_ = isam_->calculateEstimate();
  } catch (...) {
    // The initial state remains usable for prediction; optimize() reports later failures.
    optimization_failed_ = true;
  }
}

gtsam::Key TargetTracker::frameKey(char prefix, std::size_t frame_id) const
{
  return gtsam::Symbol(prefix, frame_id).key();
}

gtsam::Key TargetTracker::poseKey(std::size_t pose_id) const
{
  return gtsam::Symbol('p', pose_id).key();
}

void TargetTracker::insertIfMissing(
  gtsam::Values & values, gtsam::Key key, const gtsam::Point3 & value) const
{
  if (!values.exists(key)) {
    values.insert(key, value);
  }
}

void TargetTracker::insertIfMissing(
  gtsam::Values & values, gtsam::Key key, const gtsam::Vector3 & value) const
{
  if (!values.exists(key)) {
    values.insert(key, value);
  }
}

void TargetTracker::insertIfMissing(
  gtsam::Values & values, gtsam::Key key, const gtsam::Rot2 & value) const
{
  if (!values.exists(key)) {
    values.insert(key, value);
  }
}

void TargetTracker::insertIfMissing(
  gtsam::Values & values, gtsam::Key key, const gtsam::Pose3 & value) const
{
  if (!values.exists(key)) {
    values.insert(key, value);
  }
}

void TargetTracker::insertIfMissing(gtsam::Values & values, gtsam::Key key, double value) const
{
  if (!values.exists(key)) {
    values.insert(key, value);
  }
}

void TargetTracker::addFactor(
  gtsam::NonlinearFactorGraph & graph, const gtsam::NonlinearFactor::shared_ptr & factor)
{
  graph.add(factor);
  for (const gtsam::Key key : factor->keys()) {
    factor_keys_.insert(key);
  }
}

void TargetTracker::addSharedValuesAndPriors(
  gtsam::NonlinearFactorGraph & graph, gtsam::Values & values)
{
  const gtsam::Key radius_key = frameKey('g', 0);
  const gtsam::Key radius_offset_key = frameKey('d', 0);
  const gtsam::Key height_offset_key = frameKey('h', 0);
  double radius = state_[8];
  double radius_offset = state_[9];
  double height_offset = state_[10];
  if (last_values_.exists(radius_key)) {
    radius = last_values_.at<double>(radius_key);
  }
  if (last_values_.exists(radius_offset_key)) {
    radius_offset = last_values_.at<double>(radius_offset_key);
  }
  if (last_values_.exists(height_offset_key)) {
    height_offset = last_values_.at<double>(height_offset_key);
  }
  insertIfMissing(values, radius_key, radius);
  insertIfMissing(values, radius_offset_key, radius_offset);
  insertIfMissing(values, height_offset_key, height_offset);
  addFactor(graph, boost::make_shared<gtsam::PriorFactor<double>>(
      radius_key, radius, isotropicNoise(1, config_.sigma.prior_sigma[8])));
  addFactor(graph, boost::make_shared<gtsam::PriorFactor<double>>(
      radius_offset_key, radius_offset, isotropicNoise(1, config_.sigma.prior_sigma[9])));
  addFactor(graph, boost::make_shared<gtsam::PriorFactor<double>>(
      height_offset_key, height_offset, isotropicNoise(1, config_.sigma.prior_sigma[10])));
}

void TargetTracker::addFrame(
  gtsam::NonlinearFactorGraph & graph,
  gtsam::Values & values,
  const FrameRecord & frame,
  const FrameRecord * previous,
  bool add_priors)
{
  const gtsam::Key x_key = frameKey('x', frame.frame_id);
  const gtsam::Key v_key = frameKey('v', frame.frame_id);
  const gtsam::Key r_key = frameKey('r', frame.frame_id);
  const gtsam::Key w_key = frameKey('w', frame.frame_id);
  gtsam::Point3 initial_x(state_[0], state_[2], state_[4]);
  gtsam::Vector3 initial_v(state_[1], state_[3], state_[5]);
  gtsam::Rot2 initial_r = gtsam::Rot2::fromAngle(state_[6]);
  double initial_w = state_[7];
  if (last_values_.exists(x_key)) {
    initial_x = last_values_.at<gtsam::Point3>(x_key);
  }
  if (last_values_.exists(v_key)) {
    initial_v = last_values_.at<gtsam::Vector3>(v_key);
  }
  if (last_values_.exists(r_key)) {
    initial_r = last_values_.at<gtsam::Rot2>(r_key);
  }
  if (last_values_.exists(w_key)) {
    initial_w = last_values_.at<double>(w_key);
  }
  insertIfMissing(values, x_key, initial_x);
  insertIfMissing(values, v_key, initial_v);
  insertIfMissing(values, r_key, initial_r);
  insertIfMissing(values, w_key, initial_w);

  if (previous != nullptr) {
    const double dt = std::max(0.0, secondsBetween(previous->stamp, frame.stamp));
    const auto & band = std::abs(state_[7]) < config_.process_noise.middle_speed_angular_velocity_threshold ?
      config_.process_noise.low_speed :
      (std::abs(state_[7]) < config_.process_noise.high_speed_angular_velocity_threshold ?
      config_.process_noise.middle_speed : config_.process_noise.high_speed);
    const double translation_sigma =
      config_.sigma.translation_sigma * std::sqrt(positiveSigma(band.process_noise_xy));
    const double velocity_sigma =
      config_.sigma.velocity_sigma * std::sqrt(positiveSigma(band.process_noise_xy));
    const double yaw_sigma =
      config_.sigma.yaw_sigma * std::sqrt(positiveSigma(band.process_noise_yaw));
    const double yaw_velocity_sigma =
      config_.sigma.yaw_velocity_sigma * std::sqrt(positiveSigma(band.process_noise_yaw));
    addFactor(graph, boost::make_shared<TranslationFactor>(
        frameKey('x', previous->frame_id), frameKey('v', previous->frame_id), x_key, dt,
        isotropicNoise(3, translation_sigma)));
    addFactor(graph, boost::make_shared<VelocityFactor>(
        frameKey('v', previous->frame_id), v_key, isotropicNoise(3, velocity_sigma)));
    addFactor(graph, boost::make_shared<YawFactor>(
        frameKey('r', previous->frame_id), frameKey('w', previous->frame_id), r_key, dt,
        isotropicNoise(1, yaw_sigma)));
    addFactor(graph, boost::make_shared<VyawFactor>(
        frameKey('w', previous->frame_id), w_key, isotropicNoise(1, yaw_velocity_sigma)));
  }

  if (add_priors && previous == nullptr) {
    addFactor(graph, boost::make_shared<gtsam::PriorFactor<gtsam::Point3>>(
        x_key, initial_x, isotropicNoise(3, config_.sigma.prior_sigma[0])));
    addFactor(graph, boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        v_key, initial_v, isotropicNoise(3, config_.sigma.prior_sigma[1])));
    addFactor(graph, boost::make_shared<gtsam::PriorFactor<gtsam::Rot2>>(
        r_key, initial_r, isotropicNoise(1, config_.sigma.prior_sigma[6])));
    addFactor(graph, boost::make_shared<gtsam::PriorFactor<double>>(
        w_key, initial_w, isotropicNoise(1, config_.sigma.prior_sigma[7])));
  }

  for (const MeasurementRecord & measurement : frame.measurements) {
    gtsam::Pose3 initial_pose = toPose3(measurement.observation.camera_pose);
    if (last_values_.exists(measurement.pose_key)) {
      initial_pose = last_values_.at<gtsam::Pose3>(measurement.pose_key);
    }
    insertIfMissing(values, measurement.pose_key, initial_pose);
    addMeasurementFactors(graph, values, frame, measurement);
    addFactor(graph, boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        measurement.pose_key, initial_pose, isotropicNoise(6, 1.0)));
  }
}

void TargetTracker::addMeasurementFactors(
  gtsam::NonlinearFactorGraph & graph,
  gtsam::Values & values,
  const FrameRecord & frame,
  const MeasurementRecord & measurement)
{
  const gtsam::Key radius_key = frameKey('g', 0);
  const gtsam::Key radius_offset_key = frameKey('d', 0);
  const gtsam::Key height_offset_key = frameKey('h', 0);
  const gtsam::Key yaw_key = frameKey('r', frame.frame_id);
  const gtsam::Key position_key = frameKey('x', frame.frame_id);
  const auto & calibration = calibrations_[cameraIndex(measurement.observation.source)];
  const double scale = cameraScale(measurement.observation.source);
  addFactor(graph, boost::make_shared<ArmorGeometryFactor>(
      measurement.pose_key, radius_key, radius_offset_key, height_offset_key, yaw_key,
      position_key, calibration.camera_to_world, measurement.armor_index,
      isotropicNoise(4, config_.sigma.geometry_sigma[0] * scale)));

  if (calibrationIsUsable(measurement.observation.source)) {
    std::array<gtsam::Point2, 4> corners;
    for (std::size_t index = 0; index < corners.size(); ++index) {
      corners[index] = gtsam::Point2(
        measurement.observation.corners[index].x, measurement.observation.corners[index].y);
    }
    addFactor(graph, boost::make_shared<ArmorReprojFactor>(
        measurement.pose_key, calibration.camera_matrix,
        calibration.distortion_coefficients, armorPoints(measurement.observation), corners,
        isotropicNoise(8, config_.sigma.pixel_sigma * scale)));
  }
  (void)values;
}

void TargetTracker::queueFrame(const FrameRecord & frame, const FrameRecord * previous)
{
  addFrame(pending_graph_, pending_values_, frame, previous, false);
  insertIfMissing(last_values_, frameKey('x', frame.frame_id), gtsam::Point3(state_[0], state_[2], state_[4]));
  insertIfMissing(last_values_, frameKey('v', frame.frame_id), gtsam::Vector3(state_[1], state_[3], state_[5]));
  insertIfMissing(last_values_, frameKey('r', frame.frame_id), gtsam::Rot2::fromAngle(state_[6]));
  insertIfMissing(last_values_, frameKey('w', frame.frame_id), state_[7]);
  for (const auto & measurement : frame.measurements) {
    insertIfMissing(last_values_, measurement.pose_key, toPose3(measurement.observation.camera_pose));
  }
}

void TargetTracker::queueMeasurement(
  const FrameRecord & frame, const MeasurementRecord & measurement)
{
  insertIfMissing(
    pending_values_, measurement.pose_key, toPose3(measurement.observation.camera_pose));
  insertIfMissing(last_values_, measurement.pose_key, toPose3(measurement.observation.camera_pose));
  addMeasurementFactors(pending_graph_, pending_values_, frame, measurement);
  addFactor(pending_graph_, boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      measurement.pose_key, toPose3(measurement.observation.camera_pose), isotropicNoise(6, 1.0)));
}

bool TargetTracker::rebuildGraph()
{
  const std::set<gtsam::Key> previous_factor_keys = factor_keys_;
  factor_keys_.clear();
  pending_graph_.resize(0);
  pending_values_.clear();
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  addSharedValuesAndPriors(graph, values);
  const FrameRecord * previous = nullptr;
  for (const FrameRecord & frame : frames_) {
    addFrame(graph, values, frame, previous, true);
    previous = &frame;
  }

  auto candidate = std::make_unique<gtsam::ISAM2>();
  try {
    candidate->update(graph, values);
    const gtsam::Values estimate = candidate->calculateEstimate();
    isam_ = std::move(candidate);
    last_values_ = estimate;
    needs_rebuild_ = false;
    return true;
  } catch (...) {
    factor_keys_ = previous_factor_keys;
    needs_rebuild_ = false;
    optimization_failed_ = true;
    return false;
  }
}

bool TargetTracker::setStateFromValues(const gtsam::Values & estimate, std::size_t frame_id)
{
  const gtsam::Key x_key = frameKey('x', frame_id);
  const gtsam::Key v_key = frameKey('v', frame_id);
  const gtsam::Key r_key = frameKey('r', frame_id);
  const gtsam::Key w_key = frameKey('w', frame_id);
  const gtsam::Key radius_key = frameKey('g', 0);
  const gtsam::Key radius_offset_key = frameKey('d', 0);
  const gtsam::Key height_offset_key = frameKey('h', 0);
  if (!estimate.exists(x_key) || !estimate.exists(v_key) || !estimate.exists(r_key) ||
    !estimate.exists(w_key) || !estimate.exists(radius_key) ||
    !estimate.exists(radius_offset_key) || !estimate.exists(height_offset_key))
  {
    return false;
  }

  const gtsam::Point3 position = estimate.at<gtsam::Point3>(x_key);
  const gtsam::Vector3 velocity = estimate.at<gtsam::Vector3>(v_key);
  const gtsam::Rot2 yaw = estimate.at<gtsam::Rot2>(r_key);
  const double angular_velocity = estimate.at<double>(w_key);
  state_[0] = position.x();
  state_[1] = velocity.x();
  state_[2] = position.y();
  state_[3] = velocity.y();
  state_[4] = position.z();
  state_[5] = velocity.z();
  state_[6] = wrapAngle(yaw.theta());
  state_[7] = angular_velocity;
  state_[8] = estimate.at<double>(radius_key);
  state_[9] = estimate.at<double>(radius_offset_key);
  state_[10] = estimate.at<double>(height_offset_key);
  last_frame_id_ = frame_id;
  return true;
}

bool TargetTracker::calibrationIsUsable(CameraSource source) const
{
  const cv::Mat & camera_matrix = calibrations_[cameraIndex(source)].camera_matrix;
  return !camera_matrix.empty() && camera_matrix.rows == 3 && camera_matrix.cols == 3;
}

std::array<gtsam::Point3, 4> TargetTracker::armorPoints(
  const ArmorObservation & observation) const
{
  const double half_width = observation.armor_class.class_id == 1 ?
    kBigArmorHalfWidth : kSmallArmorHalfWidth;
  return {
    gtsam::Point3(0.0, half_width, -kHalfLightbarLength),
    gtsam::Point3(0.0, half_width, kHalfLightbarLength),
    gtsam::Point3(0.0, -half_width, kHalfLightbarLength),
    gtsam::Point3(0.0, -half_width, -kHalfLightbarLength)};
}

}  // namespace tracker
