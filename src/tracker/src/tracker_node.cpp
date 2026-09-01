#include "tracker/tracker_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include <opencv2/core.hpp>

namespace tracker
{
namespace
{

rclcpp::QoS highRateQos()
{
  return rclcpp::SensorDataQoS();
}

std::vector<double> parameterVector(const rclcpp::Node & node, const std::string & name)
{
  if (!node.has_parameter(name)) {
    return {};
  }
  return node.get_parameter(name).as_double_array();
}

cv::Mat matrixFromRowMajor(const std::vector<double> & values, int rows, int columns)
{
  if (values.size() != static_cast<std::size_t>(rows * columns)) {
    return {};
  }
  cv::Mat result(rows, columns, CV_64F);
  for (int row = 0; row < rows; ++row) {
    for (int column = 0; column < columns; ++column) {
      result.at<double>(row, column) = values[static_cast<std::size_t>(row * columns + column)];
    }
  }
  return result;
}

}  // namespace

TrackerNode::TrackerNode(const rclcpp::NodeOptions & options)
: Node("tracker_node", options)
{
  declareAndLoadParameters();
  initializeCameraInputs();
  target_state_pub_ = create_publisher<aim_msgs::msg::TargetStateArray>(
    fused_target_state_topic_, highRateQos());
  delay_statistics_pub_ = create_publisher<aim_msgs::msg::DelayStatistics>(
    delay_statistics_topic_, highRateQos());
  if (enable_visualization_) {
    visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      visualization_topic_, highRateQos());
  }
  last_delay_statistics_publish_time_ = std::chrono::steady_clock::now();
}

void TrackerNode::declareAndLoadParameters()
{
  declare_parameter<std::string>(
    "front_0_armor_pose_set_topic", "/aim_solver/front_0/armor_pose_sets");
  declare_parameter<std::string>(
    "front_1_armor_pose_set_topic", "/aim_solver/front_1/armor_pose_sets");
  declare_parameter<std::string>("back_armor_pose_set_topic", "/aim_solver/back/armor_pose_sets");
  declare_parameter<std::string>("fused_target_state_topic", "/aim_predictor/fused/target_states");
  declare_parameter<std::string>("delay_statistics_topic", "/aim_predictor/delay_statistics");
  declare_parameter<std::string>("visualization_topic", "/aim_predictor/visualization");
  declare_parameter<std::string>("world_frame_id", "gimbal_world");
  declare_parameter<bool>("enable_visualization", true);
  declare_parameter<int64_t>("window_size", 30);
  declare_parameter<int64_t>("min_consecutive_detections_to_track", 10);
  declare_parameter<int64_t>("outpost_id", 6);
  declare_parameter<double>("confirmation_min_interval_sec", 0.03);
  declare_parameter<double>("front_target_hold_sec", 0.10);
  declare_parameter<double>("target_lost_timeout_sec", 0.20);
  declare_parameter<double>("init_radius", config_.geometry_initialization.radius);
  declare_parameter<double>("init_radius_offset", config_.geometry_initialization.radius_offset);
  declare_parameter<double>("init_height_offset", config_.geometry_initialization.height_offset);
  declare_parameter<double>("low_speed_process_noise_xy", 1600.0);
  declare_parameter<double>("low_speed_process_noise_z", 1600.0);
  declare_parameter<double>("low_speed_process_noise_yaw", 400.0);
  declare_parameter<double>("middle_speed_process_noise_xy", 1600.0);
  declare_parameter<double>("middle_speed_process_noise_z", 1600.0);
  declare_parameter<double>("middle_speed_process_noise_yaw", 400.0);
  declare_parameter<double>("high_speed_process_noise_xy", 1600.0);
  declare_parameter<double>("high_speed_process_noise_z", 1600.0);
  declare_parameter<double>("high_speed_process_noise_yaw", 400.0);
  declare_parameter<double>("middle_speed_angular_velocity_threshold", 2.0);
  declare_parameter<double>("high_speed_angular_velocity_threshold", 4.0);
  declare_parameter<std::vector<double>>(
    "prior_sigma",
    std::vector<double>(config_.sigma.prior_sigma.begin(), config_.sigma.prior_sigma.end()));
  declare_parameter<double>("translation_sigma", config_.sigma.translation_sigma);
  declare_parameter<double>("velocity_sigma", config_.sigma.velocity_sigma);
  declare_parameter<double>("yaw_sigma", config_.sigma.yaw_sigma);
  declare_parameter<double>("yaw_velocity_sigma", config_.sigma.yaw_velocity_sigma);
  declare_parameter<std::vector<double>>(
    "geometry_sigma",
    std::vector<double>(config_.sigma.geometry_sigma.begin(), config_.sigma.geometry_sigma.end()));
  declare_parameter<double>("pixel_sigma", config_.sigma.pixel_sigma);
  declare_parameter<double>("front_0_noise_scale", 1.0);
  declare_parameter<double>("front_1_noise_scale", 1.0);
  declare_parameter<double>("back_noise_scale", 1.0);
  for (const std::string & prefix : {"front_0", "front_1", "back"}) {
    declare_parameter<std::vector<double>>(prefix + "_camera_matrix", std::vector<double>{});
    declare_parameter<std::vector<double>>(
      prefix + "_distortion_coefficients", std::vector<double>{});
  }

  fused_target_state_topic_ = get_parameter("fused_target_state_topic").as_string();
  delay_statistics_topic_ = get_parameter("delay_statistics_topic").as_string();
  visualization_topic_ = get_parameter("visualization_topic").as_string();
  world_frame_id_ = get_parameter("world_frame_id").as_string();
  enable_visualization_ = get_parameter("enable_visualization").as_bool();
  outpost_id_ = static_cast<std::uint8_t>(std::clamp(
      get_parameter("outpost_id").as_int(), static_cast<int64_t>(0), static_cast<int64_t>(255)));
  min_consecutive_detections_to_track_ = std::max(
    1, static_cast<int>(get_parameter("min_consecutive_detections_to_track").as_int()));
  confirmation_min_interval_sec_ = std::max(
    0.0, get_parameter("confirmation_min_interval_sec").as_double());
  front_target_hold_sec_ = std::max(0.0, get_parameter("front_target_hold_sec").as_double());
  target_lost_timeout_sec_ = std::max(0.0, get_parameter("target_lost_timeout_sec").as_double());
  loadTrackerConfig();
  loadCalibration(CameraSource::Front0, "front_0");
  loadCalibration(CameraSource::Front1, "front_1");
  loadCalibration(CameraSource::Back, "back");
}

void TrackerNode::loadTrackerConfig()
{
  config_.window_size = std::max<std::size_t>(
    1U, static_cast<std::size_t>(get_parameter("window_size").as_int()));
  config_.geometry_initialization.radius = get_parameter("init_radius").as_double();
  config_.geometry_initialization.radius_offset = get_parameter("init_radius_offset").as_double();
  config_.geometry_initialization.height_offset = get_parameter("init_height_offset").as_double();
  config_.process_noise.low_speed.process_noise_xy =
    get_parameter("low_speed_process_noise_xy").as_double();
  config_.process_noise.low_speed.process_noise_z =
    get_parameter("low_speed_process_noise_z").as_double();
  config_.process_noise.low_speed.process_noise_yaw =
    get_parameter("low_speed_process_noise_yaw").as_double();
  config_.process_noise.middle_speed.process_noise_xy =
    get_parameter("middle_speed_process_noise_xy").as_double();
  config_.process_noise.middle_speed.process_noise_z =
    get_parameter("middle_speed_process_noise_z").as_double();
  config_.process_noise.middle_speed.process_noise_yaw =
    get_parameter("middle_speed_process_noise_yaw").as_double();
  config_.process_noise.high_speed.process_noise_xy =
    get_parameter("high_speed_process_noise_xy").as_double();
  config_.process_noise.high_speed.process_noise_z =
    get_parameter("high_speed_process_noise_z").as_double();
  config_.process_noise.high_speed.process_noise_yaw =
    get_parameter("high_speed_process_noise_yaw").as_double();
  config_.process_noise.middle_speed_angular_velocity_threshold =
    get_parameter("middle_speed_angular_velocity_threshold").as_double();
  config_.process_noise.high_speed_angular_velocity_threshold =
    get_parameter("high_speed_angular_velocity_threshold").as_double();
  const auto prior_sigma = parameterVector(*this, "prior_sigma");
  if (prior_sigma.size() == config_.sigma.prior_sigma.size()) {
    std::copy(prior_sigma.begin(), prior_sigma.end(), config_.sigma.prior_sigma.begin());
  }
  config_.sigma.translation_sigma = get_parameter("translation_sigma").as_double();
  config_.sigma.velocity_sigma = get_parameter("velocity_sigma").as_double();
  config_.sigma.yaw_sigma = get_parameter("yaw_sigma").as_double();
  config_.sigma.yaw_velocity_sigma = get_parameter("yaw_velocity_sigma").as_double();
  const auto geometry_sigma = parameterVector(*this, "geometry_sigma");
  if (geometry_sigma.size() == config_.sigma.geometry_sigma.size()) {
    std::copy(geometry_sigma.begin(), geometry_sigma.end(), config_.sigma.geometry_sigma.begin());
  }
  config_.sigma.pixel_sigma = get_parameter("pixel_sigma").as_double();
  config_.camera_noise_scales.front_0 = get_parameter("front_0_noise_scale").as_double();
  config_.camera_noise_scales.front_1 = get_parameter("front_1_noise_scale").as_double();
  config_.camera_noise_scales.back = get_parameter("back_noise_scale").as_double();
}

void TrackerNode::loadCalibration(CameraSource source, const std::string & prefix)
{
  const std::size_t index = cameraIndex(source);
  calibrations_[index].camera_matrix = matrixFromRowMajor(
    parameterVector(*this, prefix + "_camera_matrix"), 3, 3);
  const auto distortion = parameterVector(*this, prefix + "_distortion_coefficients");
  if (!distortion.empty()) {
    calibrations_[index].distortion_coefficients = cv::Mat(
      static_cast<int>(distortion.size()), 1, CV_64F);
    for (std::size_t i = 0; i < distortion.size(); ++i) {
      calibrations_[index].distortion_coefficients.at<double>(static_cast<int>(i), 0) = distortion[i];
    }
  }
}

void TrackerNode::initializeCameraInputs()
{
  front_0_input_.topic = get_parameter("front_0_armor_pose_set_topic").as_string();
  front_1_input_.topic = get_parameter("front_1_armor_pose_set_topic").as_string();
  back_input_.topic = get_parameter("back_armor_pose_set_topic").as_string();
  const auto subscribe = [this](CameraInput & input, CameraSource source) {
      input.subscription = create_subscription<ArmorPoseSetArray>(
        input.topic, highRateQos(),
        [this, source](const ArmorPoseSetArray::ConstSharedPtr message) {
          onArmorPoseSets(source, message);
        });
    };
  subscribe(front_0_input_, CameraSource::Front0);
  subscribe(front_1_input_, CameraSource::Front1);
  subscribe(back_input_, CameraSource::Back);
}

std::size_t TrackerNode::cameraIndex(CameraSource source)
{
  switch (source) {
    case CameraSource::Front0: return 0;
    case CameraSource::Front1: return 1;
    case CameraSource::Back: return 2;
  }
  return 0;
}

bool TrackerNode::isOlder(
  const builtin_interfaces::msg::Time & lhs, const builtin_interfaces::msg::Time & rhs)
{
  return lhs.sec < rhs.sec || (lhs.sec == rhs.sec && lhs.nanosec < rhs.nanosec);
}

double TrackerNode::secondsBetween(
  const builtin_interfaces::msg::Time & lhs, const builtin_interfaces::msg::Time & rhs)
{
  return static_cast<double>(rhs.sec - lhs.sec) +
    1e-9 * static_cast<double>(rhs.nanosec) -
    1e-9 * static_cast<double>(lhs.nanosec);
}

bool TrackerNode::hasNormalObservation(const ArmorPoseSetArray & message, std::uint8_t outpost_id)
{
  return std::any_of(
    message.armor_pose_sets.begin(), message.armor_pose_sets.end(),
    [outpost_id](const auto & set) {
      return set.id != outpost_id && !set.observations.empty();
    });
}

void TrackerNode::onArmorPoseSets(
  CameraSource source, const ArmorPoseSetArray::ConstSharedPtr message)
{
  processArmorPoseSets(source, message);
}

bool TrackerNode::hasRecentFrontTarget(const builtin_interfaces::msg::Time & stamp) const
{
  return std::any_of(
    latest_front_target_stamps_.begin(), latest_front_target_stamps_.end(),
    [this, &stamp](const auto & latest) {
      return latest.has_value() &&
             std::abs(secondsBetween(*latest, stamp)) <= front_target_hold_sec_;
    });
}

TargetTracker & TrackerNode::trackerFor(std::uint8_t target_id)
{
  auto result = trackers_.try_emplace(target_id, config_, calibrations_);
  return result.first->second;
}

void TrackerNode::configureTrackerCalibration(
  TargetTracker & target, CameraSource source)
{
  const std::size_t index = cameraIndex(source);
  target.setCameraCalibration(source, calibrations_[index]);
}

void TrackerNode::processArmorPoseSets(
  CameraSource source, const ArmorPoseSetArray::ConstSharedPtr message)
{
  if (!message) {
    return;
  }
  const auto stamp = message->header.stamp;
  if (last_processed_stamp_.has_value() && isOlder(stamp, *last_processed_stamp_)) {
    return;
  }

  if (source == CameraSource::Front0 || source == CameraSource::Front1) {
    if (hasNormalObservation(*message, outpost_id_)) {
      latest_front_target_stamps_[source == CameraSource::Front0 ? 0U : 1U] = stamp;
    }
  } else if (hasRecentFrontTarget(stamp)) {
    return;
  }
  last_processed_stamp_ = stamp;

  for (auto & entry : trackers_) {
    entry.second.predict(stamp);
  }

  std::map<std::uint8_t, std::vector<ArmorObservation>> observations;
  for (const auto & pose_set : message->armor_pose_sets) {
    if (pose_set.id == outpost_id_) {
      continue;
    }
    for (const auto & item : pose_set.observations) {
      observations[pose_set.id].push_back(
        fromRosObservation(pose_set.id, source, stamp, item));
    }
  }

  std::vector<geometry_msgs::msg::Pose> selected_poses;
  for (auto & entry : observations) {
    auto existing = trackers_.find(entry.first);
    if (existing != trackers_.end() && existing->second.diverged()) {
      trackers_.erase(existing);
    }
    const auto confirmation = last_confirmation_stamps_.find(entry.first);
    if (confirmation == last_confirmation_stamps_.end() ||
      secondsBetween(confirmation->second, stamp) >= confirmation_min_interval_sec_)
    {
      ++consecutive_detection_counts_[entry.first];
      last_confirmation_stamps_[entry.first] = stamp;
    }
    if (consecutive_detection_counts_[entry.first] < min_consecutive_detections_to_track_) {
      continue;
    }
    auto & target = trackerFor(entry.first);
    bool update_succeeded = true;
    for (const auto & observation : entry.second) {
      configureTrackerCalibration(target, source);
      bool accepted = false;
      if (!target.active()) {
        accepted = target.initialize(observation);
      } else {
        accepted = target.addMeasurement(observation);
      }
      update_succeeded = update_succeeded && accepted;
      if (accepted) {
        selected_poses.push_back(observation.world_pose);
      }
    }
    update_succeeded = target.optimize() && update_succeeded;
    if (!update_succeeded || target.diverged()) {
      trackers_.erase(entry.first);
      last_update_stamps_.erase(entry.first);
      last_confirmation_stamps_.erase(entry.first);
      consecutive_detection_counts_.erase(entry.first);
      continue;
    }
    last_update_stamps_[entry.first] = stamp;
  }

  removeExpiredTrackers(stamp);
  publishStates(message->header, selected_poses);
}

void TrackerNode::removeExpiredTrackers(const builtin_interfaces::msg::Time & stamp)
{
  for (auto iterator = trackers_.begin(); iterator != trackers_.end();) {
    const auto update = last_update_stamps_.find(iterator->first);
    const bool recent = update != last_update_stamps_.end() &&
      secondsBetween(update->second, stamp) <= target_lost_timeout_sec_;
    if (recent) {
      ++iterator;
      continue;
    }
    consecutive_detection_counts_.erase(iterator->first);
    last_confirmation_stamps_.erase(iterator->first);
    last_update_stamps_.erase(iterator->first);
    iterator = trackers_.erase(iterator);
  }
}

void TrackerNode::publishStates(
  const std_msgs::msg::Header & header,
  const std::vector<geometry_msgs::msg::Pose> & selected_poses)
{
  aim_msgs::msg::TargetStateArray output;
  output.header = header;
  for (const auto & entry : trackers_) {
    if (entry.first == outpost_id_ || !entry.second.active() || entry.second.diverged()) {
      continue;
    }
    output.targets.push_back(entry.second.toMessage(header));
  }
  target_state_pub_->publish(std::move(output));
  recordPublishDelay(header.stamp);
  if (enable_visualization_ && visualization_pub_) {
    publishVisualization(header, selected_poses);
  }
}

void TrackerNode::recordPublishDelay(const builtin_interfaces::msg::Time & upstream_stamp)
{
  if (upstream_stamp.sec == 0 && upstream_stamp.nanosec == 0) {
    return;
  }
  const double delay_ms = (now() - rclcpp::Time(upstream_stamp)).seconds() * 1000.0;
  delay_statistics_sum_ms_ += delay_ms;
  delay_statistics_max_ms_ = std::max(delay_statistics_max_ms_, delay_ms);
  delay_statistics_latest_ms_ = delay_ms;
  ++delay_statistics_count_;
  const auto current = std::chrono::steady_clock::now();
  if (current - last_delay_statistics_publish_time_ < std::chrono::seconds(1)) {
    return;
  }
  aim_msgs::msg::DelayStatistics statistics;
  statistics.header.stamp = now();
  statistics.latest_delay_ms = delay_statistics_latest_ms_;
  statistics.average_delay_ms = delay_statistics_sum_ms_ /
    static_cast<double>(delay_statistics_count_);
  statistics.max_delay_ms = delay_statistics_max_ms_;
  statistics.sample_count = delay_statistics_count_;
  delay_statistics_pub_->publish(std::move(statistics));
  last_delay_statistics_publish_time_ = current;
  delay_statistics_sum_ms_ = 0.0;
  delay_statistics_max_ms_ = 0.0;
  delay_statistics_latest_ms_ = 0.0;
  delay_statistics_count_ = 0;
}

void TrackerNode::publishVisualization(
  const std_msgs::msg::Header & header,
  const std::vector<geometry_msgs::msg::Pose> & selected_poses)
{
  visualization_msgs::msg::MarkerArray markers;
  int marker_id = 0;
  for (const auto & entry : trackers_) {
    if (!entry.second.active() || entry.second.diverged()) {
      continue;
    }
    for (const auto & pose : entry.second.toMessage(header).predicted_armors) {
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.ns = "tracker/predicted_armors";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = pose;
      marker.scale.x = 0.01;
      marker.scale.y = entry.second.state()[8] * 0.6;
      marker.scale.z = 0.056;
      marker.color.r = 0.2F;
      marker.color.g = 0.8F;
      marker.color.b = 1.0F;
      marker.color.a = 0.85F;
      marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      markers.markers.push_back(std::move(marker));
    }
  }
  for (const auto & pose : selected_poses) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "tracker/selected_measurements";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = 0.018;
    marker.scale.y = 0.135;
    marker.scale.z = 0.056;
    marker.color.r = 1.0F;
    marker.color.g = 0.9F;
    marker.color.b = 0.1F;
    marker.color.a = 0.95F;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    markers.markers.push_back(std::move(marker));
  }
  for (int stale = marker_id; stale < last_visualization_marker_count_; ++stale) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "tracker/stale";
    marker.id = stale;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(std::move(marker));
  }
  last_visualization_marker_count_ = marker_id;
  visualization_pub_->publish(std::move(markers));
}

}  // namespace tracker
