#include "aim_predictor/aim_predictor_node.hpp"
#include "aim_predictor/aim_predictor_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.hpp>
#include <tf2_ros/qos.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace aim_predictor
{

namespace
{

constexpr double kNormalTargetRadius = 0.2;

rclcpp::QoS makeHighRateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

rclcpp::QoS makeRealtimeTfQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

}  // namespace

AimPredictorNode::AimPredictorNode(const rclcpp::NodeOptions & options)
: Node("aim_predictor_node", options)
{
  declare_parameter<std::string>("front_0_armor_pose_set_topic", "/aim_solver/front_0/armor_pose_sets");
  declare_parameter<std::string>("front_0_target_state_topic", "/aim_predictor/front_0/target_states");
  declare_parameter<std::string>("front_1_armor_pose_set_topic", "/aim_solver/front_1/armor_pose_sets");
  declare_parameter<std::string>("front_1_target_state_topic", "/aim_predictor/front_1/target_states");
  declare_parameter<std::string>("back_armor_pose_set_topic", "/aim_solver/back/armor_pose_sets");
  declare_parameter<std::string>("back_target_state_topic", "/aim_predictor/back/target_states");
  declare_parameter<std::string>("delay_statistics_topic", "/aim_predictor/delay_statistics");
  declare_parameter<bool>("enable_visualization", false);
  declare_parameter<std::string>("visualization_topic", "/aim_predictor/visualization");
  declare_parameter<std::string>("world_frame_id", "world");
  declare_parameter<std::string>("gimbal_frame_id", "gimbal_link");
  declare_parameter<double>("tf_lookup_timeout_sec", 0.02);
  declare_parameter<double>("init_radius", kNormalTargetRadius);
  declare_parameter<int>("max_lost_count", 10);
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
  declare_parameter<double>("high_speed_angular_velocity_threshold", 6.0);
  declare_parameter<double>("multi_armor_yaw_gate_deg", 60.0);
  declare_parameter<int>("min_consecutive_detections_to_track", 1);
  declare_parameter<int>("outpost_id", 6);

  front_0_pipeline_.name = "front_0";
  front_0_pipeline_.armor_pose_set_topic =
    get_parameter("front_0_armor_pose_set_topic").as_string();
  front_0_pipeline_.target_state_topic =
    get_parameter("front_0_target_state_topic").as_string();
  front_1_pipeline_.name = "front_1";
  front_1_pipeline_.armor_pose_set_topic =
    get_parameter("front_1_armor_pose_set_topic").as_string();
  front_1_pipeline_.target_state_topic =
    get_parameter("front_1_target_state_topic").as_string();
  back_pipeline_.name = "back";
  back_pipeline_.armor_pose_set_topic =
    get_parameter("back_armor_pose_set_topic").as_string();
  back_pipeline_.target_state_topic =
    get_parameter("back_target_state_topic").as_string();
  const auto delay_statistics_topic = get_parameter("delay_statistics_topic").as_string();
  enable_visualization_ = get_parameter("enable_visualization").as_bool();
  visualization_topic_ = get_parameter("visualization_topic").as_string();
  world_frame_id_ = get_parameter("world_frame_id").as_string();
  gimbal_frame_id_ = get_parameter("gimbal_frame_id").as_string();
  tf_lookup_timeout_sec_ = get_parameter("tf_lookup_timeout_sec").as_double();
  init_radius_ = get_parameter("init_radius").as_double();
  max_lost_count_ = get_parameter("max_lost_count").as_int();
  process_noise_config_.low_speed_process_noise_xy =
    get_parameter("low_speed_process_noise_xy").as_double();
  process_noise_config_.low_speed_process_noise_z =
    get_parameter("low_speed_process_noise_z").as_double();
  process_noise_config_.low_speed_process_noise_yaw =
    get_parameter("low_speed_process_noise_yaw").as_double();
  process_noise_config_.middle_speed_process_noise_xy =
    get_parameter("middle_speed_process_noise_xy").as_double();
  process_noise_config_.middle_speed_process_noise_z =
    get_parameter("middle_speed_process_noise_z").as_double();
  process_noise_config_.middle_speed_process_noise_yaw =
    get_parameter("middle_speed_process_noise_yaw").as_double();
  process_noise_config_.high_speed_process_noise_xy =
    get_parameter("high_speed_process_noise_xy").as_double();
  process_noise_config_.high_speed_process_noise_z =
    get_parameter("high_speed_process_noise_z").as_double();
  process_noise_config_.high_speed_process_noise_yaw =
    get_parameter("high_speed_process_noise_yaw").as_double();
  process_noise_config_.middle_speed_angular_velocity_threshold =
    get_parameter("middle_speed_angular_velocity_threshold").as_double();
  process_noise_config_.high_speed_angular_velocity_threshold =
    get_parameter("high_speed_angular_velocity_threshold").as_double();
  multi_armor_yaw_gate_rad_ =
    get_parameter("multi_armor_yaw_gate_deg").as_double() * kPi / 180.0;
  min_consecutive_detections_to_track_ =
    std::max(1, static_cast<int>(get_parameter("min_consecutive_detections_to_track").as_int()));
  outpost_id_ = static_cast<std::uint8_t>(
    std::clamp(get_parameter("outpost_id").as_int(), 0L, 255L));

  const auto high_rate_qos = makeHighRateQos();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_, this, true, makeRealtimeTfQos(), tf2_ros::StaticListenerQoS());
  last_delay_statistics_publish_time_ = std::chrono::steady_clock::now();
  delay_statistics_pub_ =
    create_publisher<aim_msgs::msg::DelayStatistics>(delay_statistics_topic, high_rate_qos);
  if (enable_visualization_) {
    visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      visualization_topic_, rclcpp::SensorDataQoS());
  }
  auto initialize_pipeline = [this, &high_rate_qos](Pipeline & pipeline) {
      pipeline.target_state_pub = create_publisher<aim_msgs::msg::TargetStateArray>(
        pipeline.target_state_topic, high_rate_qos);
      pipeline.armor_pose_sub = create_subscription<aim_msgs::msg::ArmorPoseSetArray>(
        pipeline.armor_pose_set_topic,
        high_rate_qos,
        [this, &pipeline](const aim_msgs::msg::ArmorPoseSetArray::ConstSharedPtr msg) {
          onArmorPoseSets(pipeline, msg);
        });
    };

  initialize_pipeline(front_0_pipeline_);
  initialize_pipeline(front_1_pipeline_);
  initialize_pipeline(back_pipeline_);
}

void AimPredictorNode::onArmorPoseSets(
  Pipeline & pipeline, const aim_msgs::msg::ArmorPoseSetArray::ConstSharedPtr msg)
{
  std::map<std::uint8_t, std::vector<ArmorMeasurement>> grouped_measurements;

  for (const auto & armor_pose_set : msg->armor_pose_sets) {
    if (armor_pose_set.id == outpost_id_) {
      continue;
    }
    for (const auto & pose : armor_pose_set.armor_poses) {
      ArmorMeasurement measurement;
      measurement.id = armor_pose_set.id;
      measurement.pose = pose;
      measurement.xyz = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
      measurement.ypr = poseToYpr(pose);
      measurement.ypd = xyzToYpd(measurement.xyz);
      grouped_measurements[armor_pose_set.id].push_back(std::move(measurement));
    }
  }

  const rclcpp::Time stamp(msg->header.stamp);
  const std::optional<double> gimbal_yaw = lookupGimbalYaw(msg->header);
  pipeline.selected_armor_poses.clear();
  pipeline.trackers.erase(outpost_id_);
  pipeline.consecutive_detection_counts.erase(outpost_id_);

  for (auto it = pipeline.consecutive_detection_counts.begin();
       it != pipeline.consecutive_detection_counts.end();) {
    if (grouped_measurements.count(it->first) == 0) {
      it = pipeline.consecutive_detection_counts.erase(it);
    } else {
      ++it;
    }
  }

  for (auto & tracker_entry : pipeline.trackers) {
    auto & tracker = tracker_entry.second;
    if (tracker.active()) {
      tracker.predict(stamp);
      ++tracker.lost_count;
    }
  }

  for (auto & [target_id, measurements] : grouped_measurements) {
    if (measurements.empty()) {
      pipeline.consecutive_detection_counts.erase(target_id);
      continue;
    }

    struct UpdateMeasurement
    {
      const ArmorMeasurement * measurement;
      double yaw_error;
    };

    const auto calc_yaw_error = [&gimbal_yaw](const ArmorMeasurement & measurement) {
        return gimbal_yaw.has_value() ?
          std::abs(limitRad(measurement.ypr.x() - *gimbal_yaw)) :
          std::abs(measurement.ypd.x());
      };

    std::vector<UpdateMeasurement> update_measurements;
    update_measurements.reserve(measurements.size());
    for (const auto & measurement : measurements) {
      const double yaw_error = calc_yaw_error(measurement);
      if (measurements.size() > 1U && yaw_error > multi_armor_yaw_gate_rad_) {
        continue;
      }
      update_measurements.push_back(UpdateMeasurement{&measurement, yaw_error});
    }

    if (update_measurements.empty()) {
      pipeline.consecutive_detection_counts.erase(target_id);
      continue;
    }

    std::sort(
      update_measurements.begin(), update_measurements.end(),
      [](const UpdateMeasurement & lhs, const UpdateMeasurement & rhs) {
        return lhs.yaw_error < rhs.yaw_error;
      });

    const int consecutive_detection_count = ++pipeline.consecutive_detection_counts[target_id];

    auto tracker_it = pipeline.trackers.find(target_id);
    const bool needs_initialize =
      tracker_it == pipeline.trackers.end() ||
      !tracker_it->second.active() ||
      tracker_it->second.lost_count > max_lost_count_ ||
      tracker_it->second.diverged();

    if (needs_initialize) {
      if (consecutive_detection_count < min_consecutive_detections_to_track_) {
        continue;
      }
      auto & tracker = pipeline.trackers[target_id];
      tracker.setProcessNoiseConfig(process_noise_config_);
      pipeline.selected_armor_poses.push_back(update_measurements.front().measurement->pose);
      tracker.initialize(*update_measurements.front().measurement, stamp, init_radius_);
      for (std::size_t i = 1; i < update_measurements.size(); ++i) {
        pipeline.selected_armor_poses.push_back(update_measurements[i].measurement->pose);
        tracker.update(*update_measurements[i].measurement);
      }
    } else {
      auto & tracker = tracker_it->second;
      tracker.setProcessNoiseConfig(process_noise_config_);
      for (const auto & update_measurement : update_measurements) {
        pipeline.selected_armor_poses.push_back(update_measurement.measurement->pose);
        tracker.update(*update_measurement.measurement);
      }
    }
  }

  for (auto it = pipeline.trackers.begin(); it != pipeline.trackers.end();) {
    if (!it->second.active() || it->second.lost_count <= max_lost_count_) {
      ++it;
      continue;
    }
    RCLCPP_DEBUG(
      get_logger(),
      "[%s] lost target: id=%u lost_count=%d max_lost_count=%d",
      pipeline.name.c_str(),
      static_cast<unsigned int>(it->first),
      it->second.lost_count,
      max_lost_count_);
    it = pipeline.trackers.erase(it);
  }

  aim_msgs::msg::TargetStateArray output;
  output.header = msg->header;
  for (const auto & [target_id, tracker] : pipeline.trackers) {
    if (target_id == outpost_id_ || !tracker.active() || tracker.diverged()) {
      continue;
    }

    aim_msgs::msg::TargetState state_msg;
    state_msg.header = msg->header;
    state_msg.id = target_id;
    state_msg.tracking = true;
    state_msg.converged = tracker.converged();
    state_msg.jumped = tracker.jumped();

    const Eigen::VectorXd & x = tracker.state();
    state_msg.center.x = x[0];
    state_msg.center.y = x[2];
    state_msg.center.z = x[4];
    state_msg.velocity.x = x[1];
    state_msg.velocity.y = x[3];
    state_msg.velocity.z = x[5];
    state_msg.yaw = x[6];
    state_msg.angular_velocity = x[7];
    state_msg.radius = x[8];
    state_msg.radius_offset = x[9];
    state_msg.height_offset = x[10];
    state_msg.predicted_armors = tracker.predictedArmors();
    output.targets.push_back(std::move(state_msg));
  }

  pipeline.target_state_pub->publish(std::move(output));
  recordPublishDelay(msg->header.stamp);
  if (enable_visualization_ && visualization_pub_) {
    publishVisualization(pipeline, msg->header);
  }
}

void AimPredictorNode::recordPublishDelay(const builtin_interfaces::msg::Time & upstream_stamp)
{
  if ((upstream_stamp.sec == 0) && (upstream_stamp.nanosec == 0)) {
    return;
  }

  const rclcpp::Time publish_time = now();
  const double delay_ms = (publish_time - rclcpp::Time(upstream_stamp)).seconds() * 1000.0;
  delay_statistics_sum_ms_ += delay_ms;
  delay_statistics_max_ms_ = std::max(delay_statistics_max_ms_, delay_ms);
  delay_statistics_latest_ms_ = delay_ms;
  ++delay_statistics_count_;

  const auto current_time = std::chrono::steady_clock::now();
  if (current_time - last_delay_statistics_publish_time_ < std::chrono::seconds(1)) {
    return;
  }

  aim_msgs::msg::DelayStatistics stats_msg;
  stats_msg.header.stamp = publish_time;
  stats_msg.latest_delay_ms = delay_statistics_latest_ms_;
  stats_msg.average_delay_ms =
    delay_statistics_sum_ms_ / static_cast<double>(delay_statistics_count_);
  stats_msg.max_delay_ms = delay_statistics_max_ms_;
  stats_msg.sample_count = delay_statistics_count_;
  delay_statistics_pub_->publish(std::move(stats_msg));

  last_delay_statistics_publish_time_ = current_time;
  delay_statistics_sum_ms_ = 0.0;
  delay_statistics_max_ms_ = 0.0;
  delay_statistics_latest_ms_ = 0.0;
  delay_statistics_count_ = 0;
}

void AimPredictorNode::publishVisualization(Pipeline & pipeline, const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;

  for (const auto & tracker_entry : pipeline.trackers) {
    const auto & tracker = tracker_entry.second;
    if (!tracker.active() || tracker.diverged()) {
      continue;
    }

    const auto predicted_armors = tracker.predictedArmors();
    for (std::size_t armor_index = 0; armor_index < predicted_armors.size(); ++armor_index) {
      const auto & pose = predicted_armors[armor_index];

      visualization_msgs::msg::Marker cube_marker;
      cube_marker.header = header;
      cube_marker.ns = pipeline.name + "/predicted_armor_pose";
      cube_marker.id = marker_id++;
      cube_marker.type = visualization_msgs::msg::Marker::CUBE;
      cube_marker.action = visualization_msgs::msg::Marker::ADD;
      cube_marker.pose = pose;
      cube_marker.scale.x = 0.01;
      cube_marker.scale.y = 0.135;
      cube_marker.scale.z = 0.056;
      cube_marker.color.r = 0.2F;
      cube_marker.color.g = 0.8F;
      cube_marker.color.b = 1.0F;
      cube_marker.color.a = 0.85F;
      cube_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      marker_array.markers.push_back(std::move(cube_marker));

      visualization_msgs::msg::Marker axis_marker;
      axis_marker.header = header;
      axis_marker.ns = pipeline.name + "/predicted_armor_axis";
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
      marker_array.markers.push_back(std::move(axis_marker));
    }
  }

  int selected_marker_id = 0;
  for (const auto & pose : pipeline.selected_armor_poses) {
    visualization_msgs::msg::Marker selected_marker;
    selected_marker.header = header;
    selected_marker.ns = pipeline.name + "/selected_armor_measurement";
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
    marker_array.markers.push_back(std::move(selected_marker));

    visualization_msgs::msg::Marker selected_axis_marker;
    selected_axis_marker.header = header;
    selected_axis_marker.ns = pipeline.name + "/selected_armor_measurement_axis";
    selected_axis_marker.id = selected_marker_id++;
    selected_axis_marker.type = visualization_msgs::msg::Marker::ARROW;
    selected_axis_marker.action = visualization_msgs::msg::Marker::ADD;
    selected_axis_marker.scale.x = 0.014;
    selected_axis_marker.scale.y = 0.03;
    selected_axis_marker.scale.z = 0.04;
    selected_axis_marker.color.r = 1.0F;
    selected_axis_marker.color.g = 0.45F;
    selected_axis_marker.color.b = 0.0F;
    selected_axis_marker.color.a = 1.0F;
    selected_axis_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    selected_axis_marker.points.push_back(pose.position);
    selected_axis_marker.points.push_back(pointAlongPoseXAxis(pose, 0.14));
    marker_array.markers.push_back(std::move(selected_axis_marker));
  }

  for (int stale_id = selected_marker_id; stale_id < pipeline.last_selected_marker_count; ++stale_id) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns =
      (stale_id % 2 == 0) ?
      pipeline.name + "/selected_armor_measurement" :
      pipeline.name + "/selected_armor_measurement_axis";
    marker.id = stale_id;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(std::move(marker));
  }

  for (int stale_id = marker_id; stale_id < pipeline.last_marker_count; ++stale_id) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = (stale_id % 2 == 0) ?
      pipeline.name + "/predicted_armor_pose" :
      pipeline.name + "/predicted_armor_axis";
    marker.id = stale_id;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(std::move(marker));
  }

  pipeline.last_marker_count = marker_id;
  pipeline.last_selected_marker_count = selected_marker_id;
  visualization_pub_->publish(std::move(marker_array));
}

std::optional<double> AimPredictorNode::lookupGimbalYaw(const std_msgs::msg::Header & header)
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
      "failed to lookup gimbal yaw for armor selection: target='%s' source='%s': %s",
      source_frame.c_str(), gimbal_frame_id_.c_str(), ex.what());
    return std::nullopt;
  }
}

}  // namespace aim_predictor
