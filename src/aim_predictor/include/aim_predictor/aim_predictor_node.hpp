#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "aim_predictor/normal_target_tracker.hpp"
#include "aim_msgs/msg/armor_pose_set_array.hpp"
#include "aim_msgs/msg/delay_statistics.hpp"
#include "aim_msgs/msg/target_state_array.hpp"

namespace aim_predictor
{

class AimPredictorNode : public rclcpp::Node
{
public:
  explicit AimPredictorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Pipeline
  {
    std::string name;
    std::string target_state_topic;
    rclcpp::Publisher<aim_msgs::msg::TargetStateArray>::SharedPtr target_state_pub;
    std::map<std::uint8_t, NormalTargetTracker> trackers;
    std::map<std::uint8_t, int> consecutive_detection_counts;
    std::map<std::uint8_t, rclcpp::Time> last_confirmation_stamps;
    std::map<std::uint8_t, rclcpp::Time> last_successful_update_stamps;
    std::vector<geometry_msgs::msg::Pose> selected_armor_poses;
    std::optional<rclcpp::Time> last_processed_stamp;
    int last_marker_count{0};
    int last_selected_marker_count{0};
  };

  struct CameraInput
  {
    std::string armor_pose_set_topic;
    rclcpp::Subscription<aim_msgs::msg::ArmorPoseSetArray>::SharedPtr armor_pose_sub;
  };

  void onArmorPoseSets(
    Pipeline & pipeline,
    const aim_msgs::msg::ArmorPoseSetArray::ConstSharedPtr msg,
    CameraSource source);
  void onCameraArmorPoseSets(
    CameraSource source, const aim_msgs::msg::ArmorPoseSetArray::ConstSharedPtr msg);
  bool hasRecentFrontTarget(const rclcpp::Time & stamp) const;
  const MeasurementNoiseConfig & measurementNoiseConfigFor(CameraSource source) const;
  void publishVisualization(Pipeline & pipeline, const std_msgs::msg::Header & header);
  std::optional<double> lookupGimbalYaw(const std_msgs::msg::Header & header);
  void recordPublishDelay(const builtin_interfaces::msg::Time & upstream_stamp);

  rclcpp::Publisher<aim_msgs::msg::DelayStatistics>::SharedPtr delay_statistics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  Pipeline fused_pipeline_;
  CameraInput front_0_input_;
  CameraInput front_1_input_;
  CameraInput back_input_;
  std::array<std::optional<rclcpp::Time>, 2> latest_front_target_stamps_{};
  std::string visualization_topic_;
  std::string world_frame_id_;
  std::string gimbal_frame_id_;
  double init_radius_;
  bool enable_visualization_{false};
  double tf_lookup_timeout_sec_{0.02};
  ProcessNoiseConfig process_noise_config_;
  MeasurementNoiseConfig front_0_measurement_noise_config_;
  MeasurementNoiseConfig front_1_measurement_noise_config_;
  MeasurementNoiseConfig back_measurement_noise_config_;
  double multi_armor_yaw_gate_rad_{60.0 * 3.14159265358979323846 / 180.0};
  int min_consecutive_detections_to_track_{1};
  std::uint8_t outpost_id_{6};
  double confirmation_min_interval_sec_{0.03};
  double front_target_hold_sec_{0.10};
  double target_lost_timeout_sec_{0.20};
  std::chrono::steady_clock::time_point last_delay_statistics_publish_time_{};
  double delay_statistics_sum_ms_{0.0};
  double delay_statistics_max_ms_{0.0};
  double delay_statistics_latest_ms_{0.0};
  std::uint64_t delay_statistics_count_{0};
};

}  // namespace aim_predictor
