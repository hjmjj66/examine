#pragma once

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
    std::string armor_pose_set_topic;
    std::string target_state_topic;
    rclcpp::Subscription<aim_msgs::msg::ArmorPoseSetArray>::SharedPtr armor_pose_sub;
    rclcpp::Publisher<aim_msgs::msg::TargetStateArray>::SharedPtr target_state_pub;
    std::map<std::uint8_t, NormalTargetTracker> trackers;
    std::map<std::uint8_t, int> consecutive_detection_counts;
    std::vector<geometry_msgs::msg::Pose> selected_armor_poses;
    int last_marker_count{0};
    int last_selected_marker_count{0};
  };

  void onArmorPoseSets(Pipeline & pipeline, const aim_msgs::msg::ArmorPoseSetArray::ConstSharedPtr msg);
  void publishVisualization(Pipeline & pipeline, const std_msgs::msg::Header & header);
  std::optional<double> lookupGimbalYaw(const std_msgs::msg::Header & header);
  void recordPublishDelay(const builtin_interfaces::msg::Time & upstream_stamp);

  rclcpp::Publisher<aim_msgs::msg::DelayStatistics>::SharedPtr delay_statistics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  Pipeline front_0_pipeline_;
  Pipeline front_1_pipeline_;
  Pipeline back_pipeline_;
  std::string visualization_topic_;
  std::string world_frame_id_;
  std::string gimbal_frame_id_;
  double init_radius_;
  int max_lost_count_;
  bool enable_visualization_{false};
  double tf_lookup_timeout_sec_{0.02};
  ProcessNoiseConfig process_noise_config_;
  double multi_armor_yaw_gate_rad_{60.0 * 3.14159265358979323846 / 180.0};
  int min_consecutive_detections_to_track_{1};
  std::uint8_t outpost_id_{6};
  std::chrono::steady_clock::time_point last_delay_statistics_publish_time_{};
  double delay_statistics_sum_ms_{0.0};
  double delay_statistics_max_ms_{0.0};
  double delay_statistics_latest_ms_{0.0};
  std::uint64_t delay_statistics_count_{0};
};

}  // namespace aim_predictor
