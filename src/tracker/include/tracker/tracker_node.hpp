#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "aim_msgs/msg/armor_pose_set_array.hpp"
#include "aim_msgs/msg/delay_statistics.hpp"
#include "aim_msgs/msg/target_state_array.hpp"
#include "tracker/target_tracker.hpp"

namespace tracker
{

class TrackerNode : public rclcpp::Node
{
public:
  explicit TrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using ArmorPoseSetArray = aim_msgs::msg::ArmorPoseSetArray;

  struct CameraInput
  {
    std::string topic;
    rclcpp::Subscription<ArmorPoseSetArray>::SharedPtr subscription;
  };

  static std::size_t cameraIndex(CameraSource source);
  static bool isOlder(
    const builtin_interfaces::msg::Time & lhs,
    const builtin_interfaces::msg::Time & rhs);
  static double secondsBetween(
    const builtin_interfaces::msg::Time & lhs,
    const builtin_interfaces::msg::Time & rhs);
  static bool hasNormalObservation(const ArmorPoseSetArray & message, std::uint8_t outpost_id);

  void onArmorPoseSets(
    CameraSource source,
    const ArmorPoseSetArray::ConstSharedPtr message);
  void processArmorPoseSets(
    CameraSource source,
    const ArmorPoseSetArray::ConstSharedPtr message);
  bool hasRecentFrontTarget(const builtin_interfaces::msg::Time & stamp) const;
  void publishStates(
    const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Pose> & selected_poses);
  void publishVisualization(
    const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Pose> & selected_poses);
  void recordPublishDelay(const builtin_interfaces::msg::Time & upstream_stamp);
  void declareAndLoadParameters();
  void initializeCameraInputs();
  void loadCalibration(CameraSource source, const std::string & prefix);
  void loadTrackerConfig();
  void configureTrackerCalibration(TargetTracker & target, CameraSource source);
  TargetTracker & trackerFor(std::uint8_t target_id);
  void removeExpiredTrackers(const builtin_interfaces::msg::Time & stamp);

  TrackerConfig config_{};
  std::array<CameraCalibration, 3> calibrations_{};
  std::map<std::uint8_t, TargetTracker> trackers_;
  std::map<std::uint8_t, builtin_interfaces::msg::Time> last_update_stamps_;
  std::map<std::uint8_t, builtin_interfaces::msg::Time> last_confirmation_stamps_;
  std::map<std::uint8_t, int> consecutive_detection_counts_;
  std::optional<builtin_interfaces::msg::Time> last_processed_stamp_;
  std::array<std::optional<builtin_interfaces::msg::Time>, 2> latest_front_target_stamps_{};

  CameraInput front_0_input_;
  CameraInput front_1_input_;
  CameraInput back_input_;
  rclcpp::Publisher<aim_msgs::msg::TargetStateArray>::SharedPtr target_state_pub_;
  rclcpp::Publisher<aim_msgs::msg::DelayStatistics>::SharedPtr delay_statistics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;

  std::string fused_target_state_topic_;
  std::string delay_statistics_topic_;
  std::string visualization_topic_;
  std::string world_frame_id_;
  bool enable_visualization_{false};
  std::uint8_t outpost_id_{6};
  int min_consecutive_detections_to_track_{1};
  double confirmation_min_interval_sec_{0.03};
  double front_target_hold_sec_{0.10};
  double target_lost_timeout_sec_{0.20};

  std::chrono::steady_clock::time_point last_delay_statistics_publish_time_{};
  double delay_statistics_sum_ms_{0.0};
  double delay_statistics_max_ms_{0.0};
  double delay_statistics_latest_ms_{0.0};
  std::uint64_t delay_statistics_count_{0};
  int last_visualization_marker_count_{0};
  int last_selected_marker_count_{0};
};

}  // namespace tracker
