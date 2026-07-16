#pragma once

#include <Eigen/Dense>

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "aim_msgs/msg/armor_pose_set_array.hpp"
#include "aim_msgs/msg/outpost_state.hpp"
#include "aim_outpost_predictor/front_camera_arbitrator.hpp"

namespace aim_outpost_predictor
{

class AimOutpostPredictorNode : public rclcpp::Node
{
public:
  explicit AimOutpostPredictorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  class ExtendedKalmanFilter
  {
  public:
    ExtendedKalmanFilter() = default;
    ExtendedKalmanFilter(
      const Eigen::VectorXd & x0,
      const Eigen::MatrixXd & p0,
      std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add);

    Eigen::VectorXd predict(
      const Eigen::MatrixXd & f,
      const Eigen::MatrixXd & q,
      std::function<Eigen::VectorXd(const Eigen::VectorXd &)> transition);

    Eigen::VectorXd update(
      const Eigen::VectorXd & z,
      const Eigen::MatrixXd & h,
      const Eigen::MatrixXd & r,
      std::function<Eigen::VectorXd(const Eigen::VectorXd &)> observation,
      std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract);

    Eigen::VectorXd x;
    Eigen::MatrixXd p;
    std::deque<int> recent_nis_failures{0};
    std::size_t window_size{100};

  private:
    Eigen::MatrixXd identity_;
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add_;
  };

  struct ArmorMeasurement
  {
    std::uint8_t id{0};
    geometry_msgs::msg::Pose pose;
    Eigen::Vector3d xyz{Eigen::Vector3d::Zero()};
    Eigen::Vector3d ypr{Eigen::Vector3d::Zero()};
    Eigen::Vector3d ypd{Eigen::Vector3d::Zero()};
  };

  struct TrackerConfig
  {
    double init_radius{0.2765};
    double initial_low_height_offset{-0.10};
    double initial_high_height_offset{0.10};
    double process_noise_xy{10.0};
    double process_noise_z{10.0};
    double process_noise_yaw{100.0};
    double angular_velocity_clamp{2.51};
    int converged_min_updates{10};
    double match_yaw_sigma{0.20};
    double match_pitch_sigma{0.20};
    double match_distance_sigma{0.35};
    double match_angle_sigma{0.35};
    double match_z_sigma{0.08};
    double match_yaw_gate{1.0};
    double match_pitch_gate{0.9};
    double match_distance_gate{2.0};
    double match_angle_gate{1.0};
    double match_z_gate{0.35};
    double reverse_direction_penalty{2.5};
    double max_pair_cost{18.0};
    double fast_reanchor_cost{8.0};
    int fast_reanchor_frames{2};
  };

  class OutpostTracker
  {
  public:
    void setConfig(const TrackerConfig & config);
    void initialize(const ArmorMeasurement & armor, const rclcpp::Time & stamp);
    void reset();
    void predict(const rclcpp::Time & stamp);
    bool update(const std::vector<ArmorMeasurement> & armors);

    [[nodiscard]] bool active() const;
    [[nodiscard]] bool converged() const;
    [[nodiscard]] bool diverged() const;
    [[nodiscard]] bool jumped() const;
    [[nodiscard]] bool hasPrimaryArmor() const;
    [[nodiscard]] int primarySlot() const;
    [[nodiscard]] const std::array<bool, 3> & visibleSlots() const;
    [[nodiscard]] double nisFailureRatio() const;
    [[nodiscard]] const Eigen::VectorXd & state() const;
    [[nodiscard]] std::vector<geometry_msgs::msg::Pose> predictedArmors() const;
    [[nodiscard]] geometry_msgs::msg::Pose primaryArmorPose() const;

    int lost_count{0};

  private:
    static constexpr int kOutpostSlots = 3;

    struct OutpostAssignment
    {
      bool valid{false};
      int obs_count{0};
      double total_cost{std::numeric_limits<double>::infinity()};
      std::array<int, kOutpostSlots> obs_to_slot{{-1, -1, -1}};
      std::array<double, kOutpostSlots> obs_cost{{0.0, 0.0, 0.0}};
    };

    struct OutpostPrimaryDecision
    {
      int slot{-1};
    };

    double outpostMatchCost(const ArmorMeasurement & armor, const Eigen::Vector4d & slot_xyza) const;
    OutpostAssignment findBestAssignment(
      const std::vector<ArmorMeasurement> & armors,
      const std::vector<Eigen::Vector4d> & slot_xyza) const;
    OutpostPrimaryDecision choosePrimarySlot(const OutpostAssignment & assignment) const;
    int findObservationIndexForSlot(const OutpostAssignment & assignment, int slot) const;
    int findMinCostObservationIndex(const OutpostAssignment & assignment) const;
    int findMinCostSlot(const OutpostAssignment & assignment) const;
    Eigen::Vector3d armorPosition(const Eigen::VectorXd & x, int slot) const;
    Eigen::MatrixXd observationJacobian(const Eigen::VectorXd & x, int slot) const;
    std::vector<Eigen::Vector4d> predictedArmorStates() const;

    bool initialized_{false};
    bool jumped_{false};
    bool has_primary_slot_{false};
    int primary_slot_{0};
    std::array<bool, kOutpostSlots> visible_slots_{{false, false, false}};
    int mismatch_streak_{0};
    int update_count_{0};
    TrackerConfig config_;
    rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
    ExtendedKalmanFilter ekf_;
  };

  void onArmorPoseSets(
    bool from_front_0,
    const aim_msgs::msg::ArmorPoseSetArray::ConstSharedPtr msg);
  std::vector<ArmorMeasurement> extractMeasurements(
    const aim_msgs::msg::ArmorPoseSetArray::ConstSharedPtr & msg);
  void processArmorPoseSets(
    const aim_msgs::msg::ArmorPoseSetArray::ConstSharedPtr & msg,
    std::vector<ArmorMeasurement> measurements);
  void publishVisualization(
    const std_msgs::msg::Header & header,
    const aim_msgs::msg::OutpostState & state_msg);
  std::optional<double> lookupGimbalYaw(const std_msgs::msg::Header & header);

  static Eigen::Vector3d poseToYpr(const geometry_msgs::msg::Pose & pose);
  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion);
  static Eigen::Vector3d xyzToYpd(const Eigen::Vector3d & xyz);
  static Eigen::MatrixXd xyzToYpdJacobian(const Eigen::Vector3d & xyz);
  static double limitRad(double angle);
  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
  static geometry_msgs::msg::Point pointAlongPoseXAxis(
    const geometry_msgs::msg::Pose & pose,
    double distance);

  rclcpp::Subscription<aim_msgs::msg::ArmorPoseSetArray>::SharedPtr front_0_armor_pose_sub_;
  rclcpp::Subscription<aim_msgs::msg::ArmorPoseSetArray>::SharedPtr front_1_armor_pose_sub_;
  rclcpp::Publisher<aim_msgs::msg::OutpostState>::SharedPtr outpost_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string front_0_armor_pose_set_topic_;
  std::string front_1_armor_pose_set_topic_;
  std::string outpost_state_topic_;
  std::string visualization_topic_;
  std::string world_frame_id_;
  std::string gimbal_frame_id_;
  bool enable_visualization_{false};
  double tf_lookup_timeout_sec_{0.02};
  double armor_gimbal_yaw_gate_rad_{60.0 * 3.14159265358979323846 / 180.0};
  std::uint8_t outpost_id_{6};
  int max_lost_count_{75};
  int min_consecutive_detections_to_track_{1};
  double max_nis_failure_ratio_{0.4};
  double front_0_fallback_timeout_sec_{0.2};
  bool enable_front_1_fallback_{false};
  TrackerConfig tracker_config_;
  OutpostTracker tracker_;
  FrontCameraArbitrator front_camera_arbitrator_;
  std::mutex tracker_mutex_;
  int consecutive_detection_count_{0};
  int last_marker_count_{0};
  int last_selected_marker_count_{0};
  std::vector<geometry_msgs::msg::Pose> selected_armor_poses_;
};

}  // namespace aim_outpost_predictor
