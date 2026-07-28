#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "aim_msgs/msg/armor_pose_set_array.hpp"
#include "aim_msgs/msg/armor_set_array.hpp"
#include "aim_msgs/msg/armor.hpp"
#include "aim_solver/fixed_timestamp_offsets.hpp"

namespace aim_solver
{

class AimSolverNode : public rclcpp::Node
{
public:
  explicit AimSolverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AimSolverNode() override;

private:
  enum class ArmorType
  {
    One,
    Two,
    Three,
    Four,
    Sentry,
    Outpost,
    Base,
    Negative,
  };

  struct LightBar
  {
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f axis;
  };

  struct ArmorObservation
  {
    ArmorType type{ArmorType::Negative};
    LightBar left_light;
    LightBar right_light;
  };

  struct PnPResult
  {
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    std::vector<double> project_errors;
  };

  struct SolverPipeline
  {
    std::string camera_name;
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    bool has_camera_intrinsics{false};
    std::mutex intrinsics_mutex;

    std::string armor_set_topic;
    std::string armor_pose_set_topic;
    std::string camera_info_topic;
    std::string visualization_topic;

    rclcpp::Subscription<aim_msgs::msg::ArmorSetArray>::SharedPtr armor_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    rclcpp::Publisher<aim_msgs::msg::ArmorPoseSetArray>::SharedPtr armor_pose_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub;
    double tf_timestamp_offset_sec{0.0};

    int last_marker_count{0};
  };

  void initSolverPipeline(
    SolverPipeline & pipeline,
    const std::string & prefix,
    bool use_camera_info_topic,
    bool enable_visualization);

  bool loadIntrinsicsForPipeline(
    SolverPipeline & pipeline,
    const std::string & prefix);

  void onArmorSets(
    SolverPipeline & pipeline,
    const aim_msgs::msg::ArmorSetArray::ConstSharedPtr msg);
  void onCameraInfo(
    SolverPipeline & pipeline,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  void publishVisualization(
    SolverPipeline & pipeline,
    const std_msgs::msg::Header & header,
    const std::vector<std::pair<ArmorType, geometry_msgs::msg::Pose>> & solved_poses);
  bool lookupFrameTransform(
    const std::string & source_frame_id,
    const builtin_interfaces::msg::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform,
    bool warn_on_failure = true);
  bool transformPose(
    const geometry_msgs::msg::Pose & source_pose,
    const geometry_msgs::msg::TransformStamped & transform,
    geometry_msgs::msg::Pose & target_pose);
  static geometry_msgs::msg::Point pointAlongPoseXAxis(
    const geometry_msgs::msg::Pose & pose,
    double distance);

  bool solveArmorPose(
    SolverPipeline & pipeline,
    const aim_msgs::msg::Armor & armor_msg,
    ArmorType armor_type,
    geometry_msgs::msg::Pose & pose_msg);
  void optimizeYaw(
    SolverPipeline & pipeline,
    const aim_msgs::msg::Armor & armor_msg,
    ArmorType armor_type,
    const geometry_msgs::msg::TransformStamped * target_transform,
    geometry_msgs::msg::Pose & pose_msg);
  bool buildObservation(
    const aim_msgs::msg::Armor & armor_msg,
    ArmorType armor_type,
    ArmorObservation & observation) const;
  bool solvePnP(
    SolverPipeline & pipeline,
    const ArmorObservation & observation,
    PnPResult & result);
  void sortPnPResult(const ArmorObservation & observation, PnPResult & result) const;
  std::vector<cv::Point2f> reprojectArmor(
    SolverPipeline & pipeline,
    const geometry_msgs::msg::Point & target_position,
    double yaw,
    ArmorType armor_type,
    const geometry_msgs::msg::TransformStamped * target_transform);
  double armorReprojectionError(
    SolverPipeline & pipeline,
    const aim_msgs::msg::Armor & armor_msg,
    const geometry_msgs::msg::Pose & pose_msg,
    double yaw,
    ArmorType armor_type,
    const geometry_msgs::msg::TransformStamped * target_transform);

  static ArmorType armorTypeFromClassId(std::uint8_t class_id);
  static const std::vector<cv::Point3f> & getArmorPoints(
    ArmorType armor_type, bool outpost_is_small_armor);
  static geometry_msgs::msg::Vector3 markerScaleForArmorType(
    ArmorType armor_type, bool outpost_is_small_armor);
  static std_msgs::msg::ColorRGBA markerColorForArmorType(ArmorType armor_type);
  static double limitRadian(
    double radian,
    std::pair<double, double> range = {-3.14159265358979323846, 3.14159265358979323846});
  static double radianToAngle(double radian);
  static cv::Vec3d rotationMatrixToRPY(const cv::Matx33d & rotation);
  static geometry_msgs::msg::Quaternion rotationMatrixToQuaternion(const cv::Matx33d & rotation);
  static cv::Matx33d quaternionToRotationMatrix(
    const geometry_msgs::msg::Quaternion & quaternion);
  static cv::Matx33d rpyToRotationMatrix(double roll, double pitch, double yaw);
  static double projectErrorAt(const std::vector<double> & project_errors, std::size_t index);
  double fixedPitchForArmorType(ArmorType armor_type) const;

  std::string target_frame_;
  bool use_current_time_for_tf_{false};
  bool use_generic_mode_{false};
  double project_error_ratio_thres_{3.0};
  double roll_thres_degree_{15.0};
  bool outpost_is_small_armor_{true};
  bool enable_optimize_yaw_{true};
  double optimize_yaw_pitch_rad_{15.0 * 3.14159265358979323846 / 180.0};
  double optimize_yaw_outpost_pitch_rad_{-15.0 * 3.14159265358979323846 / 180.0};
  double tf_lookup_timeout_sec_{0.05};
  SolverPipeline front_pipeline_;
  SolverPipeline back_pipeline_;
  SolverPipeline front_1_pipeline_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace aim_solver
