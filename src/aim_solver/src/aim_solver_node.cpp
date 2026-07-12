#include "aim_solver/aim_solver_node.hpp"
#include "aim_solver/coordinate_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/qos.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace aim_solver
{

namespace
{

constexpr double kBigArmorWidth = 230e-3;
constexpr double kSmallArmorWidth = 135e-3;
constexpr double kLightbarLength = 56e-3;

rclcpp::QoS makeHighRateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

rclcpp::QoS makeRealtimeTfQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

const std::vector<cv::Point3f> kBigArmorPoints{
  {0.0F, static_cast<float>(kBigArmorWidth / 2.0),
    static_cast<float>(-kLightbarLength / 2.0)},
  {0.0F, static_cast<float>(kBigArmorWidth / 2.0),
    static_cast<float>(kLightbarLength / 2.0)},
  {0.0F, static_cast<float>(-kBigArmorWidth / 2.0),
    static_cast<float>(kLightbarLength / 2.0)},
  {0.0F, static_cast<float>(-kBigArmorWidth / 2.0),
    static_cast<float>(-kLightbarLength / 2.0)},
};

const std::vector<cv::Point3f> kSmallArmorPoints{
  {0.0F, static_cast<float>(kSmallArmorWidth / 2.0),
    static_cast<float>(-kLightbarLength / 2.0)},
  {0.0F, static_cast<float>(kSmallArmorWidth / 2.0),
    static_cast<float>(kLightbarLength / 2.0)},
  {0.0F, static_cast<float>(-kSmallArmorWidth / 2.0),
    static_cast<float>(kLightbarLength / 2.0)},
  {0.0F, static_cast<float>(-kSmallArmorWidth / 2.0),
    static_cast<float>(-kLightbarLength / 2.0)},
};

}  // namespace

AimSolverNode::AimSolverNode(const rclcpp::NodeOptions & options)
: Node("aim_solver_node", options)
{
  // ---------- 鍓嶇浉鏈?澶ф亽宸ヤ笟鐩告満) 鍙傛暟 ----------
  declare_parameter<std::string>("front_armor_set_topic",
    "/aim_detector/front_0/armor_sets");
  declare_parameter<std::string>("front_armor_pose_set_topic",
    "/aim_solver/front_0/armor_pose_sets");
  declare_parameter<std::string>("front_camera_info_topic",
    "/gx_camera_0/camera_info");
  declare_parameter<std::string>("front_visualization_topic",
    "/aim_solver/front_0/visualization");
  declare_parameter<bool>("front_use_camera_info_topic", false);
  declare_parameter<std::vector<double>>("front_camera_matrix",
    std::vector<double>{});
  declare_parameter<std::vector<double>>("front_distortion_coefficients",
    std::vector<double>{});

  // ---------- 鍚庣浉鏈?USB鎽勫儚澶? 鍙傛暟 ----------
  declare_parameter<std::string>("back_armor_set_topic",
    "/aim_detector/back/armor_sets");
  declare_parameter<std::string>("back_armor_pose_set_topic",
    "/aim_solver/back/armor_pose_sets");
  declare_parameter<std::string>("back_camera_info_topic",
    "/usb_camera/camera_info");
  declare_parameter<std::string>("back_visualization_topic",
    "/aim_solver/back/visualization");
  declare_parameter<bool>("back_use_camera_info_topic", false);
  declare_parameter<std::vector<double>>("back_camera_matrix",
    std::vector<double>{});
  declare_parameter<std::vector<double>>("back_distortion_coefficients",
    std::vector<double>{});

  // ---------- 鍓嶇浉鏈?#1(澶ф亽宸ヤ笟鐩告満) 鍙傛暟 ----------
  declare_parameter<std::string>("front_1_armor_set_topic",
    "/aim_detector/front_1/armor_sets");
  declare_parameter<std::string>("front_1_armor_pose_set_topic",
    "/aim_solver/front_1/armor_pose_sets");
  declare_parameter<std::string>("front_1_camera_info_topic",
    "/gx_camera_1/camera_info");
  declare_parameter<std::string>("front_1_visualization_topic",
    "/aim_solver/front_1/visualization");
  declare_parameter<bool>("front_1_use_camera_info_topic", false);
  declare_parameter<std::vector<double>>("front_1_camera_matrix",
    std::vector<double>{});
  declare_parameter<std::vector<double>>("front_1_distortion_coefficients",
    std::vector<double>{});

  // ---------- 鍏变韩鍙傛暟 ----------
  declare_parameter<std::string>("target_frame", "gimbal_world");
  declare_parameter<double>("tf_lookup_timeout_sec", 0.05);
  declare_parameter<double>("tf_timestamp_offset_sec", 0.0);
  declare_parameter<bool>("use_current_time_for_tf", false);
  declare_parameter<bool>("enable_visualization", false);
  declare_parameter<bool>("use_generic_mode", false);
  declare_parameter<double>("project_error_ratio_thres", 3.0);
  declare_parameter<double>("roll_thres_degree", 15.0);
  declare_parameter<bool>("outpost_is_small_armor", true);
  declare_parameter<bool>("enable_optimize_yaw", true);
  declare_parameter<double>("optimize_yaw_pitch_deg", 15.0);
  declare_parameter<double>("optimize_yaw_outpost_pitch_deg", -15.0);

  target_frame_ = get_parameter("target_frame").as_string();
  tf_lookup_timeout_sec_ = get_parameter("tf_lookup_timeout_sec").as_double();
  tf_timestamp_offset_sec_ = get_parameter("tf_timestamp_offset_sec").as_double();
  use_current_time_for_tf_ = get_parameter("use_current_time_for_tf").as_bool();
  use_generic_mode_ = get_parameter("use_generic_mode").as_bool();
  project_error_ratio_thres_ = get_parameter("project_error_ratio_thres").as_double();
  roll_thres_degree_ = get_parameter("roll_thres_degree").as_double();
  outpost_is_small_armor_ = get_parameter("outpost_is_small_armor").as_bool();
  enable_optimize_yaw_ = get_parameter("enable_optimize_yaw").as_bool();
  optimize_yaw_pitch_rad_ =
    get_parameter("optimize_yaw_pitch_deg").as_double() * M_PI / 180.0;
  optimize_yaw_outpost_pitch_rad_ =
    get_parameter("optimize_yaw_outpost_pitch_deg").as_double() * M_PI / 180.0;

  const bool enable_vis = get_parameter("enable_visualization").as_bool();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_, this, true, makeRealtimeTfQos(), tf2_ros::StaticListenerQoS());

  front_pipeline_.camera_name = "front";
  back_pipeline_.camera_name = "back";
  front_1_pipeline_.camera_name = "front_1";

  const bool front_use_info = get_parameter("front_use_camera_info_topic").as_bool();
  const bool back_use_info = get_parameter("back_use_camera_info_topic").as_bool();
  const bool front_1_use_info = get_parameter("front_1_use_camera_info_topic").as_bool();

  initSolverPipeline(front_pipeline_, "front", front_use_info, enable_vis);
  initSolverPipeline(back_pipeline_, "back", back_use_info, enable_vis);
  initSolverPipeline(front_1_pipeline_, "front_1", front_1_use_info, enable_vis);

}

void AimSolverNode::initSolverPipeline(
  SolverPipeline & pipeline,
  const std::string & prefix,
  bool use_camera_info_topic,
  bool enable_visualization)
{
  const std::string armor_set_param = prefix + "_armor_set_topic";
  const std::string armor_pose_param = prefix + "_armor_pose_set_topic";
  const std::string camera_info_param = prefix + "_camera_info_topic";
  const std::string vis_param = prefix + "_visualization_topic";

  pipeline.armor_set_topic = get_parameter(armor_set_param).as_string();
  pipeline.armor_pose_set_topic = get_parameter(armor_pose_param).as_string();
  pipeline.camera_info_topic = get_parameter(camera_info_param).as_string();
  pipeline.visualization_topic = get_parameter(vis_param).as_string();

  pipeline.has_camera_intrinsics = loadIntrinsicsForPipeline(pipeline, prefix);

  const auto high_rate_qos = makeHighRateQos();
  pipeline.armor_pose_pub = create_publisher<aim_msgs::msg::ArmorPoseSetArray>(
    pipeline.armor_pose_set_topic, high_rate_qos);

  if (enable_visualization) {
    pipeline.visualization_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
      pipeline.visualization_topic, rclcpp::SensorDataQoS());
  }

  pipeline.armor_sub = create_subscription<aim_msgs::msg::ArmorSetArray>(
    pipeline.armor_set_topic,
    high_rate_qos,
    [this, &pipeline](const aim_msgs::msg::ArmorSetArray::ConstSharedPtr msg) {
      onArmorSets(pipeline, msg);
    });

  if (use_camera_info_topic) {
    pipeline.camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
      pipeline.camera_info_topic,
      rclcpp::SensorDataQoS().keep_last(1),
      [this, &pipeline](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        onCameraInfo(pipeline, msg);
      });
  }

  RCLCPP_INFO(
    get_logger(),
    "[%s] solver pipeline initialized: armor_in=%s, pose_out=%s",
    pipeline.camera_name.c_str(),
    pipeline.armor_set_topic.c_str(),
    pipeline.armor_pose_set_topic.c_str());
}

bool AimSolverNode::loadIntrinsicsForPipeline(
  SolverPipeline & pipeline,
  const std::string & prefix)
{
  const std::string camera_matrix_param = prefix + "_camera_matrix";
  const std::string distortion_param = prefix + "_distortion_coefficients";

  const auto camera_matrix_data =
    get_parameter(camera_matrix_param).as_double_array();
  const auto distortion_data =
    get_parameter(distortion_param).as_double_array();

  if (camera_matrix_data.size() != 9U) {
    RCLCPP_WARN(
      get_logger(),
      "[%s] camera_matrix parameter not set (size=%zu), will wait for CameraInfo topic",
      pipeline.camera_name.c_str(),
      camera_matrix_data.size());
    return false;
  }

  cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F);
  for (std::size_t i = 0; i < camera_matrix_data.size(); ++i) {
    camera_matrix.at<double>(
      static_cast<int>(i / 3U), static_cast<int>(i % 3U)) = camera_matrix_data[i];
  }

  cv::Mat distortion_coeffs = cv::Mat(
    static_cast<int>(distortion_data.size()), 1, CV_64F, cv::Scalar(0.0));
  for (std::size_t i = 0; i < distortion_data.size(); ++i) {
    distortion_coeffs.at<double>(static_cast<int>(i), 0) = distortion_data[i];
  }

  std::lock_guard<std::mutex> lock(pipeline.intrinsics_mutex);
  pipeline.camera_matrix = camera_matrix;
  pipeline.distortion_coefficients = distortion_coeffs;

  RCLCPP_INFO(
    get_logger(),
    "[%s] loaded intrinsics from parameters: fx=%.3f, fy=%.3f",
    pipeline.camera_name.c_str(),
    camera_matrix.at<double>(0, 0),
    camera_matrix.at<double>(1, 1));

  return true;
}

void AimSolverNode::onCameraInfo(
  SolverPipeline & pipeline,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F);
  for (std::size_t i = 0; i < msg->k.size(); ++i) {
    camera_matrix.at<double>(
      static_cast<int>(i / 3U), static_cast<int>(i % 3U)) = msg->k[i];
  }

  cv::Mat distortion_coefficients = cv::Mat(
    static_cast<int>(msg->d.size()), 1, CV_64F, cv::Scalar(0.0));
  for (std::size_t i = 0; i < msg->d.size(); ++i) {
    distortion_coefficients.at<double>(static_cast<int>(i), 0) = msg->d[i];
  }

  {
    std::lock_guard<std::mutex> lock(pipeline.intrinsics_mutex);
    pipeline.camera_matrix = camera_matrix;
    pipeline.distortion_coefficients = distortion_coefficients;
    pipeline.has_camera_intrinsics = true;
  }
}

void AimSolverNode::onArmorSets(
  SolverPipeline & pipeline,
  const aim_msgs::msg::ArmorSetArray::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(pipeline.intrinsics_mutex);
    if (!pipeline.has_camera_intrinsics || pipeline.camera_matrix.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[%s] camera intrinsics unavailable for armor pose solving",
        pipeline.camera_name.c_str());
      return;
    }
  }

  builtin_interfaces::msg::Time transform_stamp_msg = msg->header.stamp;
  if (!use_current_time_for_tf_) {
    transform_stamp_msg =
      rclcpp::Time(msg->header.stamp) +
      rclcpp::Duration::from_seconds(tf_timestamp_offset_sec_);
  }
  geometry_msgs::msg::TransformStamped frame_transform;
  const bool should_transform =
    !target_frame_.empty() && msg->header.frame_id != target_frame_;
  if (should_transform) {
    if (!lookupFrameTransform(
        msg->header.frame_id, transform_stamp_msg, frame_transform))
    {
      return;
    }
    transform_stamp_msg = frame_transform.header.stamp;
  }

  aim_msgs::msg::ArmorPoseSetArray output;
  output.header = msg->header;
  output.header.stamp = transform_stamp_msg;
  if (should_transform) {
    output.header.frame_id = target_frame_;
  }
  output.armor_pose_sets.reserve(msg->armor_sets.size());
  std::vector<std::pair<ArmorType, geometry_msgs::msg::Pose>> solved_poses;

  for (const auto & armor_set : msg->armor_sets) {
    aim_msgs::msg::ArmorPoseSet pose_set;
    pose_set.header = msg->header;
    pose_set.header.stamp = transform_stamp_msg;
    if (should_transform) {
      pose_set.header.frame_id = target_frame_;
    }
    pose_set.id = armor_set.id;
    pose_set.armor_poses.reserve(armor_set.armors.size());

    for (const auto & armor_msg : armor_set.armors) {
      geometry_msgs::msg::Pose pose_msg;
      const auto armor_type = armorTypeFromClassId(armor_msg.armor_class.class_id);
      if (armor_type == ArmorType::Negative) {
        continue;
      }
      if (!solveArmorPose(pipeline, armor_msg, armor_type, pose_msg)) {
        continue;
      }

      if (should_transform) {
        geometry_msgs::msg::Pose transformed_pose;
        if (!transformPose(pose_msg, frame_transform, transformed_pose)) {
          continue;
        }
        pose_msg = transformed_pose;
      }

      if (enable_optimize_yaw_) {
        optimizeYaw(
          pipeline, armor_msg, armor_type,
          should_transform ? &frame_transform : nullptr,
          pose_msg);
      }

      pose_set.armor_poses.push_back(pose_msg);
      solved_poses.emplace_back(armor_type, pose_msg);
    }

    if (!pose_set.armor_poses.empty()) {
      output.armor_pose_sets.push_back(std::move(pose_set));
    }
  }

  const auto visualization_header = output.header;

  if (!output.armor_pose_sets.empty()) {
    RCLCPP_DEBUG(
      get_logger(),
      "[%s] solved %zu armor pose sets",
      pipeline.camera_name.c_str(),
      output.armor_pose_sets.size());
  }

  pipeline.armor_pose_pub->publish(std::move(output));

  if (pipeline.visualization_pub) {
    publishVisualization(pipeline, visualization_header, solved_poses);
  }
}

bool AimSolverNode::lookupFrameTransform(
  const std::string & source_frame_id,
  const builtin_interfaces::msg::Time & stamp,
  geometry_msgs::msg::TransformStamped & transform)
{
  if (source_frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "armor set header.frame_id is empty, cannot align frames");
    return false;
  }

  try {
    const auto tf_stamp = use_current_time_for_tf_ ?
      tf2::TimePointZero :
      tf2_ros::fromMsg(stamp);
    transform = tf_buffer_->lookupTransform(
      target_frame_, source_frame_id, tf_stamp,
      tf2::durationFromSec(tf_lookup_timeout_sec_));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "failed to lookup transform from %s to %s near stamp %.9f: %s",
      source_frame_id.c_str(),
      target_frame_.c_str(),
      rclcpp::Time(stamp).seconds(),
      ex.what());
    return false;
  }
}

bool AimSolverNode::transformPose(
  const geometry_msgs::msg::Pose & source_pose,
  const geometry_msgs::msg::TransformStamped & transform,
  geometry_msgs::msg::Pose & target_pose)
{
  geometry_msgs::msg::PoseStamped source_pose_stamped;
  source_pose_stamped.header = transform.header;
  source_pose_stamped.header.frame_id = transform.child_frame_id;
  source_pose_stamped.pose = source_pose;

  geometry_msgs::msg::PoseStamped target_pose_stamped;
  tf2::doTransform(source_pose_stamped, target_pose_stamped, transform);
  target_pose = target_pose_stamped.pose;
  return true;
}

bool AimSolverNode::solveArmorPose(
  SolverPipeline & pipeline,
  const aim_msgs::msg::Armor & armor_msg,
  ArmorType armor_type,
  geometry_msgs::msg::Pose & pose_msg)
{
  ArmorObservation observation;
  if (!buildObservation(armor_msg, armor_type, observation)) {
    return false;
  }

  PnPResult pnp_result;
  if (!solvePnP(pipeline, observation, pnp_result)) {
    return false;
  }

  const cv::Mat & tvec = pnp_result.tvecs.front();
  const cv::Vec3d optical_position(
    tvec.at<double>(0),
    tvec.at<double>(1),
    tvec.at<double>(2));
  const cv::Vec3d camera_position = opticalPointToSolverInputFrame(optical_position);
  pose_msg.position.x = camera_position[0];
  pose_msg.position.y = camera_position[1];
  pose_msg.position.z = camera_position[2];

  cv::Mat rotation_cv;
  cv::Rodrigues(pnp_result.rvecs.front(), rotation_cv);
  cv::Matx33d optical_rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      optical_rotation(row, col) = rotation_cv.at<double>(row, col);
    }
  }
  pose_msg.orientation = rotationMatrixToQuaternion(
    opticalRotationToSolverInputFrame(optical_rotation));
  return true;
}

void AimSolverNode::optimizeYaw(
  SolverPipeline & pipeline,
  const aim_msgs::msg::Armor & armor_msg,
  ArmorType armor_type,
  const geometry_msgs::msg::TransformStamped * target_transform,
  geometry_msgs::msg::Pose & pose_msg)
{
  const double current_yaw =
    rotationMatrixToRPY(quaternionToRotationMatrix(pose_msg.orientation))[2];
  constexpr double search_range_deg = 140.0;
  const double yaw0 =
    limitRadian(current_yaw - search_range_deg * 0.5 * M_PI / 180.0);

  double min_error = std::numeric_limits<double>::max();
  double best_yaw = current_yaw;
  for (int i = 0; i < static_cast<int>(search_range_deg); ++i) {
    const double yaw = limitRadian(
      yaw0 + static_cast<double>(i) * M_PI / 180.0);
    const double error = armorReprojectionError(
      pipeline, armor_msg, pose_msg, yaw, armor_type, target_transform);
    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  const double pitch = fixedPitchForArmorType(armor_type);
  const double roll = 0.0;
  const cv::Matx33d optimized_rotation =
    cv::Matx33d(
      std::cos(best_yaw) * std::cos(pitch),
      std::cos(best_yaw) * std::sin(pitch) * std::sin(roll) -
        std::sin(best_yaw) * std::cos(roll),
      std::cos(best_yaw) * std::sin(pitch) * std::cos(roll) +
        std::sin(best_yaw) * std::sin(roll),
      std::sin(best_yaw) * std::cos(pitch),
      std::sin(best_yaw) * std::sin(pitch) * std::sin(roll) +
        std::cos(best_yaw) * std::cos(roll),
      std::sin(best_yaw) * std::sin(pitch) * std::cos(roll) -
        std::cos(best_yaw) * std::sin(roll),
      -std::sin(pitch),
      std::cos(pitch) * std::sin(roll),
      std::cos(pitch) * std::cos(roll));
  pose_msg.orientation = rotationMatrixToQuaternion(optimized_rotation);
}

void AimSolverNode::publishVisualization(
  SolverPipeline & pipeline,
  const std_msgs::msg::Header & header,
  const std::vector<std::pair<ArmorType, geometry_msgs::msg::Pose>> & solved_poses)
{
  if (!pipeline.visualization_pub) {
    return;
  }

  std_msgs::msg::Header marker_header = header;
  if (!target_frame_.empty()) {
    marker_header.frame_id = target_frame_;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;

  for (const auto & solved_pose : solved_poses) {
    const auto scale = markerScaleForArmorType(
      solved_pose.first, outpost_is_small_armor_);
    const auto color = markerColorForArmorType(solved_pose.first);

    visualization_msgs::msg::Marker cube_marker;
    cube_marker.header = marker_header;
    cube_marker.ns = "armor_pose";
    cube_marker.id = marker_id++;
    cube_marker.type = visualization_msgs::msg::Marker::CUBE;
    cube_marker.action = visualization_msgs::msg::Marker::ADD;
    cube_marker.pose = solved_pose.second;
    cube_marker.scale = scale;
    cube_marker.color = color;
    cube_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    cube_marker.frame_locked = false;
    marker_array.markers.push_back(std::move(cube_marker));

    visualization_msgs::msg::Marker axis_marker;
    axis_marker.header = marker_header;
    axis_marker.ns = "armor_pose_x_axis";
    axis_marker.id = marker_id++;
    axis_marker.type = visualization_msgs::msg::Marker::ARROW;
    axis_marker.action = visualization_msgs::msg::Marker::ADD;
    axis_marker.scale.x = 0.01;
    axis_marker.scale.y = 0.02;
    axis_marker.scale.z = 0.03;
    axis_marker.color.r = 1.0F;
    axis_marker.color.g = 0.1F;
    axis_marker.color.b = 0.1F;
    axis_marker.color.a = 0.95F;
    axis_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    axis_marker.frame_locked = false;
    axis_marker.points.push_back(solved_pose.second.position);
    axis_marker.points.push_back(pointAlongPoseXAxis(
        solved_pose.second,
        std::max({scale.x, scale.y, scale.z}) * 0.7));
    marker_array.markers.push_back(std::move(axis_marker));
  }

  for (int stale_id = marker_id;
       stale_id < pipeline.last_marker_count;
       ++stale_id)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = marker_header;
    marker.ns = (stale_id % 2 == 0) ? "armor_pose" : "armor_pose_x_axis";
    marker.id = stale_id;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(std::move(marker));
  }

  pipeline.last_marker_count = marker_id;
  pipeline.visualization_pub->publish(std::move(marker_array));
}

geometry_msgs::msg::Point AimSolverNode::pointAlongPoseXAxis(
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

bool AimSolverNode::buildObservation(
  const aim_msgs::msg::Armor & armor_msg,
  ArmorType armor_type,
  ArmorObservation & observation) const
{
  const auto & corners = armor_msg.corners;
  const cv::Point2f left_top(
    static_cast<float>(corners[0].x),
    static_cast<float>(corners[0].y));
  const cv::Point2f left_bottom(
    static_cast<float>(corners[1].x),
    static_cast<float>(corners[1].y));
  const cv::Point2f right_bottom(
    static_cast<float>(corners[2].x),
    static_cast<float>(corners[2].y));
  const cv::Point2f right_top(
    static_cast<float>(corners[3].x),
    static_cast<float>(corners[3].y));

  const auto left_axis = left_top - left_bottom;
  const auto right_axis = right_top - right_bottom;
  const float left_norm = cv::norm(left_axis);
  const float right_norm = cv::norm(right_axis);
  if (left_norm <= 1e-6F || right_norm <= 1e-6F) {
    return false;
  }

  observation.type = armor_type;
  observation.left_light.top = left_top;
  observation.left_light.bottom = left_bottom;
  observation.left_light.axis = left_axis * (1.0F / left_norm);
  observation.right_light.top = right_top;
  observation.right_light.bottom = right_bottom;
  observation.right_light.axis = right_axis * (1.0F / right_norm);
  return true;
}

bool AimSolverNode::solvePnP(
  SolverPipeline & pipeline,
  const ArmorObservation & observation,
  PnPResult & result)
{
  cv::Mat camera_matrix;
  cv::Mat distortion_coefficients;
  {
    std::lock_guard<std::mutex> lock(pipeline.intrinsics_mutex);
    camera_matrix = pipeline.camera_matrix.clone();
    distortion_coefficients = pipeline.distortion_coefficients.clone();
  }

  const auto & object_points = getArmorPoints(
    observation.type, outpost_is_small_armor_);
  const std::vector<cv::Point2f> image_points{
    observation.left_light.bottom,
    observation.left_light.top,
    observation.right_light.top,
    observation.right_light.bottom,
  };

  if (use_generic_mode_) {
    int solutions = cv::solvePnPGeneric(
      object_points,
      image_points,
      camera_matrix,
      distortion_coefficients,
      result.rvecs,
      result.tvecs,
      false,
      cv::SOLVEPNP_IPPE,
      cv::noArray(),
      cv::noArray(),
      result.project_errors);
    if (solutions <= 0 || result.rvecs.empty() || result.tvecs.empty()) {
      return false;
    }
    sortPnPResult(observation, result);
    return true;
  }

  result.rvecs.resize(1);
  result.tvecs.resize(1);
  const bool success = cv::solvePnP(
    object_points,
    image_points,
    camera_matrix,
    distortion_coefficients,
    result.rvecs[0],
    result.tvecs[0],
    false,
    cv::SOLVEPNP_IPPE);
  return success;
}

void AimSolverNode::sortPnPResult(
  const ArmorObservation & observation,
  PnPResult & result) const
{
  if (result.rvecs.size() < 2U || result.tvecs.size() < 2U ||
      result.project_errors.size() < 2U) {
    return;
  }

  const double error0 = projectErrorAt(result.project_errors, 0U);
  const double error1 = projectErrorAt(result.project_errors, 1U);
  if (error0 <= 1e-9) {
    return;
  }

  const double ratio = error1 / error0;
  if (ratio > project_error_ratio_thres_) {
    return;
  }

  cv::Mat rotation0_cv;
  cv::Mat rotation1_cv;
  cv::Rodrigues(result.rvecs[0], rotation0_cv);
  cv::Rodrigues(result.rvecs[1], rotation1_cv);

  cv::Matx33d rotation0;
  cv::Matx33d rotation1;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation0(row, col) = rotation0_cv.at<double>(row, col);
      rotation1(row, col) = rotation1_cv.at<double>(row, col);
    }
  }

  const cv::Vec3d rpy0 = rotationMatrixToRPY(
    opticalRotationToCameraFrame(rotation0));
  const cv::Vec3d rpy1 = rotationMatrixToRPY(
    opticalRotationToCameraFrame(rotation1));

  const double roll0 = radianToAngle(limitRadian(
      rpy0[0], {-1.5707963267948966, 1.5707963267948966}));
  if (roll0 > roll_thres_degree_) {
    return;
  }

  const double left_angle = radianToAngle(
    std::atan2(static_cast<double>(observation.left_light.axis.y),
      static_cast<double>(observation.left_light.axis.x)));
  const double right_angle = radianToAngle(
    std::atan2(static_cast<double>(observation.right_light.axis.y),
      static_cast<double>(observation.right_light.axis.x)));
  const double angle = (left_angle + right_angle) / 2.0;
  double armor_angle = angle + 90.0;
  if (observation.type == ArmorType::Outpost) {
    armor_angle = -armor_angle;
  }

  if ((armor_angle > 0.0 && rpy0[2] > 0.0 && rpy1[2] < 0.0) ||
    (armor_angle < 0.0 && rpy0[2] < 0.0 && rpy1[2] > 0.0))
  {
    std::swap(result.rvecs[0], result.rvecs[1]);
    std::swap(result.tvecs[0], result.tvecs[1]);
  }
}

std::vector<cv::Point2f> AimSolverNode::reprojectArmor(
  SolverPipeline & pipeline,
  const geometry_msgs::msg::Point & target_position,
  double yaw,
  ArmorType armor_type,
  const geometry_msgs::msg::TransformStamped * target_transform)
{
  cv::Mat camera_matrix;
  cv::Mat distortion_coefficients;
  {
    std::lock_guard<std::mutex> lock(pipeline.intrinsics_mutex);
    camera_matrix = pipeline.camera_matrix.clone();
    distortion_coefficients = pipeline.distortion_coefficients.clone();
  }

  const auto & object_points = getArmorPoints(
    armor_type, outpost_is_small_armor_);
  const double pitch = fixedPitchForArmorType(armor_type);

  const cv::Matx33d rotation_armor_to_world(
    std::cos(yaw) * std::cos(pitch), -std::sin(yaw),
    std::cos(yaw) * std::sin(pitch),
    std::sin(yaw) * std::cos(pitch), std::cos(yaw),
    std::sin(yaw) * std::sin(pitch),
    -std::sin(pitch), 0.0, std::cos(pitch));

  cv::Matx33d rotation_world_to_camera = cv::Matx33d::eye();
  cv::Vec3d translation_world_to_camera(0.0, 0.0, 0.0);
  if (target_transform != nullptr) {
    rotation_world_to_camera =
      quaternionToRotationMatrix(target_transform->transform.rotation).t();
    translation_world_to_camera = cv::Vec3d(
      -target_transform->transform.translation.x,
      -target_transform->transform.translation.y,
      -target_transform->transform.translation.z);
    translation_world_to_camera =
      rotation_world_to_camera * translation_world_to_camera;
  }

  const cv::Matx33d rotation_armor_to_camera =
    rotation_world_to_camera * rotation_armor_to_world;
  const cv::Vec3d target_world(
    target_position.x, target_position.y, target_position.z);
  const cv::Vec3d target_camera =
    rotation_world_to_camera * target_world + translation_world_to_camera;
  const cv::Matx33d rotation_armor_to_optical =
    opticalRotationToSolverInputFrame(rotation_armor_to_camera);
  const cv::Vec3d target_optical = opticalPointToSolverInputFrame(target_camera);

  cv::Mat rotation_cv(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation_cv.at<double>(row, col) = rotation_armor_to_optical(row, col);
    }
  }

  cv::Vec3d rvec;
  cv::Rodrigues(rotation_cv, rvec);
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(
    object_points, rvec, target_optical,
    camera_matrix, distortion_coefficients, image_points);
  return image_points;
}

double AimSolverNode::armorReprojectionError(
  SolverPipeline & pipeline,
  const aim_msgs::msg::Armor & armor_msg,
  const geometry_msgs::msg::Pose & pose_msg,
  double yaw,
  ArmorType armor_type,
  const geometry_msgs::msg::TransformStamped * target_transform)
{
  const auto image_points = reprojectArmor(
    pipeline, pose_msg.position, yaw, armor_type, target_transform);
  if (image_points.size() != 4U) {
    return std::numeric_limits<double>::max();
  }

  const std::array<cv::Point2f, 4> reordered_corners{
    cv::Point2f(
      static_cast<float>(armor_msg.corners[1].x),
      static_cast<float>(armor_msg.corners[1].y)),
    cv::Point2f(
      static_cast<float>(armor_msg.corners[0].x),
      static_cast<float>(armor_msg.corners[0].y)),
    cv::Point2f(
      static_cast<float>(armor_msg.corners[3].x),
      static_cast<float>(armor_msg.corners[3].y)),
    cv::Point2f(
      static_cast<float>(armor_msg.corners[2].x),
      static_cast<float>(armor_msg.corners[2].y)),
  };

  double error = 0.0;
  for (int i = 0; i < 4; ++i) {
    error += cv::norm(
      reordered_corners[static_cast<std::size_t>(i)] -
      image_points[static_cast<std::size_t>(i)]);
  }
  return error;
}

double AimSolverNode::fixedPitchForArmorType(ArmorType armor_type) const
{
  return armor_type == ArmorType::Outpost ?
    optimize_yaw_outpost_pitch_rad_ : optimize_yaw_pitch_rad_;
}

AimSolverNode::ArmorType AimSolverNode::armorTypeFromClassId(
  std::uint8_t class_id)
{
  switch (class_id) {
    case 0:
      return ArmorType::Sentry;
    case 1:
      return ArmorType::One;
    case 2:
      return ArmorType::Two;
    case 3:
      return ArmorType::Three;
    case 4:
      return ArmorType::Four;
    case 6:
      return ArmorType::Outpost;
    case 7:
      return ArmorType::Base;
    default:
      return ArmorType::Negative;
  }
}

const std::vector<cv::Point3f> & AimSolverNode::getArmorPoints(
  ArmorType armor_type,
  bool outpost_is_small_armor)
{
  switch (armor_type) {
    case ArmorType::One:
      return kBigArmorPoints;
    case ArmorType::Outpost:
      return outpost_is_small_armor ? kSmallArmorPoints : kBigArmorPoints;
    case ArmorType::Two:
    case ArmorType::Three:
    case ArmorType::Four:
    case ArmorType::Sentry:
    case ArmorType::Base:
    case ArmorType::Negative:
    default:
      return kSmallArmorPoints;
  }
}

geometry_msgs::msg::Vector3 AimSolverNode::markerScaleForArmorType(
  ArmorType armor_type,
  bool outpost_is_small_armor)
{
  geometry_msgs::msg::Vector3 scale;
  const auto & armor_points = getArmorPoints(armor_type, outpost_is_small_armor);
  const double width = std::abs(armor_points[0].y - armor_points[2].y);
  const double height = std::abs(armor_points[1].z - armor_points[0].z);
  scale.x = 0.01;
  scale.y = width;
  scale.z = height;
  return scale;
}

std_msgs::msg::ColorRGBA AimSolverNode::markerColorForArmorType(
  ArmorType armor_type)
{
  std_msgs::msg::ColorRGBA color;
  color.a = 0.8F;
  switch (armor_type) {
    case ArmorType::One:
      color.r = 1.0F;
      color.g = 0.3F;
      color.b = 0.3F;
      break;
    case ArmorType::Outpost:
      color.r = 1.0F;
      color.g = 0.8F;
      color.b = 0.2F;
      break;
    case ArmorType::Base:
      color.r = 0.8F;
      color.g = 0.2F;
      color.b = 1.0F;
      break;
    default:
      color.r = 0.2F;
      color.g = 0.9F;
      color.b = 0.3F;
      break;
  }
  return color;
}

double AimSolverNode::limitRadian(double radian, std::pair<double, double> range)
{
  const double low = range.first;
  const double high = range.second;
  const double width = high - low;
  radian = std::fmod(radian - low, width);
  if (radian < 0.0) {
    radian += width;
  }
  return radian + low;
}

double AimSolverNode::radianToAngle(double radian)
{
  return radian * 180.0 / 3.14159265358979323846;
}

cv::Vec3d AimSolverNode::rotationMatrixToRPY(const cv::Matx33d & rotation)
{
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  const double pitch = std::atan2(
    -rotation(2, 0), std::hypot(rotation(2, 1), rotation(2, 2)));
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  return {roll, pitch, yaw};
}

geometry_msgs::msg::Quaternion AimSolverNode::rotationMatrixToQuaternion(
  const cv::Matx33d & rotation)
{
  geometry_msgs::msg::Quaternion quaternion;
  const double trace = rotation(0, 0) + rotation(1, 1) + rotation(2, 2);

  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    quaternion.w = 0.25 * s;
    quaternion.x = (rotation(2, 1) - rotation(1, 2)) / s;
    quaternion.y = (rotation(0, 2) - rotation(2, 0)) / s;
    quaternion.z = (rotation(1, 0) - rotation(0, 1)) / s;
    return quaternion;
  }

  if (rotation(0, 0) > rotation(1, 1) && rotation(0, 0) > rotation(2, 2)) {
    const double s = std::sqrt(
      1.0 + rotation(0, 0) - rotation(1, 1) - rotation(2, 2)) * 2.0;
    quaternion.w = (rotation(2, 1) - rotation(1, 2)) / s;
    quaternion.x = 0.25 * s;
    quaternion.y = (rotation(0, 1) + rotation(1, 0)) / s;
    quaternion.z = (rotation(0, 2) + rotation(2, 0)) / s;
    return quaternion;
  }

  if (rotation(1, 1) > rotation(2, 2)) {
    const double s = std::sqrt(
      1.0 + rotation(1, 1) - rotation(0, 0) - rotation(2, 2)) * 2.0;
    quaternion.w = (rotation(0, 2) - rotation(2, 0)) / s;
    quaternion.x = (rotation(0, 1) + rotation(1, 0)) / s;
    quaternion.y = 0.25 * s;
    quaternion.z = (rotation(1, 2) + rotation(2, 1)) / s;
    return quaternion;
  }

  const double s = std::sqrt(
    1.0 + rotation(2, 2) - rotation(0, 0) - rotation(1, 1)) * 2.0;
  quaternion.w = (rotation(1, 0) - rotation(0, 1)) / s;
  quaternion.x = (rotation(0, 2) + rotation(2, 0)) / s;
  quaternion.y = (rotation(1, 2) + rotation(2, 1)) / s;
  quaternion.z = 0.25 * s;
  return quaternion;
}

cv::Matx33d AimSolverNode::quaternionToRotationMatrix(
  const geometry_msgs::msg::Quaternion & quaternion)
{
  const double x = quaternion.x;
  const double y = quaternion.y;
  const double z = quaternion.z;
  const double w = quaternion.w;

  return cv::Matx33d(
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
    2.0 * (x * z + y * w),
    2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
    2.0 * (y * z - x * w),
    2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
    1.0 - 2.0 * (x * x + y * y));
}

cv::Matx33d AimSolverNode::rpyToRotationMatrix(
  double roll, double pitch, double yaw)
{
  const double sr = std::sin(roll);
  const double cr = std::cos(roll);
  const double sp = std::sin(pitch);
  const double cp = std::cos(pitch);
  const double sy = std::sin(yaw);
  const double cy = std::cos(yaw);

  return cv::Matx33d(
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr);
}

double AimSolverNode::projectErrorAt(
  const std::vector<double> & project_errors, std::size_t index)
{
  if (index >= project_errors.size()) {
    return 0.0;
  }
  return project_errors[index];
}

}  // namespace aim_solver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aim_solver::AimSolverNode>());
  rclcpp::shutdown();
  return 0;
}
