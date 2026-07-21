#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "gimbal_driver/msg/gimbal_angles.hpp"

namespace aim_handeye_calibrator
{

class AimHandeyeCalibratorNode : public rclcpp::Node
{
public:
  explicit AimHandeyeCalibratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Sample
  {
    rclcpp::Time stamp;
    double yaw_rad{0.0};
    double pitch_rad{0.0};
    cv::Mat rotation_target_to_optical;
    cv::Mat translation_target_to_optical;
    double reprojection_error_px{0.0};
  };

  struct SolveResult
  {
    cv::Mat rotation_gimbal_to_camera;
    cv::Mat translation_gimbal_to_camera;
    double mean_target_translation_delta_m{0.0};
    double max_target_translation_delta_m{0.0};
    double mean_target_rotation_delta_deg{0.0};
    double max_target_rotation_delta_deg{0.0};
  };

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void onGimbalState(const gimbal_driver::msg::GimbalAngles::ConstSharedPtr msg);

  void handleCapture(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleSolve(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleClear(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool detectTargetPose(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    cv::Mat & debug_image,
    cv::Mat & rotation_target_to_optical,
    cv::Mat & translation_target_to_optical,
    double & reprojection_error_px);
  bool solveCalibration(SolveResult & result) const;

  static cv::Mat toDoubleMatrix(const cv::Mat & input);
  cv::Mat buildBaseToGimbalTransform(double yaw_rad, double pitch_rad) const;
  cv::Mat cameraToOpticalRotation() const;
  std::vector<cv::Point3f> boardObjectPoints() const;
  double computeReprojectionError(
    const std::vector<cv::Point2f> & image_points,
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coefficients,
    const cv::Mat & rotation_target_to_optical,
    const cv::Mat & translation_target_to_optical) const;
  std::string formatTransformYaml(const SolveResult & result) const;
  bool saveResultToFile(const std::string & content) const;

  mutable std::mutex mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  std::optional<gimbal_driver::msg::GimbalAngles> latest_gimbal_state_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  std::vector<Sample> samples_;

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string gimbal_state_topic_;
  std::string debug_image_topic_;
  std::string output_file_;
  std::string parent_frame_id_;
  std::string child_frame_id_;
  std::string output_tf_section_;
  double square_size_m_{0.0};
  double yaw_sign_{1.0};
  double pitch_sign_{1.0};
  double min_angle_delta_rad_{0.0};
  double barrel_offset_z_{0.0};
  int board_cols_{9};
  int board_rows_{6};
  int min_samples_{8};
  bool angles_in_degree_{false};
  bool use_camera_info_topic_{false};
  bool use_pitch_{true};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<gimbal_driver::msg::GimbalAngles>::SharedPtr gimbal_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr solve_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
};

}  // namespace aim_handeye_calibrator
