#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace aim_handeye_calibrator
{

class AimCameraIntrinsicCalibratorNode : public rclcpp::Node
{
public:
  explicit AimCameraIntrinsicCalibratorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct CalibrationResult
  {
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    std::vector<cv::Mat> rotation_vectors;
    std::vector<cv::Mat> translation_vectors;
    double rms_reprojection_error_px{0.0};
    double mean_reprojection_error_px{0.0};
    double max_reprojection_error_px{0.0};
  };

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void handleCapture(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleSolve(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleClear(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool detectBoardCorners(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    std::vector<cv::Point2f> & corners,
    cv::Mat & debug_image) const;
  bool solveCalibration(CalibrationResult & result) const;

  std::vector<cv::Point3f> buildObjectPoints() const;
  double computeReprojectionErrors(CalibrationResult & result) const;
  std::string formatCalibrationYaml(const CalibrationResult & result) const;
  bool saveResultToFile(const std::string & content) const;

  mutable std::mutex mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  std::vector<std::vector<cv::Point2f>> image_points_;
  cv::Size image_size_;

  std::string image_topic_;
  std::string debug_image_topic_;
  std::string output_file_;
  std::string camera_name_;
  std::string frame_id_;
  std::string distortion_model_;
  std::string aim_solver_camera_matrix_key_;
  std::string aim_solver_distortion_key_;
  int board_cols_{9};
  int board_rows_{6};
  int min_samples_{12};
  double square_size_m_{0.025};
  bool use_fisheye_model_{false};
  bool fix_aspect_ratio_{false};
  bool zero_tangent_distortion_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr solve_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
};

}  // namespace aim_handeye_calibrator
