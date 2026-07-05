#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/videoio.hpp>

namespace aim_camera_driver
{

class UsbCameraComponent : public rclcpp::Node
{
public:
  explicit UsbCameraComponent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publishFrame();

  cv::VideoCapture cap_;
  std::string frame_id_;
  std::string camera_info_topic_;
  int width_{1280};
  int height_{1024};
  int device_index_{0};
  int consecutive_failures_{0};
  std::string distortion_model_;
  std::vector<double> camera_matrix_;
  std::vector<double> distortion_coefficients_;
  std::vector<double> rectification_matrix_;
  std::vector<double> projection_matrix_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace aim_camera_driver
