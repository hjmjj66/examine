#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "GxAPI.h"

namespace aim_camera_driver
{

class GxCameraComponent : public rclcpp::Node
{
public:
  explicit GxCameraComponent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publishFrame();

  bool openCamera();

  // ---- 大恒 SDK 操作 ----
  bool initGxLib();
  bool openGxDevice();
  bool configureGxDevice();
  bool setGxExposureGain();
  bool setGxWhiteBalance();
  bool setGxRoi();
  bool setGxAutoRoi();

  // ---- 相机底层句柄 ----
  GX_DEV_HANDLE device_{nullptr};
  bool lib_initialized_{false};
  bool stream_on_{false};

  // ---- 相机配置 ----
  std::string device_sn_{"auto"};
  double exposure_time_{5000.0};
  double auto_exposure_time_min_{3000.0};
  double auto_exposure_time_max_{8000.0};
  bool auto_exposure_{false};
  double gain_{12.0};
  double auto_gain_min_{6.0};
  double auto_gain_max_{18.0};
  bool auto_gain_{false};
  int64_t gray_value_min_{100};
  int64_t gray_value_max_{200};
  float balance_ratio_red_{1.0F};
  float balance_ratio_green_{1.0F};
  float balance_ratio_blue_{1.0F};
  int aae_width_{640};
  int aae_height_{512};
  int aae_offset_x_{320};
  int aae_offset_y_{256};

  // ---- ROS 相关 ----
  std::string frame_id_;
  std::string camera_info_topic_;
  int width_{1280};
  int height_{1024};
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
