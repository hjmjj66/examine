#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "DxImageProc.h"
#include "GxAPI.h"
#include "aim_msgs/msg/gx_timestamp_status.hpp"
#include "aim_camera_driver/gx_timestamp_mapper.hpp"

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
  bool initializeTimestampMapper();
  bool readTimestampLatch(std::uint64_t & device_tick, rclcpp::Time & host_time);
  bool refreshTimestampLatch();
  void maybeRefreshTimestampLatch();
  void recordTimestampMappingFailure(const char * stage, GX_STATUS status);
  void clearTimestampMappingFailure();

  // ---- 相机底层句柄 ----
  GX_DEV_HANDLE device_{nullptr};
  bool lib_initialized_{false};
  bool stream_on_{false};
  DX_PIXEL_COLOR_FILTER bayer_filter_{BAYERBG};

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
  std::string auto_balance_mode_{"once"};
  bool use_device_timestamp_{false};
  std::string timestamp_status_topic_;
  double timestamp_offset_sec_{0.0};
  double timestamp_tick_frequency_override_hz_{0.0};
  double timestamp_latch_period_sec_{1.0};
  double timestamp_latch_alpha_{0.1};
  double timestamp_latch_max_correction_sec_{0.02};
  double timestamp_frequency_estimation_interval_sec_{0.05};
  double timestamp_tick_frequency_hz_{0.0};
  bool timestamp_frequency_estimated_{false};
  bool timestamp_frequency_override_used_{false};
  int32_t timestamp_mapping_error_code_{0};
  std::string timestamp_mapping_error_stage_;
  GxTimestampMapper timestamp_mapper_;
  mutable std::mutex timestamp_mutex_;
  rclcpp::Time last_timestamp_latch_time_{0, 0, RCL_ROS_TIME};
  std::uint64_t last_timestamp_latch_tick_{0U};
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
  std::string image_topic_name_;
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
  rclcpp::Publisher<aim_msgs::msg::GxTimestampStatus>::SharedPtr
    timestamp_status_pub_;
  rclcpp::CallbackGroup::SharedPtr timestamp_latch_callback_group_;
  rclcpp::TimerBase::SharedPtr timestamp_latch_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace aim_camera_driver
