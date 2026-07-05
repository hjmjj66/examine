#include "aim_camera_driver/usb_camera_component.hpp"

#include <chrono>
#include <cstring>
#include <string>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace aim_camera_driver
{

UsbCameraComponent::UsbCameraComponent(const rclcpp::NodeOptions & options)
: Node("usb_camera_node", options)
{
  // ---- 设备和话题 ----
  declare_parameter<int>("device_index", 0);
  declare_parameter<std::string>("frame_id", "usb_camera");
  declare_parameter<std::string>("image_topic", "/usb_camera/image_raw");
  declare_parameter<std::string>("camera_info_topic", "/usb_camera/camera_info");
  declare_parameter<double>("publish_rate", 30.0);

  // ---- 图像尺寸 ----
  declare_parameter<int>("width", 1280);
  declare_parameter<int>("height", 1024);
  declare_parameter<double>("fps", 30.0);

  // ---- 内参 ----
  declare_parameter<double>("camera_fx", 1000.0);
  declare_parameter<double>("camera_fy", 1000.0);
  declare_parameter<double>("camera_cx", 640.0);
  declare_parameter<double>("camera_cy", 512.0);
  declare_parameter<double>("dist_k1", 0.0);
  declare_parameter<double>("dist_k2", 0.0);
  declare_parameter<double>("dist_p1", 0.0);
  declare_parameter<double>("dist_p2", 0.0);
  declare_parameter<double>("dist_k3", 0.0);

  // ---- V4L2 图像控制 ----
  declare_parameter<bool>("exposure_auto", true);
  declare_parameter<double>("exposure", -1.0);
  declare_parameter<double>("gain", -1.0);
  declare_parameter<double>("brightness", -1.0);
  declare_parameter<double>("saturation", -1.0);
  declare_parameter<double>("contrast", -1.0);
  declare_parameter<double>("v4l2_exposure_mode_auto_value", 0.25);
  declare_parameter<double>("v4l2_exposure_mode_manual_value", 1.0);

  frame_id_ = get_parameter("frame_id").as_string();
  const auto image_topic = get_parameter("image_topic").as_string();
  camera_info_topic_ = get_parameter("camera_info_topic").as_string();
  const auto publish_rate = get_parameter("publish_rate").as_double();
  const auto device_index = static_cast<int>(get_parameter("device_index").as_int());
  width_ = get_parameter("width").as_int();
  height_ = get_parameter("height").as_int();
  const auto fps = get_parameter("fps").as_double();

  const double fx = get_parameter("camera_fx").as_double();
  const double fy = get_parameter("camera_fy").as_double();
  const double cx = get_parameter("camera_cx").as_double();
  const double cy = get_parameter("camera_cy").as_double();
  const double k1 = get_parameter("dist_k1").as_double();
  const double k2 = get_parameter("dist_k2").as_double();
  const double p1 = get_parameter("dist_p1").as_double();
  const double p2 = get_parameter("dist_p2").as_double();
  const double k3 = get_parameter("dist_k3").as_double();

  distortion_model_ = "plumb_bob";
  camera_matrix_ = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  distortion_coefficients_ = {k1, k2, p1, p2, k3};
  rectification_matrix_ = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  projection_matrix_ = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};

  // ---- 打开摄像头 ----
  cap_.open(device_index, cv::CAP_V4L2);
  if (!cap_.isOpened()) {
    cap_.open(device_index);
  }
  if (!cap_.isOpened()) {
    throw std::runtime_error("failed to open USB camera, index=" + std::to_string(device_index));
  }

  cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  if (width_ > 0) {
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
  }
  if (height_ > 0) {
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
  }
  if (fps > 0.0) {
    cap_.set(cv::CAP_PROP_FPS, fps);
  }
  cap_.set(cv::CAP_PROP_BUFFERSIZE, 10);

  device_index_ = device_index;

  // 应用 V4L2 控制参数
  const bool exp_auto = get_parameter("exposure_auto").as_bool();
  const double exp = get_parameter("exposure").as_double();
  const double gain = get_parameter("gain").as_double();
  const double bright = get_parameter("brightness").as_double();
  const double sat = get_parameter("saturation").as_double();
  const double contr = get_parameter("contrast").as_double();
  const double v4l2_auto = get_parameter("v4l2_exposure_mode_auto_value").as_double();
  const double v4l2_manual = get_parameter("v4l2_exposure_mode_manual_value").as_double();

  if (exp_auto) {
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, v4l2_auto);
  } else {
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, v4l2_manual);
    if (exp != -1.0) {
      cap_.set(cv::CAP_PROP_EXPOSURE, exp);
    }
  }
  if (gain != -1.0) { cap_.set(cv::CAP_PROP_GAIN, gain); }
  if (bright != -1.0) { cap_.set(cv::CAP_PROP_BRIGHTNESS, bright); }
  if (sat != -1.0) { cap_.set(cv::CAP_PROP_SATURATION, sat); }
  if (contr != -1.0) { cap_.set(cv::CAP_PROP_CONTRAST, contr); }

  // ---- 创建发布者 ----
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::QoS(1).transient_local());

  // ---- 创建定时器 ----
  const double safe_rate = publish_rate > 0.0 ? publish_rate : 30.0;
  const auto period = std::chrono::duration<double>(1.0 / safe_rate);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { publishFrame(); });

  RCLCPP_INFO(
    get_logger(),
    "usb camera component started: index=%d topic=%s info=%s rate=%.2fHz",
    device_index, image_topic.c_str(), camera_info_topic_.c_str(), safe_rate);
}

void UsbCameraComponent::publishFrame()
{
  cv::Mat bgr_frame;
  if (!cap_.read(bgr_frame) || bgr_frame.empty()) {
    ++consecutive_failures_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "failed to read frame from USB camera (consecutive=%d)", consecutive_failures_);
    if (consecutive_failures_ >= 5) {
      RCLCPP_WARN(get_logger(),
        "USB camera lost after %d consecutive failures, attempting reconnect...",
        consecutive_failures_);
      cap_.release();
      cap_.open(device_index_, cv::CAP_V4L2);
      if (!cap_.isOpened()) {
        cap_.open(device_index_);
      }
      if (cap_.isOpened()) {
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 10);
        consecutive_failures_ = 0;
        RCLCPP_INFO(get_logger(), "USB camera reconnected");
      }
    }
    return;
  }
  consecutive_failures_ = 0;

  // 使用 unique_ptr 发布，实现进程内零拷贝
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.frame_id = frame_id_;
  msg->height = static_cast<uint32_t>(bgr_frame.rows);
  msg->width = static_cast<uint32_t>(bgr_frame.cols);
  msg->encoding = "bgr8";
  msg->is_bigendian = false;
  msg->step = static_cast<uint32_t>(bgr_frame.cols * 3);
  const auto data_size = static_cast<size_t>(msg->step) * static_cast<size_t>(msg->height);
  msg->data.resize(data_size);

  // OpenCV BGR Mat 是连续存储的，直接拷贝
  std::memcpy(msg->data.data(), bgr_frame.data, data_size);

  const auto stamp = now();
  msg->header.stamp = stamp;

  // 构建 CameraInfo
  sensor_msgs::msg::CameraInfo cam_info;
  cam_info.header.stamp = stamp;
  cam_info.header.frame_id = frame_id_;
  cam_info.height = msg->height;
  cam_info.width = msg->width;
  cam_info.distortion_model = distortion_model_;
  cam_info.d = distortion_coefficients_;
  std::copy(camera_matrix_.begin(), camera_matrix_.end(), cam_info.k.begin());
  std::copy(rectification_matrix_.begin(), rectification_matrix_.end(), cam_info.r.begin());
  std::copy(projection_matrix_.begin(), projection_matrix_.end(), cam_info.p.begin());

  // 使用 unique_ptr move 发布，实现零拷贝
  image_pub_->publish(std::move(msg));
  camera_info_pub_->publish(cam_info);
}

}  // namespace aim_camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(aim_camera_driver::UsbCameraComponent)
