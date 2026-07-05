#include "aim_camera_driver/gx_camera_component.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <string>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "DxImageProc.h"

namespace aim_camera_driver
{

namespace
{

bool gx_ok(GX_STATUS status)
{
  return status == GX_STATUS_SUCCESS;
}

}  // namespace

GxCameraComponent::GxCameraComponent(const rclcpp::NodeOptions & options)
: Node("gx_camera_node", options)
{
  // ---- 设备和话题 ----
  declare_parameter<std::string>("device_sn", "");
  declare_parameter<int>("device_index", 1);
  declare_parameter<std::string>("frame_id", "gx_camera");
  declare_parameter<std::string>("image_topic", "/gx_camera/image_raw");
  declare_parameter<std::string>("camera_info_topic", "/gx_camera/camera_info");
  declare_parameter<double>("publish_rate", 100.0);

  // ---- 图像尺寸 ----
  declare_parameter<int>("width", 1280);
  declare_parameter<int>("height", 1024);

  // ---- 曝光/增益 ----
  declare_parameter<double>("gain", 12.0);
  declare_parameter<double>("auto_gain_max", 12.0);
  declare_parameter<double>("auto_gain_min", 2.0);
  declare_parameter<int>("auto_gain", 0);
  declare_parameter<double>("exposure_time", 4000.0);
  declare_parameter<double>("auto_exposure_time_max", 10000.0);
  declare_parameter<double>("auto_exposure_time_min", 1000.0);
  declare_parameter<int>("auto_exposure", 0);

  // ---- 白平衡 ----
  declare_parameter<double>("red_balance_ratio", 1.0);
  declare_parameter<double>("green_balance_ratio", 1.0);
  declare_parameter<double>("blue_balance_ratio", 1.0);

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

  // ---- 读取 ROS 参数到成员变量 ----
  frame_id_ = get_parameter("frame_id").as_string();
  const auto image_topic = get_parameter("image_topic").as_string();
  camera_info_topic_ = get_parameter("camera_info_topic").as_string();
  const auto publish_rate = get_parameter("publish_rate").as_double();
  width_ = get_parameter("width").as_int();
  height_ = get_parameter("height").as_int();

  device_sn_ = get_parameter("device_sn").as_string();
  exposure_time_ = get_parameter("exposure_time").as_double();
  auto_exposure_time_max_ = get_parameter("auto_exposure_time_max").as_double();
  auto_exposure_time_min_ = get_parameter("auto_exposure_time_min").as_double();
  auto_exposure_ = (get_parameter("auto_exposure").as_int() != 0);
  gain_ = get_parameter("gain").as_double();
  auto_gain_max_ = get_parameter("auto_gain_max").as_double();
  auto_gain_min_ = get_parameter("auto_gain_min").as_double();
  auto_gain_ = (get_parameter("auto_gain").as_int() != 0);
  balance_ratio_red_ = static_cast<float>(get_parameter("red_balance_ratio").as_double());
  balance_ratio_green_ = static_cast<float>(get_parameter("green_balance_ratio").as_double());
  balance_ratio_blue_ = static_cast<float>(get_parameter("blue_balance_ratio").as_double());

  aae_width_ = width_ / 2;
  aae_height_ = height_ / 2;
  aae_offset_x_ = width_ / 4;
  aae_offset_y_ = height_ / 4;

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

  // ---- 创建发布者 ----
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::SensorDataQoS());
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::QoS(1).transient_local());

  // ---- 打开相机 ----
  if (!openCamera()) {
    throw std::runtime_error("failed to open daheng camera");
  }

  // ---- 创建定时器 ----
  const double safe_rate = publish_rate > 0.0 ? publish_rate : 100.0;
  const auto period = std::chrono::duration<double>(1.0 / safe_rate);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { publishFrame(); });

  RCLCPP_INFO(
    get_logger(),
    "gx camera component started: topic=%s, info=%s, rate=%.2fHz",
    image_topic.c_str(), camera_info_topic_.c_str(), safe_rate);
}

// ===========================================================================
// openCamera —— 初始化 SDK → 打开设备 → 配置 → 开启采集流
// ===========================================================================
bool GxCameraComponent::openCamera()
{
  if (!initGxLib()) { return false; }
  if (!openGxDevice()) { return false; }
  if (!configureGxDevice()) {
    GXCloseDevice(device_);
    device_ = nullptr;
    return false;
  }
  return true;
}

// ===========================================================================
// initGxLib —— 初始化大恒 SDK 库（进程级，只做一次）
// ===========================================================================
bool GxCameraComponent::initGxLib()
{
  if (lib_initialized_) { return true; }
  const auto status = GXInitLib();
  if (!gx_ok(status)) {
    RCLCPP_ERROR(get_logger(), "GXInitLib failed: %d", static_cast<int>(status));
    return false;
  }
  lib_initialized_ = true;
  return true;
}

// ===========================================================================
// openGxDevice —— 枚举并打开相机设备
// ===========================================================================
bool GxCameraComponent::openGxDevice()
{
  uint32_t device_count = 0;
  auto status = GXUpdateDeviceList(&device_count, 1000);
  if (!gx_ok(status) || device_count == 0U) {
    RCLCPP_ERROR(get_logger(), "no GX camera found (count=%u)", device_count);
    return false;
  }

  // 打印所有已枚举设备的 index / model / SN
  {
    size_t buf_size = 0;
    status = GXGetAllDeviceBaseInfo(nullptr, &buf_size);
    if (gx_ok(status) && buf_size > 0) {
      std::vector<GX_DEVICE_BASE_INFO> info_list(buf_size / sizeof(GX_DEVICE_BASE_INFO));
      status = GXGetAllDeviceBaseInfo(info_list.data(), &buf_size);
      if (gx_ok(status)) {
        for (size_t i = 0; i < info_list.size(); ++i) {
          RCLCPP_INFO(get_logger(),
            "GX device[%zu]: model=%s, sn=%s",
            i + 1, info_list[i].szModelName, info_list[i].szSN);
        }
      }
    }
  }

  GX_OPEN_PARAM open_param{};
  open_param.accessMode = GX_ACCESS_EXCLUSIVE;

  const std::string idx_str =
    std::to_string(static_cast<int>(get_parameter("device_index").as_int()));

  if (device_sn_.empty() || device_sn_ == "auto") {
    open_param.openMode = GX_OPEN_INDEX;
    open_param.pszContent = const_cast<char *>(idx_str.c_str());
  } else {
    open_param.openMode = GX_OPEN_SN;
    open_param.pszContent = const_cast<char *>(device_sn_.c_str());
  }

  status = GXOpenDevice(&open_param, &device_);
  if (!gx_ok(status) && open_param.openMode == GX_OPEN_SN) {
    open_param.openMode = GX_OPEN_INDEX;
    open_param.pszContent = const_cast<char *>(idx_str.c_str());
    status = GXOpenDevice(&open_param, &device_);
  }
  if (!gx_ok(status)) {
    RCLCPP_ERROR(get_logger(), "GXOpenDevice failed: %d", static_cast<int>(status));
    return false;
  }

  bool color_supported = false;
  status = GXIsImplemented(device_, GX_ENUM_PIXEL_COLOR_FILTER, &color_supported);
  if (!gx_ok(status) || !color_supported) {
    RCLCPP_ERROR(get_logger(), "camera does not support color");
    return false;
  }
  return true;
}

// ===========================================================================
// configureGxDevice —— ROI → 曝光/增益 → 白平衡 → AAROI → 开始采集流
// ===========================================================================
bool GxCameraComponent::configureGxDevice()
{
  if (!setGxRoi() || !setGxExposureGain() || !setGxWhiteBalance() || !setGxAutoRoi()) {
    return false;
  }

  auto status = GXSetEnum(device_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
  if (!gx_ok(status)) { return false; }

  status = GXSetEnum(device_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
  if (!gx_ok(status)) { return false; }

  uint64_t buffer_num = 5;
  status = GXSetAcqusitionBufferNumber(device_, buffer_num);
  if (!gx_ok(status)) { return false; }

  status = GXStreamOn(device_);
  if (!gx_ok(status)) { return false; }
  stream_on_ = true;
  return true;
}

// ===========================================================================
// setGxRoi —— 设置图像 ROI
// ===========================================================================
bool GxCameraComponent::setGxRoi()
{
  GXSetInt(device_, GX_INT_WIDTH, 64);
  GXSetInt(device_, GX_INT_HEIGHT, 64);
  GXSetInt(device_, GX_INT_OFFSET_X, 0);
  GXSetInt(device_, GX_INT_OFFSET_Y, 0);
  auto status = GXSetInt(device_, GX_INT_WIDTH, width_);
  if (!gx_ok(status)) { return false; }
  status = GXSetInt(device_, GX_INT_HEIGHT, height_);
  return gx_ok(status);
}

// ===========================================================================
// setGxExposureGain —— 曝光时间 + 增益 + 触发源 + 灰度目标
// ===========================================================================
bool GxCameraComponent::setGxExposureGain()
{
  // 曝光
  if (auto_exposure_) {
    GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, auto_exposure_time_max_);
    GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, auto_exposure_time_min_);
  } else {
    GXSetEnum(device_, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
    GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    GXSetFloat(device_, GX_FLOAT_EXPOSURE_TIME, exposure_time_);
  }

  // 增益
  if (auto_gain_) {
    GXSetEnum(device_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
    GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MAX, auto_gain_max_);
    GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MIN, auto_gain_min_);
  } else {
    GXSetEnum(device_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
    GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
    GXSetFloat(device_, GX_FLOAT_GAIN, gain_);
  }

  // 灰度目标值
  GXSetInt(device_, GX_INT_GRAY_VALUE, gray_value_min_);
  GXSetInt(device_, GX_INT_GRAY_VALUE, gray_value_max_);
  return true;
}

// ===========================================================================
// setGxWhiteBalance —— 手动白平衡
// ===========================================================================
bool GxCameraComponent::setGxWhiteBalance()
{
  auto status = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);

  status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
  status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, balance_ratio_red_);

  status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
  status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, balance_ratio_blue_);

  status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
  status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, balance_ratio_green_);

  return gx_ok(status);
}

// ===========================================================================
// setGxAutoRoi —— 设置自动曝光/增益的测光 ROI
// ===========================================================================
bool GxCameraComponent::setGxAutoRoi()
{
  auto status = GXSetInt(device_, GX_INT_AAROI_WIDTH, aae_width_);
  status = GXSetInt(device_, GX_INT_AAROI_HEIGHT, aae_height_);
  status = GXSetInt(device_, GX_INT_AAROI_OFFSETX, aae_offset_x_);
  status = GXSetInt(device_, GX_INT_AAROI_OFFSETY, aae_offset_y_);
  return gx_ok(status);
}

// ===========================================================================
// publishFrame —— 采集一帧并发布 Image + CameraInfo
// ===========================================================================
void GxCameraComponent::publishFrame()
{
  if (device_ == nullptr) { return; }

  // ----- 出队一帧 -----
  PGX_FRAME_BUFFER frame_buffer = nullptr;
  auto status = GXDQBuf(device_, &frame_buffer, 1500);
  if (!gx_ok(status) || frame_buffer == nullptr) {
    if (frame_buffer != nullptr) { GXQBuf(device_, frame_buffer); }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "GXDQBuf failed");
    return;
  }

  if (frame_buffer->nStatus != GX_FRAME_STATUS_SUCCESS) {
    GXQBuf(device_, frame_buffer);
    return;
  }

  // ----- 预分配 Image 消息（unique_ptr，进程内零拷贝） -----
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.frame_id = frame_id_;
  msg->height = static_cast<uint32_t>(height_);
  msg->width = static_cast<uint32_t>(width_);
  msg->encoding = "bgr8";
  msg->is_bigendian = false;
  msg->step = static_cast<uint32_t>(width_ * 3);
  msg->data.resize(static_cast<size_t>(msg->step) * static_cast<size_t>(msg->height));

  // ----- Bayer BG Raw8 → BGR24 转换（直接写入 Image 的 data 缓冲区） -----
  DxRaw8toRGB24(
    static_cast<unsigned char *>(frame_buffer->pImgBuf),
    msg->data.data(),
    frame_buffer->nWidth,
    frame_buffer->nHeight,
    RAW2RGB_NEIGHBOUR,
    DX_PIXEL_COLOR_FILTER(BAYERBG),
    false);

  // DxRaw8toRGB24 输出 RGB，但 downstream 按 BGR 解码，交换 R/B 通道
  for (size_t i = 0; i < msg->data.size(); i += 3) {
    std::swap(msg->data[i], msg->data[i + 2]);
  }

  // 归还缓冲区
  GXQBuf(device_, frame_buffer);

  const auto stamp = now();
  msg->header.stamp = stamp;

  // ----- 构建 CameraInfo -----
  sensor_msgs::msg::CameraInfo cam_info;
  cam_info.header.stamp = stamp;
  cam_info.header.frame_id = frame_id_;
  cam_info.height = static_cast<uint32_t>(height_);
  cam_info.width = static_cast<uint32_t>(width_);
  cam_info.distortion_model = distortion_model_;
  cam_info.d = distortion_coefficients_;
  std::copy(camera_matrix_.begin(), camera_matrix_.end(), cam_info.k.begin());
  std::copy(rectification_matrix_.begin(), rectification_matrix_.end(), cam_info.r.begin());
  std::copy(projection_matrix_.begin(), projection_matrix_.end(), cam_info.p.begin());

  image_pub_->publish(std::move(msg));
  camera_info_pub_->publish(cam_info);
}

}  // namespace aim_camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(aim_camera_driver::GxCameraComponent)
