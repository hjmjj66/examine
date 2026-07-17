#include "aim_camera_driver/gx_camera_component.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>
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

GX_BALANCE_WHITE_AUTO_ENTRY parse_auto_balance_mode(const std::string & mode)
{
  std::string normalized = mode;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (normalized == "off") {
    return GX_BALANCE_WHITE_AUTO_OFF;
  }
  if (normalized == "once") {
    return GX_BALANCE_WHITE_AUTO_ONCE;
  }
  if (normalized == "continuous") {
    return GX_BALANCE_WHITE_AUTO_CONTINUOUS;
  }
  return GX_BALANCE_WHITE_AUTO_OFF;
}

bool get_bayer_filter(
  GX_DEV_HANDLE device,
  DX_PIXEL_COLOR_FILTER & bayer_filter)
{
  int64_t gx_filter = 0;
  if (!gx_ok(GXGetEnum(device, GX_ENUM_PIXEL_COLOR_FILTER, &gx_filter))) {
    return false;
  }

  switch (gx_filter) {
    case GX_COLOR_FILTER_BAYER_RG:
      bayer_filter = BAYERRG;
      return true;
    case GX_COLOR_FILTER_BAYER_GB:
      bayer_filter = BAYERGB;
      return true;
    case GX_COLOR_FILTER_BAYER_GR:
      bayer_filter = BAYERGR;
      return true;
    case GX_COLOR_FILTER_BAYER_BG:
      bayer_filter = BAYERBG;
      return true;
    default:
      return false;
  }
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

  // ---- 设备时间戳 ----
  declare_parameter<bool>("use_device_timestamp", false);
  declare_parameter<double>("timestamp_offset_sec", 0.0);
  declare_parameter<double>("timestamp_tick_frequency_hz_override", 0.0);
  declare_parameter<std::string>(
    "timestamp_status_topic", "/gx_camera/timestamp_status");
  declare_parameter<double>("timestamp_latch_period_sec", 1.0);
  declare_parameter<double>("timestamp_latch_alpha", 0.1);
  declare_parameter<double>("timestamp_latch_max_correction_sec", 0.02);
  declare_parameter<double>("timestamp_frequency_estimation_interval_sec", 0.05);

  // ---- 白平衡 ----
  declare_parameter<std::string>("auto_balance_mode", "once");
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
  image_topic_name_ = image_topic;
  camera_info_topic_ = get_parameter("camera_info_topic").as_string();
  const auto publish_rate = get_parameter("publish_rate").as_double();
  width_ = get_parameter("width").as_int();
  height_ = get_parameter("height").as_int();

  device_sn_ = get_parameter("device_sn").as_string();
  exposure_time_ = get_parameter("exposure_time").as_double();
  auto_exposure_time_max_ = get_parameter("auto_exposure_time_max").as_double();
  auto_exposure_time_min_ = get_parameter("auto_exposure_time_min").as_double();
  auto_exposure_ = (get_parameter("auto_exposure").as_int() != 0);
  use_device_timestamp_ = get_parameter("use_device_timestamp").as_bool();
  timestamp_offset_sec_ = get_parameter("timestamp_offset_sec").as_double();
  timestamp_tick_frequency_override_hz_ = get_parameter(
    "timestamp_tick_frequency_hz_override").as_double();
  timestamp_status_topic_ = get_parameter("timestamp_status_topic").as_string();
  timestamp_latch_period_sec_ =
    std::max(0.1, get_parameter("timestamp_latch_period_sec").as_double());
  timestamp_latch_alpha_ = get_parameter("timestamp_latch_alpha").as_double();
  timestamp_latch_max_correction_sec_ =
    get_parameter("timestamp_latch_max_correction_sec").as_double();
  timestamp_frequency_estimation_interval_sec_ = std::max(
    0.005, get_parameter("timestamp_frequency_estimation_interval_sec").as_double());
  timestamp_mapper_ = GxTimestampMapper({
      0.0,
      timestamp_offset_sec_,
      timestamp_latch_alpha_,
      timestamp_latch_max_correction_sec_});
  gain_ = get_parameter("gain").as_double();
  auto_gain_max_ = get_parameter("auto_gain_max").as_double();
  auto_gain_min_ = get_parameter("auto_gain_min").as_double();
  auto_gain_ = (get_parameter("auto_gain").as_int() != 0);
  auto_balance_mode_ = get_parameter("auto_balance_mode").as_string();
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
  timestamp_status_pub_ = create_publisher<aim_msgs::msg::GxTimestampStatus>(
    timestamp_status_topic_, rclcpp::SensorDataQoS());

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

  if (use_device_timestamp_) {
    // The latch command performs a remote-device feature transaction.  Keep it
    // in a separate re-entrant callback group so a slow latch cannot delay the
    // 100 Hz frame callback in the multi-threaded component container.
    timestamp_latch_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    timestamp_latch_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timestamp_latch_period_sec_)),
      [this]() { maybeRefreshTimestampLatch(); },
      timestamp_latch_callback_group_);
  }

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

  if (!get_bayer_filter(device_, bayer_filter_)) {
    RCLCPP_ERROR(get_logger(), "failed to read a supported Bayer color filter");
    return false;
  }

  RCLCPP_INFO(get_logger(), "camera Bayer color filter: %d", static_cast<int>(bayer_filter_));
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
  if (use_device_timestamp_ && !initializeTimestampMapper()) {
    RCLCPP_WARN(
      get_logger(),
      "GX device timestamp requested but timestamp mapping is unavailable; "
      "falling back to host receive time");
  }
  return true;
}

bool GxCameraComponent::initializeTimestampMapper()
{
  {
    std::lock_guard<std::mutex> lock(timestamp_mutex_);
    timestamp_mapper_.reset();
    timestamp_tick_frequency_hz_ = 0.0;
    timestamp_frequency_estimated_ = false;
    timestamp_frequency_override_used_ = false;
    timestamp_mapping_error_code_ = 0;
    timestamp_mapping_error_stage_.clear();
  }

  // The feature-query calls are deliberately diagnostic rather than a hard
  // gate.  A few GX firmware versions expose the timestamp features but
  // return an incomplete feature table; the direct reads below are still the
  // authoritative test.
  bool frequency_implemented = false;
  auto status = GXIsImplemented(
    device_, GX_INT_TIMESTAMP_TICK_FREQUENCY, &frequency_implemented);
  if (!gx_ok(status)) {
    recordTimestampMappingFailure("query_tick_frequency_implemented", status);
  } else if (!frequency_implemented) {
    recordTimestampMappingFailure(
      "tick_frequency_not_implemented", GX_STATUS_NOT_IMPLEMENTED);
  }

  bool frequency_readable = false;
  status = GXIsReadable(device_, GX_INT_TIMESTAMP_TICK_FREQUENCY, &frequency_readable);
  if (!gx_ok(status)) {
    recordTimestampMappingFailure("query_tick_frequency_readable", status);
  } else if (!frequency_readable) {
    recordTimestampMappingFailure("tick_frequency_not_readable", GX_STATUS_INVALID_ACCESS);
  }

  bool latch_implemented = false;
  status = GXIsImplemented(device_, GX_COMMAND_TIMESTAMP_LATCH, &latch_implemented);
  if (!gx_ok(status)) {
    recordTimestampMappingFailure("query_timestamp_latch_implemented", status);
  } else if (!latch_implemented) {
    recordTimestampMappingFailure(
      "timestamp_latch_not_implemented", GX_STATUS_NOT_IMPLEMENTED);
  }

  bool latch_value_implemented = false;
  status = GXIsImplemented(device_, GX_INT_TIMESTAMP_LATCH_VALUE, &latch_value_implemented);
  if (!gx_ok(status)) {
    recordTimestampMappingFailure("query_timestamp_latch_value_implemented", status);
  } else if (!latch_value_implemented) {
    recordTimestampMappingFailure(
      "timestamp_latch_value_not_implemented", GX_STATUS_NOT_IMPLEMENTED);
  }

  bool latch_value_readable = false;
  status = GXIsReadable(device_, GX_INT_TIMESTAMP_LATCH_VALUE, &latch_value_readable);
  if (!gx_ok(status)) {
    recordTimestampMappingFailure("query_timestamp_latch_value_readable", status);
  } else if (!latch_value_readable) {
    recordTimestampMappingFailure(
      "timestamp_latch_value_not_readable", GX_STATUS_INVALID_ACCESS);
  }

  double tick_frequency_hz = 0.0;
  int64_t tick_frequency = 0;
  status = GXGetInt(device_, GX_INT_TIMESTAMP_TICK_FREQUENCY, &tick_frequency);
  if (gx_ok(status) && tick_frequency > 0) {
    tick_frequency_hz = static_cast<double>(tick_frequency);
  } else {
    recordTimestampMappingFailure("read_tick_frequency", status);
    if (gx_ok(status) && tick_frequency <= 0) {
      recordTimestampMappingFailure("read_tick_frequency_invalid", GX_STATUS_OUT_OF_RANGE);
    }
  }

  if (tick_frequency_hz <= 0.0 && timestamp_tick_frequency_override_hz_ > 0.0) {
    tick_frequency_hz = timestamp_tick_frequency_override_hz_;
    timestamp_frequency_override_used_ = true;
    RCLCPP_WARN(
      get_logger(),
      "GX timestamp tick frequency read failed; using configured override %.3fHz",
      tick_frequency_hz);
  }

  if (tick_frequency_hz <= 0.0) {
    // Some GX devices expose the image timestamp and latch command but not
    // GX_INT_TIMESTAMP_TICK_FREQUENCY.  Estimate the counter rate from two
    // host-midpoint/device-latch pairs instead of assuming nanoseconds.
    std::uint64_t first_tick = 0U;
    std::uint64_t second_tick = 0U;
    rclcpp::Time first_host(0, 0, RCL_ROS_TIME);
    rclcpp::Time second_host(0, 0, RCL_ROS_TIME);
    if (!readTimestampLatch(first_tick, first_host)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(
      timestamp_frequency_estimation_interval_sec_));
    if (!readTimestampLatch(second_tick, second_host)) {
      return false;
    }

    const double host_delta_sec = (second_host - first_host).seconds();
    const std::uint64_t tick_delta = second_tick > first_tick ?
      second_tick - first_tick : 0U;
    const double estimated_frequency = host_delta_sec > 0.0 ?
      static_cast<double>(tick_delta) / host_delta_sec : 0.0;
    if (second_tick <= first_tick || !std::isfinite(estimated_frequency) ||
      estimated_frequency <= 0.0 || estimated_frequency > 1.0e12)
    {
      recordTimestampMappingFailure(
        "estimate_tick_frequency_from_latch", GX_STATUS_ERROR);
      RCLCPP_WARN(
        get_logger(),
        "GX timestamp latch frequency estimate invalid: tick_delta=%llu host_delta=%.9fs",
        static_cast<unsigned long long>(tick_delta), host_delta_sec);
      return false;
    }
    tick_frequency_hz = estimated_frequency;
    timestamp_frequency_estimated_ = true;
    RCLCPP_WARN(
      get_logger(),
      "GX timestamp tick frequency unavailable; estimated %.3fHz from latch pairs",
      tick_frequency_hz);
  }

  {
    std::lock_guard<std::mutex> lock(timestamp_mutex_);
    timestamp_tick_frequency_hz_ = tick_frequency_hz;
    timestamp_mapper_.setTickFrequency(tick_frequency_hz);
  }

  if (!refreshTimestampLatch()) {
    return false;
  }

  clearTimestampMappingFailure();
  RCLCPP_INFO(
    get_logger(),
    "GX device timestamp enabled: tick_frequency=%.3fHz%s%s fixed_offset=%.6fs",
    tick_frequency_hz,
    timestamp_frequency_estimated_ ? " (estimated)" : "",
    timestamp_frequency_override_used_ ? " (override)" : "",
    timestamp_offset_sec_);
  return true;
}

bool GxCameraComponent::readTimestampLatch(
  std::uint64_t & device_tick, rclcpp::Time & host_time)
{
  if (device_ == nullptr) {
    recordTimestampMappingFailure("read_timestamp_latch_invalid_device", GX_STATUS_INVALID_HANDLE);
    return false;
  }

  const auto before = now();
  const auto status = GXSendCommand(device_, GX_COMMAND_TIMESTAMP_LATCH);
  if (!gx_ok(status)) {
    recordTimestampMappingFailure("send_timestamp_latch", status);
    return false;
  }

  int64_t latched_tick = 0;
  const auto read_status = GXGetInt(
    device_, GX_INT_TIMESTAMP_LATCH_VALUE, &latched_tick);
  if (!gx_ok(read_status) || latched_tick < 0) {
    recordTimestampMappingFailure(
      "read_timestamp_latch_value", gx_ok(read_status) ? GX_STATUS_OUT_OF_RANGE : read_status);
    return false;
  }

  const auto after = now();
  const auto elapsed = after - before;
  host_time = before + rclcpp::Duration::from_nanoseconds(elapsed.nanoseconds() / 2);
  device_tick = static_cast<std::uint64_t>(latched_tick);
  return true;
}

bool GxCameraComponent::refreshTimestampLatch()
{
  double tick_frequency_hz = 0.0;
  {
    std::lock_guard<std::mutex> lock(timestamp_mutex_);
    tick_frequency_hz = timestamp_tick_frequency_hz_;
  }
  if (device_ == nullptr || tick_frequency_hz <= 0.0) {
    return false;
  }

  std::uint64_t latched_tick = 0U;
  rclcpp::Time midpoint(0, 0, RCL_ROS_TIME);
  if (!readTimestampLatch(latched_tick, midpoint)) {
    return false;
  }

  bool updated = false;
  double correction_sec = 0.0;
  {
    std::lock_guard<std::mutex> lock(timestamp_mutex_);
    updated = timestamp_mapper_.valid() ?
      timestamp_mapper_.updateLatch(latched_tick, midpoint) :
      timestamp_mapper_.initialize(latched_tick, midpoint);
    correction_sec = timestamp_mapper_.estimatedLatchCorrectionSec();
    if (updated) {
      last_timestamp_latch_tick_ = latched_tick;
      last_timestamp_latch_time_ = midpoint;
    }
  }
  if (updated) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "GX timestamp latch: tick=%lld correction=%.6fms",
      static_cast<long long>(latched_tick), correction_sec * 1000.0);
  } else {
    recordTimestampMappingFailure("update_timestamp_mapping", GX_STATUS_ERROR);
  }
  return updated;
}

void GxCameraComponent::maybeRefreshTimestampLatch()
{
  if (!use_device_timestamp_) {
    return;
  }

  const auto current_time = now();
  {
    std::lock_guard<std::mutex> lock(timestamp_mutex_);
    if (!timestamp_mapper_.valid() ||
      (current_time - last_timestamp_latch_time_).seconds() < timestamp_latch_period_sec_)
    {
      return;
    }
  }

  if (!refreshTimestampLatch()) {
    return;
  }
  clearTimestampMappingFailure();
}

void GxCameraComponent::recordTimestampMappingFailure(
  const char * stage, GX_STATUS status)
{
  {
    std::lock_guard<std::mutex> lock(timestamp_mutex_);
    timestamp_mapping_error_code_ = static_cast<int32_t>(status);
    timestamp_mapping_error_stage_ = stage == nullptr ? "unknown" : stage;
  }
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "GX timestamp mapping failure at %s: status=%d",
    stage == nullptr ? "unknown" : stage, static_cast<int>(status));
}

void GxCameraComponent::clearTimestampMappingFailure()
{
  std::lock_guard<std::mutex> lock(timestamp_mutex_);
  timestamp_mapping_error_code_ = 0;
  timestamp_mapping_error_stage_.clear();
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
// setGxWhiteBalance -- auto/manual white balance
// ===========================================================================
bool GxCameraComponent::setGxWhiteBalance()
{
  const auto auto_balance = parse_auto_balance_mode(auto_balance_mode_);
  if (auto_balance != GX_BALANCE_WHITE_AUTO_OFF) {
    return gx_ok(GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, auto_balance));
  }

  auto status = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
  if (!gx_ok(status)) { return false; }

  status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
  if (!gx_ok(status)) { return false; }
  status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, balance_ratio_red_);
  if (!gx_ok(status)) { return false; }

  status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
  if (!gx_ok(status)) { return false; }
  status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, balance_ratio_blue_);
  if (!gx_ok(status)) { return false; }

  status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
  if (!gx_ok(status)) { return false; }
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

  // Capture host receive time immediately after dequeue.  If the device
  // mapping is unavailable this is the correct fallback boundary; image
  // conversion and DDS publication happen after this point.
  const auto receive_stamp = now();

  // ----- 预分配 Image 消息（unique_ptr，进程内零拷贝） -----
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.frame_id = frame_id_;
  msg->height = static_cast<uint32_t>(height_);
  msg->width = static_cast<uint32_t>(width_);
  msg->encoding = "bgr8";
  msg->is_bigendian = false;
  msg->step = static_cast<uint32_t>(width_ * 3);
  msg->data.resize(static_cast<size_t>(msg->step) * static_cast<size_t>(msg->height));

  // ----- 相机实际 Bayer 排列 → BGR24 转换（直接写入 Image 的 data 缓冲区） -----
  DxRaw8toRGB24(
    static_cast<unsigned char *>(frame_buffer->pImgBuf),
    msg->data.data(),
    frame_buffer->nWidth,
    frame_buffer->nHeight,
    RAW2RGB_NEIGHBOUR,
    bayer_filter_,
    false);

  const auto device_timestamp = frame_buffer->nTimestamp;

  // DxRaw8toRGB24 输出 RGB，但 downstream 按 BGR 解码，交换 R/B 通道
  // for (size_t i = 0; i < msg->data.size(); i += 3) {
  //   std::swap(msg->data[i], msg->data[i + 2]);
  // }

  // 归还缓冲区
  GXQBuf(device_, frame_buffer);

  rclcpp::Time stamp = receive_stamp;
  bool mapping_valid = false;
  std::uint64_t latch_tick = 0U;
  double tick_frequency_hz = 0.0;
  double latch_correction_sec = 0.0;
  double fixed_offset_sec = timestamp_offset_sec_;
  bool frequency_estimated = false;
  bool frequency_override = false;
  int32_t mapping_error_code = 0;
  std::string mapping_error_stage;
  {
    std::lock_guard<std::mutex> lock(timestamp_mutex_);
    if (use_device_timestamp_) {
      stamp = timestamp_mapper_.map(device_timestamp, receive_stamp);
    }
    mapping_valid = timestamp_mapper_.valid();
    latch_tick = last_timestamp_latch_tick_;
    tick_frequency_hz = timestamp_mapper_.tickFrequencyHz();
    latch_correction_sec = timestamp_mapper_.estimatedLatchCorrectionSec();
    fixed_offset_sec = timestamp_mapper_.fixedOffsetSec();
    frequency_estimated = timestamp_frequency_estimated_;
    frequency_override = timestamp_frequency_override_used_;
    mapping_error_code = timestamp_mapping_error_code_;
    mapping_error_stage = timestamp_mapping_error_stage_;
  }
  msg->header.stamp = stamp;

  if (timestamp_status_pub_) {
    aim_msgs::msg::GxTimestampStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = frame_id_;
    status.camera_name = image_topic_name_;
    status.enabled = use_device_timestamp_ && mapping_valid;
    status.mapping_valid = mapping_valid;
    status.device_tick = device_timestamp;
    status.latch_tick = latch_tick;
    status.tick_frequency_hz = tick_frequency_hz;
    status.tick_frequency_estimated = frequency_estimated;
    status.tick_frequency_override = frequency_override;
    status.mapped_stamp_sec = stamp.seconds();
    status.host_receive_stamp_sec = receive_stamp.seconds();
    status.latch_correction_sec = latch_correction_sec;
    status.fixed_offset_sec = fixed_offset_sec;
    status.mapping_error_code = mapping_error_code;
    status.mapping_error_stage = mapping_error_stage;
    timestamp_status_pub_->publish(std::move(status));
  }

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
