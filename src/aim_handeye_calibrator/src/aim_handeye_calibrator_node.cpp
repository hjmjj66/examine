#include "aim_handeye_calibrator/aim_handeye_calibrator_node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge.hpp>
#else
  #include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace aim_handeye_calibrator
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double rad2deg(double rad)
{
  return rad * 180.0 / kPi;
}

cv::Mat makeIdentityTransform()
{
  return cv::Mat::eye(4, 4, CV_64F);
}

cv::Mat makeTransform(const cv::Mat & rotation, const cv::Mat & translation)
{
  cv::Mat transform = makeIdentityTransform();
  rotation.copyTo(transform(cv::Rect(0, 0, 3, 3)));
  translation.copyTo(transform(cv::Rect(3, 0, 1, 3)));
  return transform;
}

cv::Mat invertTransform(const cv::Mat & transform)
{
  cv::Mat rotation = transform(cv::Rect(0, 0, 3, 3)).clone();
  cv::Mat translation = transform(cv::Rect(3, 0, 1, 3)).clone();
  cv::Mat rotation_inv = rotation.t();
  cv::Mat translation_inv = -rotation_inv * translation;
  return makeTransform(rotation_inv, translation_inv);
}

double rotationAngleDeg(const cv::Mat & rotation)
{
  cv::Mat rvec;
  cv::Rodrigues(rotation, rvec);
  return rad2deg(cv::norm(rvec));
}

cv::Mat rotationFromRollPitchYaw(double roll_rad, double pitch_rad, double yaw_rad)
{
  const double cr = std::cos(roll_rad);
  const double sr = std::sin(roll_rad);
  const double cp = std::cos(pitch_rad);
  const double sp = std::sin(pitch_rad);
  const double cy = std::cos(yaw_rad);
  const double sy = std::sin(yaw_rad);

  cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
  rotation.at<double>(0, 0) = cy * cp;
  rotation.at<double>(0, 1) = cy * sp * sr - sy * cr;
  rotation.at<double>(0, 2) = cy * sp * cr + sy * sr;
  rotation.at<double>(1, 0) = sy * cp;
  rotation.at<double>(1, 1) = sy * sp * sr + cy * cr;
  rotation.at<double>(1, 2) = sy * sp * cr - cy * sr;
  rotation.at<double>(2, 0) = -sp;
  rotation.at<double>(2, 1) = cp * sr;
  rotation.at<double>(2, 2) = cp * cr;
  return rotation;
}

std::array<double, 3> rollPitchYawFromRotation(const cv::Mat & rotation)
{
  const double yaw = std::atan2(rotation.at<double>(1, 0), rotation.at<double>(0, 0));
  const double pitch = std::atan2(
    -rotation.at<double>(2, 0),
    std::hypot(rotation.at<double>(2, 1), rotation.at<double>(2, 2)));
  const double roll = std::atan2(rotation.at<double>(2, 1), rotation.at<double>(2, 2));
  return {roll, pitch, yaw};
}

std::string toLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

cv::HandEyeCalibrationMethod handeyeMethodFromString(const std::string & method_name)
{
  const auto method = toLower(method_name);
  if (method == "park") {
    return cv::CALIB_HAND_EYE_PARK;
  }
  if (method == "horaud") {
    return cv::CALIB_HAND_EYE_HORAUD;
  }
  if (method == "andreff") {
    return cv::CALIB_HAND_EYE_ANDREFF;
  }
  if (method == "daniilidis") {
    return cv::CALIB_HAND_EYE_DANIILIDIS;
  }
  return cv::CALIB_HAND_EYE_TSAI;
}

}  // namespace

AimHandeyeCalibratorNode::AimHandeyeCalibratorNode(const rclcpp::NodeOptions & options)
: Node("aim_handeye_calibrator_node", options)
{
  declare_parameter<std::string>("image_topic", "/gx_camera_0/image_raw");
  declare_parameter<std::string>("camera_info_topic", "/gx_camera_0/camera_info");
  declare_parameter<std::string>("gimbal_state_topic", "/gimbal/state");
  declare_parameter<std::string>("debug_image_topic", "/aim_handeye/debug_image");
  declare_parameter<std::string>("parent_frame_id", "gimbal_barrel");
  declare_parameter<std::string>("child_frame_id", "gx_camera");
  declare_parameter<std::string>("output_tf_section", "barrel_to_camera");
  declare_parameter<std::string>("handeye_method", "tsai");
  declare_parameter<std::string>("output_file", "");
  declare_parameter<bool>("use_camera_info_topic", false);
  declare_parameter<bool>("angles_in_degree", true);
  declare_parameter<int>("board_cols", 11);
  declare_parameter<int>("board_rows", 8);
  declare_parameter<int>("min_samples", 8);
  declare_parameter<double>("square_size_m", 0.030);
  declare_parameter<double>("yaw_sign", 1.0);
  declare_parameter<double>("pitch_sign", -1.0);
  declare_parameter<double>("min_angle_delta_deg", 2.0);
  declare_parameter<bool>("use_pitch", true);
  declare_parameter<std::vector<double>>("camera_matrix", std::vector<double>{});
  declare_parameter<std::vector<double>>("distortion_coefficients", std::vector<double>{});

  image_topic_ = get_parameter("image_topic").as_string();
  camera_info_topic_ = get_parameter("camera_info_topic").as_string();
  gimbal_state_topic_ = get_parameter("gimbal_state_topic").as_string();
  debug_image_topic_ = get_parameter("debug_image_topic").as_string();
  parent_frame_id_ = get_parameter("parent_frame_id").as_string();
  child_frame_id_ = get_parameter("child_frame_id").as_string();
  output_tf_section_ = get_parameter("output_tf_section").as_string();
  output_file_ = get_parameter("output_file").as_string();
  board_cols_ = get_parameter("board_cols").as_int();
  board_rows_ = get_parameter("board_rows").as_int();
  square_size_m_ = get_parameter("square_size_m").as_double();
  yaw_sign_ = get_parameter("yaw_sign").as_double();
  pitch_sign_ = get_parameter("pitch_sign").as_double();
  min_samples_ = get_parameter("min_samples").as_int();
  angles_in_degree_ = get_parameter("angles_in_degree").as_bool();
  use_camera_info_topic_ = get_parameter("use_camera_info_topic").as_bool();
  use_pitch_ = get_parameter("use_pitch").as_bool();
  min_angle_delta_rad_ = get_parameter("min_angle_delta_deg").as_double() * kPi / 180.0;

  if (board_cols_ < 2 || board_rows_ < 2) {
    throw std::runtime_error("board_cols and board_rows must be inner-corner counts >= 2");
  }
  if (square_size_m_ <= 0.0) {
    throw std::runtime_error("square_size_m must be positive");
  }
  if (min_samples_ < 3) {
    throw std::runtime_error("min_samples must be at least 3");
  }

  const auto camera_matrix_param = get_parameter("camera_matrix").as_double_array();
  if (camera_matrix_param.size() == 9U) {
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (std::size_t i = 0; i < camera_matrix_param.size(); ++i) {
      camera_matrix_.at<double>(static_cast<int>(i / 3U), static_cast<int>(i % 3U)) =
        camera_matrix_param[i];
    }
  }

  const auto distortion_param = get_parameter("distortion_coefficients").as_double_array();
  if (!distortion_param.empty()) {
    distortion_coefficients_ = cv::Mat(
      static_cast<int>(distortion_param.size()), 1, CV_64F);
    for (std::size_t i = 0; i < distortion_param.size(); ++i) {
      distortion_coefficients_.at<double>(static_cast<int>(i), 0) = distortion_param[i];
    }
  }

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, rclcpp::SensorDataQoS().keep_last(1),
    [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(msg); });

  if (use_camera_info_topic_) {
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { onCameraInfo(msg); });
  }

  gimbal_state_sub_ = create_subscription<sentry_gimbal::msg::GimbalAngles>(
    gimbal_state_topic_, rclcpp::SensorDataQoS().keep_last(20),
    [this](const sentry_gimbal::msg::GimbalAngles::ConstSharedPtr msg) { onGimbalState(msg); });

  debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
    debug_image_topic_, rclcpp::SensorDataQoS());

  capture_service_ = create_service<std_srvs::srv::Trigger>(
    "~/capture_sample",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      handleCapture(request, response);
    });
  solve_service_ = create_service<std_srvs::srv::Trigger>(
    "~/solve",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      handleSolve(request, response);
    });
  clear_service_ = create_service<std_srvs::srv::Trigger>(
    "~/clear_samples",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      handleClear(request, response);
    });

  RCLCPP_INFO(
    get_logger(),
    "aim handeye calibrator ready: image_topic='%s', gimbal_state_topic='%s', board=%dx%d inner corners, square=%.4f m",
    image_topic_.c_str(),
    gimbal_state_topic_.c_str(),
    board_cols_,
    board_rows_,
    square_size_m_);
}

void AimHandeyeCalibratorNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_image_ = msg;
}

void AimHandeyeCalibratorNode::onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  if (msg->k.size() != 9U) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "camera info has invalid K size");
    return;
  }

  cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F);
  for (std::size_t i = 0; i < msg->k.size(); ++i) {
    camera_matrix.at<double>(static_cast<int>(i / 3U), static_cast<int>(i % 3U)) = msg->k[i];
  }

  cv::Mat distortion_coeffs(static_cast<int>(msg->d.size()), 1, CV_64F);
  for (std::size_t i = 0; i < msg->d.size(); ++i) {
    distortion_coeffs.at<double>(static_cast<int>(i), 0) = msg->d[i];
  }

  std::lock_guard<std::mutex> lock(mutex_);
  camera_matrix_ = camera_matrix;
  distortion_coefficients_ = distortion_coeffs;
}

void AimHandeyeCalibratorNode::onGimbalState(const sentry_gimbal::msg::GimbalAngles::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_gimbal_state_ = *msg;
}

void AimHandeyeCalibratorNode::handleCapture(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  try {
    sensor_msgs::msg::Image::ConstSharedPtr image_msg;
    std::optional<sentry_gimbal::msg::GimbalAngles> gimbal_state;
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    std::size_t sample_count = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      image_msg = latest_image_;
      gimbal_state = latest_gimbal_state_;
      camera_matrix = camera_matrix_.clone();
      distortion_coefficients = distortion_coefficients_.clone();
      sample_count = samples_.size();
    }

    if (!image_msg) {
      response->success = false;
      response->message = "no image received yet";
      return;
    }
    if (!gimbal_state.has_value()) {
      response->success = false;
      response->message = "no gimbal state received yet";
      return;
    }
    if (camera_matrix.empty()) {
      response->success = false;
      response->message =
        "camera intrinsics unavailable; provide camera_matrix/distortion_coefficients or enable camera_info";
      return;
    }

    const double yaw_raw = yaw_sign_ * static_cast<double>(gimbal_state->yaw);
    const double pitch_raw = pitch_sign_ * static_cast<double>(gimbal_state->pitch);
    const double yaw_rad = angles_in_degree_ ? yaw_raw * kPi / 180.0 : yaw_raw;
    const double pitch_rad = use_pitch_ ?
      (angles_in_degree_ ? pitch_raw * kPi / 180.0 : pitch_raw) : 0.0;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!samples_.empty()) {
        const auto & last = samples_.back();
        const double delta = use_pitch_ ?
          std::hypot(yaw_rad - last.yaw_rad, pitch_rad - last.pitch_rad) :
          std::abs(yaw_rad - last.yaw_rad);
        if (delta < min_angle_delta_rad_) {
          std::ostringstream oss;
          oss << "pose change too small; current sample count=" << sample_count
              << ", need at least " << rad2deg(min_angle_delta_rad_)
              << " deg away from previous pose";
          response->success = false;
          response->message = oss.str();
          return;
        }
      }
    }

    cv::Mat debug_image;
    cv::Mat rotation_target_to_optical;
    cv::Mat translation_target_to_optical;
    double reprojection_error_px = 0.0;
    if (!detectTargetPose(
        image_msg, camera_matrix, distortion_coefficients, debug_image,
        rotation_target_to_optical, translation_target_to_optical,
        reprojection_error_px))
    {
      response->success = false;
      response->message = "chessboard corners not found or pose estimation failed";
      return;
    }

    if (debug_image_pub_->get_subscription_count() > 0U) {
      auto debug_msg = cv_bridge::CvImage(
        image_msg->header, sensor_msgs::image_encodings::BGR8, debug_image).toImageMsg();
      debug_image_pub_->publish(*debug_msg);
    }

    Sample sample;
    sample.stamp = image_msg->header.stamp;
    sample.yaw_rad = yaw_rad;
    sample.pitch_rad = pitch_rad;
    sample.rotation_target_to_optical = rotation_target_to_optical;
    sample.translation_target_to_optical = translation_target_to_optical;
    sample.reprojection_error_px = reprojection_error_px;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      samples_.push_back(sample);
      sample_count = samples_.size();
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "captured sample #" << sample_count
        << ", corners=" << board_cols_ * board_rows_
        << ", yaw=" << yaw_rad << " rad"
        << ", pitch=" << pitch_rad << " rad"
        << ", reproj=" << reprojection_error_px << " px";
    response->success = true;
    response->message = oss.str();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } catch (const cv::Exception & e) {
    response->success = false;
    response->message = std::string("OpenCV error in capture_sample: ") + e.what();
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  } catch (const std::exception & e) {
    response->success = false;
    response->message = std::string("error in capture_sample: ") + e.what();
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }
}

void AimHandeyeCalibratorNode::handleSolve(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  try {
    SolveResult result;
    if (!solveCalibration(result)) {
      response->success = false;
      response->message = "calibration failed; check sample count and pose diversity";
      return;
    }

    const auto yaml = formatTransformYaml(result);
    RCLCPP_INFO(get_logger(), "\n%s", yaml.c_str());

    if (!output_file_.empty() && !saveResultToFile(yaml)) {
      response->success = false;
      response->message = "calibration solved but failed to save output file";
      return;
    }

    response->success = true;
    response->message = yaml;
  } catch (const cv::Exception & e) {
    response->success = false;
    response->message = std::string("OpenCV error in solve: ") + e.what();
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  } catch (const std::exception & e) {
    response->success = false;
    response->message = std::string("error in solve: ") + e.what();
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }
}

void AimHandeyeCalibratorNode::handleClear(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
  response->success = true;
  response->message = "cleared all captured samples";
}

bool AimHandeyeCalibratorNode::detectTargetPose(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion_coefficients,
  cv::Mat & debug_image,
  cv::Mat & rotation_target_to_optical,
  cv::Mat & translation_target_to_optical,
  double & reprojection_error_px)
{
  cv_bridge::CvImageConstPtr cv_image;
  try {
    cv_image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (const std::exception &) {
    RCLCPP_WARN(get_logger(), "failed to decode image");
    return false;
  }

  cv::Mat gray;
  cv::cvtColor(cv_image->image, gray, cv::COLOR_BGR2GRAY);

  const cv::Size board_size(board_cols_, board_rows_);
  std::vector<cv::Point2f> corners;
  const bool found = cv::findChessboardCorners(
    gray, board_size, corners,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
  if (!found || corners.size() != static_cast<std::size_t>(board_cols_ * board_rows_)) {
    return false;
  }

  cv::cornerSubPix(
    gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));

  cv::Mat rvec;
  cv::Mat tvec;
  try {
    const bool solved = cv::solvePnP(
      boardObjectPoints(), corners, camera_matrix, distortion_coefficients, rvec, tvec);
    if (!solved) {
      return false;
    }
  } catch (const cv::Exception & e) {
    throw std::runtime_error(std::string("solvePnP failed: ") + e.what());
  }

  cv::Mat rotation;
  try {
    cv::Rodrigues(rvec, rotation);
  } catch (const cv::Exception & e) {
    throw std::runtime_error(std::string("Rodrigues failed: ") + e.what());
  }
  rotation_target_to_optical = toDoubleMatrix(rotation);
  translation_target_to_optical = tvec;
  try {
    reprojection_error_px = computeReprojectionError(
      corners,
      camera_matrix,
      distortion_coefficients,
      rotation_target_to_optical,
      translation_target_to_optical);
  } catch (const cv::Exception & e) {
    reprojection_error_px = -1.0;
    RCLCPP_WARN(
      get_logger(),
      "computeReprojectionError failed, continue without reprojection error: %s",
      e.what());
  }

  debug_image = cv_image->image.clone();
  cv::drawChessboardCorners(debug_image, board_size, corners, true);
  try {
    cv::drawFrameAxes(
      debug_image, camera_matrix, distortion_coefficients,
      rvec,
      tvec,
      static_cast<float>(square_size_m_ * std::min(board_cols_, board_rows_)));
  } catch (const cv::Exception & e) {
    RCLCPP_WARN(
      get_logger(),
      "drawFrameAxes failed, continue without axes overlay: %s",
      e.what());
  }

  return true;
}

bool AimHandeyeCalibratorNode::solveCalibration(SolveResult & result) const
{
  std::vector<Sample> samples;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    samples = samples_;
  }

  if (samples.size() < static_cast<std::size_t>(min_samples_)) {
    RCLCPP_WARN(
      get_logger(),
      "not enough samples: have %zu, need at least %d",
      samples.size(),
      min_samples_);
    return false;
  }

  std::vector<cv::Mat> rotations_gripper_to_base;
  std::vector<cv::Mat> translations_gripper_to_base;
  std::vector<cv::Mat> rotations_target_to_optical;
  std::vector<cv::Mat> translations_target_to_optical;
  rotations_gripper_to_base.reserve(samples.size());
  translations_gripper_to_base.reserve(samples.size());
  rotations_target_to_optical.reserve(samples.size());
  translations_target_to_optical.reserve(samples.size());

  for (const auto & sample : samples) {
    rotations_gripper_to_base.push_back(buildBaseToGimbalRotation(sample.yaw_rad, sample.pitch_rad));
    translations_gripper_to_base.push_back((cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0));
    rotations_target_to_optical.push_back(sample.rotation_target_to_optical);
    translations_target_to_optical.push_back(sample.translation_target_to_optical);
  }

  cv::Mat rotation_camera_to_gimbal_optical;
  cv::Mat translation_camera_to_gimbal_optical;
  try {
    cv::calibrateHandEye(
      rotations_gripper_to_base,
      translations_gripper_to_base,
      rotations_target_to_optical,
      translations_target_to_optical,
      rotation_camera_to_gimbal_optical,
      translation_camera_to_gimbal_optical,
      handeyeMethodFromString(get_parameter("handeye_method").as_string()));
  } catch (const cv::Exception & e) {
    RCLCPP_ERROR(get_logger(), "calibrateHandEye failed: %s", e.what());
    return false;
  }

  if (rotation_camera_to_gimbal_optical.empty() || translation_camera_to_gimbal_optical.empty()) {
    return false;
  }

  rotation_camera_to_gimbal_optical = toDoubleMatrix(rotation_camera_to_gimbal_optical);
  translation_camera_to_gimbal_optical = toDoubleMatrix(translation_camera_to_gimbal_optical);

  const cv::Mat rotation_gimbal_to_camera =
    rotation_camera_to_gimbal_optical * cameraToOpticalRotation().t();
  const cv::Mat translation_gimbal_to_camera = translation_camera_to_gimbal_optical.clone();

  result.rotation_gimbal_to_camera = rotation_gimbal_to_camera;
  result.translation_gimbal_to_camera = translation_gimbal_to_camera;

  const cv::Mat transform_gimbal_to_optical = makeTransform(
    rotation_camera_to_gimbal_optical, translation_camera_to_gimbal_optical);

  std::vector<cv::Mat> base_to_target_transforms;
  base_to_target_transforms.reserve(samples.size());
  for (const auto & sample : samples) {
    const cv::Mat transform_base_to_gimbal = makeTransform(
      buildBaseToGimbalRotation(sample.yaw_rad, sample.pitch_rad),
      (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0));
    const cv::Mat transform_target_to_optical = makeTransform(
      sample.rotation_target_to_optical,
      sample.translation_target_to_optical);
    base_to_target_transforms.push_back(
      transform_base_to_gimbal * transform_gimbal_to_optical * transform_target_to_optical);
  }

  const cv::Mat reference = base_to_target_transforms.front();
  double translation_sum = 0.0;
  double translation_max = 0.0;
  double rotation_sum = 0.0;
  double rotation_max = 0.0;

  for (std::size_t i = 1; i < base_to_target_transforms.size(); ++i) {
    const cv::Mat delta = invertTransform(reference) * base_to_target_transforms[i];
    const cv::Mat delta_translation = delta(cv::Rect(3, 0, 1, 3));
    const cv::Mat delta_rotation = delta(cv::Rect(0, 0, 3, 3));
    const double translation_norm = cv::norm(delta_translation);
    const double rotation_deg = rotationAngleDeg(delta_rotation);
    translation_sum += translation_norm;
    rotation_sum += rotation_deg;
    translation_max = std::max(translation_max, translation_norm);
    rotation_max = std::max(rotation_max, rotation_deg);
  }

  const double denom = std::max<std::size_t>(1U, base_to_target_transforms.size() - 1U);
  result.mean_target_translation_delta_m = translation_sum / static_cast<double>(denom);
  result.max_target_translation_delta_m = translation_max;
  result.mean_target_rotation_delta_deg = rotation_sum / static_cast<double>(denom);
  result.max_target_rotation_delta_deg = rotation_max;

  return true;
}

cv::Mat AimHandeyeCalibratorNode::toDoubleMatrix(const cv::Mat & input)
{
  if (input.empty()) {
    return input;
  }
  if (input.depth() == CV_64F) {
    return input.clone();
  }
  cv::Mat output;
  input.convertTo(output, CV_64F);
  return output;
}

cv::Mat AimHandeyeCalibratorNode::buildBaseToGimbalRotation(double yaw_rad, double pitch_rad) const
{
  return rotationFromRollPitchYaw(0.0, pitch_rad, yaw_rad);
}

cv::Mat AimHandeyeCalibratorNode::cameraToOpticalRotation() const
{
  cv::Mat rotation = cv::Mat::zeros(3, 3, CV_64F);
  rotation.at<double>(0, 2) = 1.0;
  rotation.at<double>(1, 0) = -1.0;
  rotation.at<double>(2, 1) = -1.0;
  return rotation;
}

std::vector<cv::Point3f> AimHandeyeCalibratorNode::boardObjectPoints() const
{
  std::vector<cv::Point3f> object_points;
  object_points.reserve(static_cast<std::size_t>(board_cols_ * board_rows_));
  for (int row = 0; row < board_rows_; ++row) {
    for (int col = 0; col < board_cols_; ++col) {
      object_points.emplace_back(
        static_cast<float>(col * square_size_m_),
        static_cast<float>(row * square_size_m_),
        0.0F);
    }
  }
  return object_points;
}

double AimHandeyeCalibratorNode::computeReprojectionError(
  const std::vector<cv::Point2f> & image_points,
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion_coefficients,
  const cv::Mat & rotation_target_to_optical,
  const cv::Mat & translation_target_to_optical) const
{
  cv::Mat rvec;
  cv::Rodrigues(rotation_target_to_optical, rvec);
  std::vector<cv::Point2f> projected_points;
  cv::projectPoints(
    boardObjectPoints(), rvec, translation_target_to_optical, camera_matrix,
    distortion_coefficients, projected_points);

  if (projected_points.size() != image_points.size()) {
    return std::numeric_limits<double>::infinity();
  }

  double error_sum = 0.0;
  for (std::size_t i = 0; i < image_points.size(); ++i) {
    error_sum += cv::norm(projected_points[i] - image_points[i]);
  }
  return error_sum / static_cast<double>(image_points.size());
}

std::string AimHandeyeCalibratorNode::formatTransformYaml(const SolveResult & result) const
{
  std::size_t sample_count = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sample_count = samples_.size();
  }
  const auto rpy = rollPitchYawFromRotation(result.rotation_gimbal_to_camera);
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(8);
  oss << "samples: " << sample_count << "\n";
  oss << "consistency:\n";
  oss << "  mean_target_translation_delta_m: " << result.mean_target_translation_delta_m << "\n";
  oss << "  max_target_translation_delta_m: " << result.max_target_translation_delta_m << "\n";
  oss << "  mean_target_rotation_delta_deg: " << result.mean_target_rotation_delta_deg << "\n";
  oss << "  max_target_rotation_delta_deg: " << result.max_target_rotation_delta_deg << "\n";
  oss << "\n";
  oss << "# paste into src/sentry_tf/config/sentry_tf.yaml\n";
  oss << output_tf_section_ << ":\n";
  oss << "  x: " << result.translation_gimbal_to_camera.at<double>(0, 0) << "\n";
  oss << "  y: " << result.translation_gimbal_to_camera.at<double>(1, 0) << "\n";
  oss << "  z: " << result.translation_gimbal_to_camera.at<double>(2, 0) << "\n";
  oss << "  yaw: " << rpy[2] << "\n";
  oss << "  pitch: " << rpy[1] << "\n";
  oss << "  roll: " << rpy[0] << "\n";
  oss << "  parent_frame: \"" << parent_frame_id_ << "\"\n";
  oss << "  child_frame: \"" << child_frame_id_ << "\"\n";
  oss << "\n";
  oss << "# rpy_deg reference\n";
  oss << "camera_rpy_deg: ["
      << rad2deg(rpy[0]) << ", "
      << rad2deg(rpy[1]) << ", "
      << rad2deg(rpy[2]) << "]\n";
  return oss.str();
}

bool AimHandeyeCalibratorNode::saveResultToFile(const std::string & content) const
{
  std::ofstream output(output_file_, std::ios::out | std::ios::trunc);
  if (!output.is_open()) {
    RCLCPP_ERROR(get_logger(), "failed to open output file '%s'", output_file_.c_str());
    return false;
  }
  output << content;
  return output.good();
}

}  // namespace aim_handeye_calibrator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aim_handeye_calibrator::AimHandeyeCalibratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
