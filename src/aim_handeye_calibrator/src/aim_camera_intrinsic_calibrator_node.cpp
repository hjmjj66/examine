#include "aim_handeye_calibrator/aim_camera_intrinsic_calibrator_node.hpp"

#include <algorithm>
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

bool isCameraMatrixValid(const cv::Mat & camera_matrix)
{
  return camera_matrix.rows == 3 && camera_matrix.cols == 3 &&
         camera_matrix.at<double>(0, 0) > 0.0 &&
         camera_matrix.at<double>(1, 1) > 0.0 &&
         camera_matrix.at<double>(2, 2) == 1.0;
}

std::string joinVector(const std::vector<double> & values)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(10);
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0U) {
      oss << ", ";
    }
    oss << values[i];
  }
  return oss.str();
}

std::vector<double> matToRowMajorVector(const cv::Mat & matrix)
{
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(matrix.rows * matrix.cols));
  for (int row = 0; row < matrix.rows; ++row) {
    for (int col = 0; col < matrix.cols; ++col) {
      values.push_back(matrix.at<double>(row, col));
    }
  }
  return values;
}

std::vector<double> distortionToVector(const cv::Mat & coefficients)
{
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(coefficients.total()));
  const cv::Mat flat = coefficients.reshape(1, 1);
  for (int col = 0; col < flat.cols; ++col) {
    values.push_back(flat.at<double>(0, col));
  }
  return values;
}

}  // namespace

AimCameraIntrinsicCalibratorNode::AimCameraIntrinsicCalibratorNode(
  const rclcpp::NodeOptions & options)
: Node("aim_camera_intrinsic_calibrator_node", options)
{
  declare_parameter<std::string>("image_topic", "/gx_camera_0/image_raw");
  declare_parameter<std::string>("debug_image_topic", "/aim_camera_intrinsic/debug_image");
  declare_parameter<std::string>("output_file", "");
  declare_parameter<std::string>("camera_name", "front_0");
  declare_parameter<std::string>("frame_id", "gx_camera_0");
  declare_parameter<std::string>("distortion_model", "plumb_bob");
  declare_parameter<std::string>("aim_solver_camera_matrix_key", "front_camera_matrix");
  declare_parameter<std::string>("aim_solver_distortion_key", "front_distortion_coefficients");
  declare_parameter<int>("board_cols", 11);
  declare_parameter<int>("board_rows", 8);
  declare_parameter<int>("min_samples", 12);
  declare_parameter<double>("square_size_m", 0.030);
  declare_parameter<bool>("use_fisheye_model", false);
  declare_parameter<bool>("fix_aspect_ratio", false);
  declare_parameter<bool>("zero_tangent_distortion", false);

  image_topic_ = get_parameter("image_topic").as_string();
  debug_image_topic_ = get_parameter("debug_image_topic").as_string();
  output_file_ = get_parameter("output_file").as_string();
  camera_name_ = get_parameter("camera_name").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  distortion_model_ = get_parameter("distortion_model").as_string();
  aim_solver_camera_matrix_key_ = get_parameter("aim_solver_camera_matrix_key").as_string();
  aim_solver_distortion_key_ = get_parameter("aim_solver_distortion_key").as_string();
  board_cols_ = static_cast<int>(get_parameter("board_cols").as_int());
  board_rows_ = static_cast<int>(get_parameter("board_rows").as_int());
  min_samples_ = static_cast<int>(get_parameter("min_samples").as_int());
  square_size_m_ = get_parameter("square_size_m").as_double();
  use_fisheye_model_ = get_parameter("use_fisheye_model").as_bool();
  fix_aspect_ratio_ = get_parameter("fix_aspect_ratio").as_bool();
  zero_tangent_distortion_ = get_parameter("zero_tangent_distortion").as_bool();

  if (board_cols_ < 2 || board_rows_ < 2) {
    throw std::runtime_error("board_cols and board_rows must be inner-corner counts >= 2");
  }
  if (square_size_m_ <= 0.0) {
    throw std::runtime_error("square_size_m must be positive");
  }
  if (min_samples_ < 3) {
    throw std::runtime_error("min_samples must be >= 3");
  }
  if (use_fisheye_model_) {
    distortion_model_ = "equidistant";
  }

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, rclcpp::SensorDataQoS().keep_last(1),
    [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(msg); });
  debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
    debug_image_topic_, rclcpp::SensorDataQoS().keep_last(1));
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
    "aim camera intrinsic calibrator ready: image_topic='%s', board=%dx%d inner corners, square=%.4f m",
    image_topic_.c_str(),
    board_cols_,
    board_rows_,
    square_size_m_);
}

void AimCameraIntrinsicCalibratorNode::onImage(
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_image_ = msg;
}

void AimCameraIntrinsicCalibratorNode::handleCapture(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  try {
    sensor_msgs::msg::Image::ConstSharedPtr image_msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      image_msg = latest_image_;
    }

    if (!image_msg) {
      response->success = false;
      response->message = "no image received yet";
      return;
    }

    std::vector<cv::Point2f> corners;
    cv::Mat debug_image;
    if (!detectBoardCorners(image_msg, corners, debug_image)) {
      response->success = false;
      response->message = "chessboard corners not found";
      return;
    }

    std::size_t sample_count = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const cv::Size current_size(
        static_cast<int>(image_msg->width),
        static_cast<int>(image_msg->height));
      if (!image_points_.empty() && image_size_ != current_size) {
        response->success = false;
        response->message = "image size changed; clear samples before changing resolution";
        return;
      }
      image_size_ = current_size;
      image_points_.push_back(corners);
      sample_count = image_points_.size();
    }

    if (debug_image_pub_->get_subscription_count() > 0U) {
      auto debug_msg = cv_bridge::CvImage(
        image_msg->header, sensor_msgs::image_encodings::BGR8, debug_image).toImageMsg();
      debug_image_pub_->publish(*debug_msg);
    }

    std::ostringstream oss;
    oss << "captured sample #" << sample_count
        << ", corners=" << corners.size()
        << ", image_size=" << image_msg->width << "x" << image_msg->height;
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

void AimCameraIntrinsicCalibratorNode::handleSolve(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  try {
    CalibrationResult result;
    if (!solveCalibration(result)) {
      response->success = false;
      response->message = "calibration failed; check sample count and board coverage";
      return;
    }

    const auto yaml = formatCalibrationYaml(result);
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

void AimCameraIntrinsicCalibratorNode::handleClear(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  image_points_.clear();
  image_size_ = cv::Size();
  response->success = true;
  response->message = "cleared all captured samples";
}

bool AimCameraIntrinsicCalibratorNode::detectBoardCorners(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  std::vector<cv::Point2f> & corners,
  cv::Mat & debug_image) const
{
  cv_bridge::CvImageConstPtr cv_image;
  try {
    cv_image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception &) {
    cv_image = cv_bridge::toCvShare(image_msg);
  }

  if (cv_image->image.empty()) {
    return false;
  }

  if (cv_image->image.channels() == 1) {
    cv::cvtColor(cv_image->image, debug_image, cv::COLOR_GRAY2BGR);
  } else {
    debug_image = cv_image->image.clone();
  }

  cv::Mat gray;
  if (cv_image->image.channels() == 1) {
    gray = cv_image->image;
  } else {
    cv::cvtColor(cv_image->image, gray, cv::COLOR_BGR2GRAY);
  }

  const cv::Size board_size(board_cols_, board_rows_);
  const int flags =
    cv::CALIB_CB_ADAPTIVE_THRESH |
    cv::CALIB_CB_NORMALIZE_IMAGE |
    cv::CALIB_CB_FAST_CHECK;
  bool found = cv::findChessboardCorners(gray, board_size, corners, flags);
  if (!found) {
    found = cv::findChessboardCorners(gray, board_size, corners);
  }
  if (!found) {
    cv::drawChessboardCorners(debug_image, board_size, corners, false);
    return false;
  }

  cv::cornerSubPix(
    gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
    cv::TermCriteria(
      cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
      30,
      0.001));
  cv::drawChessboardCorners(debug_image, board_size, corners, true);
  return true;
}

bool AimCameraIntrinsicCalibratorNode::solveCalibration(CalibrationResult & result) const
{
  std::vector<std::vector<cv::Point2f>> image_points;
  cv::Size image_size;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    image_points = image_points_;
    image_size = image_size_;
  }

  if (static_cast<int>(image_points.size()) < min_samples_) {
    RCLCPP_ERROR(
      get_logger(),
      "not enough samples: %zu captured, need at least %d",
      image_points.size(),
      min_samples_);
    return false;
  }
  if (image_size.width <= 0 || image_size.height <= 0) {
    RCLCPP_ERROR(get_logger(), "invalid image size");
    return false;
  }

  std::vector<std::vector<cv::Point3f>> object_points(
    image_points.size(),
    buildObjectPoints());
  result.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  result.camera_matrix.at<double>(0, 0) = static_cast<double>(image_size.width);
  result.camera_matrix.at<double>(1, 1) = static_cast<double>(image_size.height);
  result.camera_matrix.at<double>(0, 2) = static_cast<double>(image_size.width) * 0.5;
  result.camera_matrix.at<double>(1, 2) = static_cast<double>(image_size.height) * 0.5;

  if (use_fisheye_model_) {
    result.distortion_coefficients = cv::Mat::zeros(4, 1, CV_64F);
    int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    if (fix_aspect_ratio_) {
      flags |= cv::fisheye::CALIB_FIX_SKEW;
    }
    result.rms_reprojection_error_px = cv::fisheye::calibrate(
      object_points,
      image_points,
      image_size,
      result.camera_matrix,
      result.distortion_coefficients,
      result.rotation_vectors,
      result.translation_vectors,
      flags,
      cv::TermCriteria(
        cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
        100,
        1e-6));
  } else {
    result.distortion_coefficients = cv::Mat::zeros(5, 1, CV_64F);
    int flags = 0;
    if (fix_aspect_ratio_) {
      flags |= cv::CALIB_FIX_ASPECT_RATIO;
    }
    if (zero_tangent_distortion_) {
      flags |= cv::CALIB_ZERO_TANGENT_DIST;
    }
    result.rms_reprojection_error_px = cv::calibrateCamera(
      object_points,
      image_points,
      image_size,
      result.camera_matrix,
      result.distortion_coefficients,
      result.rotation_vectors,
      result.translation_vectors,
      flags);
  }

  if (!isCameraMatrixValid(result.camera_matrix)) {
    RCLCPP_ERROR(get_logger(), "calibration produced invalid camera matrix");
    return false;
  }
  result.mean_reprojection_error_px = computeReprojectionErrors(result);
  return true;
}

std::vector<cv::Point3f> AimCameraIntrinsicCalibratorNode::buildObjectPoints() const
{
  std::vector<cv::Point3f> points;
  points.reserve(static_cast<std::size_t>(board_cols_ * board_rows_));
  for (int row = 0; row < board_rows_; ++row) {
    for (int col = 0; col < board_cols_; ++col) {
      points.emplace_back(
        static_cast<float>(static_cast<double>(col) * square_size_m_),
        static_cast<float>(static_cast<double>(row) * square_size_m_),
        0.0F);
    }
  }
  return points;
}

double AimCameraIntrinsicCalibratorNode::computeReprojectionErrors(
  CalibrationResult & result) const
{
  std::vector<std::vector<cv::Point2f>> image_points;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    image_points = image_points_;
  }

  const auto object_points = buildObjectPoints();
  double total_error = 0.0;
  std::size_t total_points = 0U;
  result.max_reprojection_error_px = 0.0;

  for (std::size_t sample_index = 0; sample_index < image_points.size(); ++sample_index) {
    std::vector<cv::Point2f> projected_points;
    if (use_fisheye_model_) {
      cv::fisheye::projectPoints(
        object_points,
        projected_points,
        result.rotation_vectors[sample_index],
        result.translation_vectors[sample_index],
        result.camera_matrix,
        result.distortion_coefficients);
    } else {
      cv::projectPoints(
        object_points,
        result.rotation_vectors[sample_index],
        result.translation_vectors[sample_index],
        result.camera_matrix,
        result.distortion_coefficients,
        projected_points);
    }

    for (std::size_t point_index = 0; point_index < image_points[sample_index].size();
      ++point_index)
    {
      const double error = cv::norm(
        image_points[sample_index][point_index] - projected_points[point_index]);
      total_error += error;
      result.max_reprojection_error_px = std::max(result.max_reprojection_error_px, error);
      ++total_points;
    }
  }

  if (total_points == 0U) {
    return std::numeric_limits<double>::infinity();
  }
  return total_error / static_cast<double>(total_points);
}

std::string AimCameraIntrinsicCalibratorNode::formatCalibrationYaml(
  const CalibrationResult & result) const
{
  std::size_t sample_count = 0U;
  cv::Size image_size;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sample_count = image_points_.size();
    image_size = image_size_;
  }

  cv::Mat rectification_matrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat projection_matrix = cv::Mat::zeros(3, 4, CV_64F);
  result.camera_matrix.copyTo(projection_matrix(cv::Rect(0, 0, 3, 3)));

  const auto camera_matrix = matToRowMajorVector(result.camera_matrix);
  const auto distortion_coefficients = distortionToVector(result.distortion_coefficients);
  const auto rectification = matToRowMajorVector(rectification_matrix);
  const auto projection = matToRowMajorVector(projection_matrix);

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(10);
  oss << "camera_name: " << camera_name_ << "\n";
  oss << "frame_id: " << frame_id_ << "\n";
  oss << "image_width: " << image_size.width << "\n";
  oss << "image_height: " << image_size.height << "\n";
  oss << "samples: " << sample_count << "\n";
  oss << "board:\n";
  oss << "  cols: " << board_cols_ << "\n";
  oss << "  rows: " << board_rows_ << "\n";
  oss << "  square_size_m: " << square_size_m_ << "\n";
  oss << "reprojection_error:\n";
  oss << "  rms_px: " << result.rms_reprojection_error_px << "\n";
  oss << "  mean_px: " << result.mean_reprojection_error_px << "\n";
  oss << "  max_px: " << result.max_reprojection_error_px << "\n";
  oss << "distortion_model: " << distortion_model_ << "\n";
  oss << "camera_matrix: [" << joinVector(camera_matrix) << "]\n";
  oss << "distortion_coefficients: [" << joinVector(distortion_coefficients) << "]\n";
  oss << "rectification_matrix: [" << joinVector(rectification) << "]\n";
  oss << "projection_matrix: [" << joinVector(projection) << "]\n";
  oss << "\n";
  oss << "# paste into src/aim_solver/config/aim_solver.yaml\n";
  oss << "    " << aim_solver_camera_matrix_key_ << ": [" << joinVector(camera_matrix) << "]\n";
  oss << "    " << aim_solver_distortion_key_ << ": [" << joinVector(distortion_coefficients) << "]\n";
  oss << "\n";
  oss << "# camera driver style reference\n";
  oss << "    distortion_model: \"" << distortion_model_ << "\"\n";
  oss << "    rectification_matrix: [" << joinVector(rectification) << "]\n";
  oss << "    projection_matrix: [" << joinVector(projection) << "]\n";
  return oss.str();
}

bool AimCameraIntrinsicCalibratorNode::saveResultToFile(const std::string & content) const
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
  auto node = std::make_shared<aim_handeye_calibrator::AimCameraIntrinsicCalibratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
