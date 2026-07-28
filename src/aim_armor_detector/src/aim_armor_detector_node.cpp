#include "aim_armor_detector/aim_armor_detector_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <map>
#include <sstream>
#include <string>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge.hpp>
#else
  #include <cv_bridge/cv_bridge.h>
#endif

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "aim_msgs/msg/armor.hpp"
#include "aim_msgs/msg/armor_class.hpp"
#include "aim_msgs/msg/armor_set.hpp"

namespace aim_armor_detector
{

AimArmorDetectorNode::AimArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("aim_armor_detector_node", options)
{
  declare_parameter<std::string>("front_image_topic", "/gx_camera/image_raw");
  declare_parameter<std::string>("front_armor_topic", "/aim_detector/front/armor_sets");
  declare_parameter<std::string>("front_visualization_topic", "/aim_detector/front/visualization");
  declare_parameter<bool>("enable_front_camera", true);
  declare_parameter<std::string>("camera_name", "front");

  declare_parameter<std::string>("back_image_topic", "/usb_camera/image_raw");
  declare_parameter<std::string>("back_armor_topic", "/aim_detector/back/armor_sets");
  declare_parameter<std::string>("back_visualization_topic", "/aim_detector/back/visualization");
  declare_parameter<bool>("enable_back_camera", true);

  declare_parameter<std::string>("model_path", "");
  declare_parameter<std::string>("device_name", "CPU");
  declare_parameter<bool>("enable_visualization", false);
  declare_parameter<bool>("enable_pca_correction", true);
  declare_parameter<int>("pca.pass_optimize_lightbar_width", 3);
  declare_parameter<double>("pca.normalize_max_brightness", 25.0);
  declare_parameter<double>("pca.lightbar_min_mean_brightness", 30.0);
  declare_parameter<double>("pca.padding_scale", 0.07);
  declare_parameter<double>("pca.search_start_ratio", 0.4);
  declare_parameter<double>("pca.search_end_ratio", 0.6);
  declare_parameter<double>("pca.estimated_width_ratio", 0.18);
  declare_parameter<int>("pca.min_sample_width", 5);

  team_color_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/ly/friend/is_team_red",
    rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
      if (!msg) {
        return;
      }
      detect_color_.store(detectColorForTeam(msg->data));
    });
  enable_visualization_ = get_parameter("enable_visualization").as_bool();
  enable_pca_correction_ = get_parameter("enable_pca_correction").as_bool();

  std::optional<LightbarPcaCorrectorConfig> pca_config = std::nullopt;
  if (enable_pca_correction_) {
    LightbarPcaCorrectorConfig config;
    config.pass_optimize_lightbar_width =
      get_parameter("pca.pass_optimize_lightbar_width").as_int();
    config.normalize_max_brightness =
      static_cast<float>(get_parameter("pca.normalize_max_brightness").as_double());
    config.lightbar_min_mean_brightness =
      static_cast<float>(get_parameter("pca.lightbar_min_mean_brightness").as_double());
    config.padding_scale =
      static_cast<float>(get_parameter("pca.padding_scale").as_double());
    config.search_start_ratio =
      static_cast<float>(get_parameter("pca.search_start_ratio").as_double());
    config.search_end_ratio =
      static_cast<float>(get_parameter("pca.search_end_ratio").as_double());
    config.estimated_width_ratio =
      static_cast<float>(get_parameter("pca.estimated_width_ratio").as_double());
    config.min_sample_width =
      get_parameter("pca.min_sample_width").as_int();
    pca_config = config;
  }

  front_pipeline_.camera_name = get_parameter("camera_name").as_string();
  back_pipeline_.camera_name = "back";

  initCameraPipeline(front_pipeline_, "front", pca_config);
  initCameraPipeline(back_pipeline_, "back", pca_config);
}

AimArmorDetectorNode::~AimArmorDetectorNode()
{
  shutdownPipeline(front_pipeline_);
  shutdownPipeline(back_pipeline_);
}

void AimArmorDetectorNode::initCameraPipeline(
  CameraPipeline & pipeline,
  const std::string & prefix,
  std::optional<LightbarPcaCorrectorConfig> pca_config)
{
  const std::string enable_param = "enable_" + prefix + "_camera";
  pipeline.enabled = get_parameter(enable_param).as_bool();
  if (!pipeline.enabled) {
    RCLCPP_INFO(get_logger(), "[%s] camera pipeline disabled", pipeline.camera_name.c_str());
    return;
  }

  const std::string image_param = prefix + "_image_topic";
  const std::string armor_param = prefix + "_armor_topic";
  const std::string vis_param = prefix + "_visualization_topic";

  const auto image_topic = get_parameter(image_param).as_string();
  const auto armor_topic = get_parameter(armor_param).as_string();
  const auto visualization_topic = get_parameter(vis_param).as_string();

  const auto model_path = get_parameter("model_path").as_string();
  const auto device_name = get_parameter("device_name").as_string();
  if (model_path.empty()) {
    throw std::runtime_error(
            "aim_armor_detector requires a non-empty 'model_path' parameter. "
            "Set it in the detector yaml or launch file.");
  }

  pipeline.infer = std::make_unique<OpenvinoInfer>(model_path, device_name, pca_config);
  pipeline.last_latency_log_time = std::chrono::steady_clock::now();
  pipeline.last_transfer_latency_log_time = std::chrono::steady_clock::now();

  pipeline.infer->setInferenceLatencyCallback(
    [this, &pipeline](double latency_ms) { recordInferenceLatency(pipeline, latency_ms); });

  pipeline.armor_pub = create_publisher<aim_msgs::msg::ArmorSetArray>(
    armor_topic, rclcpp::SensorDataQoS());

  if (enable_visualization_) {
    pipeline.visualization_pub = create_publisher<sensor_msgs::msg::Image>(
      visualization_topic, rclcpp::SensorDataQoS());
  }

  auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
  pipeline.image_sub = create_subscription<sensor_msgs::msg::Image>(
    image_topic,
    image_qos,
    [this, &pipeline](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
      onImage(pipeline, msg);
    });

  pipeline.worker = std::thread([this, &pipeline]() { processLoop(pipeline); });

  RCLCPP_INFO(
    get_logger(),
    "[%s] camera pipeline initialized: image=%s, armor=%s",
    pipeline.camera_name.c_str(),
    image_topic.c_str(),
    armor_topic.c_str());
}

void AimArmorDetectorNode::shutdownPipeline(CameraPipeline & pipeline)
{
  {
    std::lock_guard<std::mutex> lock(pipeline.mutex);
    pipeline.running = false;
  }
  pipeline.cv.notify_all();
  if (pipeline.worker.joinable()) {
    pipeline.worker.join();
  }
}

void AimArmorDetectorNode::onImage(
  CameraPipeline & pipeline,
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  recordImageTransferLatency(pipeline, msg);
  {
    std::lock_guard<std::mutex> lock(pipeline.mutex);
    pipeline.latest_msg = msg;
  }
  pipeline.cv.notify_one();
}

void AimArmorDetectorNode::processLoop(CameraPipeline & pipeline)
{
  std::array<sensor_msgs::msg::Image::ConstSharedPtr, OpenvinoInfer::kPipelineDepth> in_flight_msgs;
  std::array<bool, OpenvinoInfer::kPipelineDepth> slot_active{false, false};
  std::size_t next_submit_slot = 0;

  while (rclcpp::ok() && pipeline.running) {
    sensor_msgs::msg::Image::ConstSharedPtr msg;
    if (!takeLatestFrame(pipeline, msg)) {
      break;
    }

    if (detect_color_.load() == kUnknownDetectColor) {
      continue;
    }

    if (slot_active[next_submit_slot]) {
      const auto detections =
        pipeline.infer->getResult(
          in_flight_msgs[next_submit_slot]->width > 0 ?
            cv::Size(
              static_cast<int>(in_flight_msgs[next_submit_slot]->width),
              static_cast<int>(in_flight_msgs[next_submit_slot]->height)) :
            cv::Size(),
          detect_color_.load(),
          next_submit_slot);
      publishDetections(pipeline, in_flight_msgs[next_submit_slot], detections);
      in_flight_msgs[next_submit_slot].reset();
      slot_active[next_submit_slot] = false;
    }

    submitFrame(pipeline, msg, next_submit_slot, slot_active[next_submit_slot]);
    in_flight_msgs[next_submit_slot] = msg;
    next_submit_slot = (next_submit_slot + 1) % OpenvinoInfer::kPipelineDepth;
  }

  for (std::size_t slot = 0; slot < OpenvinoInfer::kPipelineDepth; ++slot) {
    if (!slot_active[slot]) {
      continue;
    }
    const auto detections = pipeline.infer->getResult(
      cv::Size(
        static_cast<int>(in_flight_msgs[slot]->width),
        static_cast<int>(in_flight_msgs[slot]->height)),
      detect_color_.load(),
      slot);
    publishDetections(pipeline, in_flight_msgs[slot], detections);
    in_flight_msgs[slot].reset();
    slot_active[slot] = false;
  }

  RCLCPP_INFO(
    get_logger(),
    "[%s] processLoop exited",
    pipeline.camera_name.c_str());
}

void AimArmorDetectorNode::submitFrame(
  CameraPipeline & pipeline,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg,
  std::size_t slot,
  bool & slot_active)
{
  cv_bridge::CvImageConstPtr cv_image;
  try {
    cv_image = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const std::exception &) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[%s] failed to decode image message",
      pipeline.camera_name.c_str());
    return;
  }

  pipeline.infer->startAsync(cv_image->image, slot);
  slot_active = true;
}

bool AimArmorDetectorNode::takeLatestFrame(
  CameraPipeline & pipeline,
  sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  std::unique_lock<std::mutex> lock(pipeline.mutex);
  pipeline.cv.wait(lock, [&pipeline]() {
    return !pipeline.running || pipeline.latest_msg != nullptr;
  });
  if (!pipeline.running) {
    return false;
  }
  msg = std::move(pipeline.latest_msg);
  pipeline.latest_msg.reset();
  return static_cast<bool>(msg);
}

void AimArmorDetectorNode::publishDetections(
  CameraPipeline & pipeline,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg,
  const std::vector<DetectionObject> & detections)
{
  if (enable_visualization_) {
    publishVisualization(pipeline, msg, detections);
  }

  std::map<int, aim_msgs::msg::ArmorSet> grouped_sets;

  for (const auto & detection : detections) {
    aim_msgs::msg::Armor armor_msg;
    armor_msg.header = msg->header;
    for (int i = 0; i < 4; ++i) {
      geometry_msgs::msg::Point point;
      const int point_index = i * 2;
      point.x = detection.landmarks[point_index];
      point.y = detection.landmarks[point_index + 1];
      point.z = 0.0;
      armor_msg.corners[static_cast<std::size_t>(i)] = point;
    }
    armor_msg.armor_class.class_id = static_cast<std::uint8_t>(detection.label);
    armor_msg.armor_class.team = static_cast<std::uint8_t>(detection.color);

    auto & armor_set = grouped_sets[detection.label];
    armor_set.header = msg->header;
    armor_set.id = static_cast<std::uint8_t>(detection.label);
    armor_set.armors.push_back(armor_msg);
  }

  aim_msgs::msg::ArmorSetArray output;
  output.header = msg->header;
  output.armor_sets.reserve(grouped_sets.size());
  for (auto & [_, armor_set] : grouped_sets) {
    output.armor_sets.push_back(std::move(armor_set));
  }

  if (!output.armor_sets.empty()) {
    RCLCPP_DEBUG(
      get_logger(),
      "[%s] detected %zu armor sets, %zu armors total",
      pipeline.camera_name.c_str(),
      output.armor_sets.size(),
      detections.size());
  }

  pipeline.armor_pub->publish(std::move(output));
}

void AimArmorDetectorNode::publishVisualization(
  CameraPipeline & pipeline,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg,
  const std::vector<DetectionObject> & detections)
{
  if (!pipeline.visualization_pub) {
    return;
  }

  cv_bridge::CvImageConstPtr cv_image;
  try {
    cv_image = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const std::exception &) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[%s] failed to decode image message for visualization",
      pipeline.camera_name.c_str());
    return;
  }

  auto visualization = cv_image->image.clone();
  for (const auto & detection : detections) {
    const auto color = colorForDetection(detection.color);
    std::array<cv::Point, 4> corners{};
    for (int i = 0; i < 4; ++i) {
      corners[static_cast<std::size_t>(i)] = cv::Point(
        static_cast<int>(std::lround(detection.landmarks[i * 2])),
        static_cast<int>(std::lround(detection.landmarks[i * 2 + 1])));
    }

    for (std::size_t i = 0; i < corners.size(); ++i) {
      cv::line(
        visualization,
        corners[i],
        corners[(i + 1) % corners.size()],
        color,
        2,
        cv::LINE_AA);
    }

    for (int i = 0; i < 4; ++i) {
      cv::circle(
        visualization,
        corners[static_cast<std::size_t>(i)],
        3,
        color,
        cv::FILLED);
    }

    std::ostringstream label;
    label << "id:" << detection.label << " c:" << detection.color;
    cv::putText(
      visualization,
      label.str(),
      cv::Point(
        std::max(0, static_cast<int>(detection.rect.x)),
        std::max(15, static_cast<int>(detection.rect.y) - 6)),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      color,
      1,
      cv::LINE_AA);
  }

  auto visualization_msg = std::make_unique<sensor_msgs::msg::Image>();
  visualization_msg->header = msg->header;
  visualization_msg->height = static_cast<std::uint32_t>(visualization.rows);
  visualization_msg->width = static_cast<std::uint32_t>(visualization.cols);
  visualization_msg->encoding = "bgr8";
  visualization_msg->is_bigendian = false;
  visualization_msg->step =
    static_cast<sensor_msgs::msg::Image::_step_type>(visualization.step);
  const auto data_bytes =
    static_cast<std::size_t>(visualization.step) * static_cast<std::size_t>(visualization.rows);
  visualization_msg->data.assign(visualization.data, visualization.data + data_bytes);
  pipeline.visualization_pub->publish(std::move(visualization_msg));
}

cv::Scalar AimArmorDetectorNode::colorForDetection(int color_id)
{
  if (color_id == 0) {
    return cv::Scalar(255, 0, 0);
  }
  if (color_id == 1) {
    return cv::Scalar(0, 0, 255);
  }
  return cv::Scalar(0, 255, 255);
}

void AimArmorDetectorNode::recordImageTransferLatency(
  CameraPipeline & pipeline,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    return;
  }

  const double latency_ms = (now() - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0;

  std::lock_guard<std::mutex> lock(pipeline.transfer_latency_mutex);
  pipeline.transfer_latency_sum_ms += latency_ms;
  pipeline.transfer_latency_max_ms =
    std::max(pipeline.transfer_latency_max_ms, latency_ms);
  ++pipeline.transfer_latency_count;

  const auto current_time = std::chrono::steady_clock::now();
  if (current_time - pipeline.last_transfer_latency_log_time < std::chrono::seconds(1)) {
    return;
  }

  const double latency_avg_ms =
    pipeline.transfer_latency_sum_ms / static_cast<double>(pipeline.transfer_latency_count);
  RCLCPP_INFO(
    get_logger(),
    "[%s] image transfer latency: count=%lu avg=%.2f ms max=%.2f ms",
    pipeline.camera_name.c_str(),
    static_cast<unsigned long>(pipeline.transfer_latency_count),
    latency_avg_ms,
    pipeline.transfer_latency_max_ms);

  pipeline.last_transfer_latency_log_time = current_time;
  pipeline.transfer_latency_sum_ms = 0.0;
  pipeline.transfer_latency_max_ms = 0.0;
  pipeline.transfer_latency_count = 0;
}

void AimArmorDetectorNode::recordInferenceLatency(
  CameraPipeline & pipeline,
  double latency_ms)
{
  std::lock_guard<std::mutex> lock(pipeline.latency_mutex);
  pipeline.latency_sum_ms += latency_ms;
  pipeline.latency_max_ms = std::max(pipeline.latency_max_ms, latency_ms);
  ++pipeline.latency_count;

  const auto now = std::chrono::steady_clock::now();
  if (now - pipeline.last_latency_log_time < std::chrono::seconds(1)) {
    return;
  }

  const double latency_avg_ms =
    pipeline.latency_sum_ms / static_cast<double>(pipeline.latency_count);
  RCLCPP_INFO(
    get_logger(),
    "[%s] OpenVINO inference latency: count=%lu avg=%.2f ms max=%.2f ms",
    pipeline.camera_name.c_str(),
    static_cast<unsigned long>(pipeline.latency_count),
    latency_avg_ms,
    pipeline.latency_max_ms);

  pipeline.last_latency_log_time = now;
  pipeline.latency_sum_ms = 0.0;
  pipeline.latency_max_ms = 0.0;
  pipeline.latency_count = 0;
}

}  // namespace aim_armor_detector

RCLCPP_COMPONENTS_REGISTER_NODE(aim_armor_detector::AimArmorDetectorNode)
