#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include "aim_armor_detector/team_color.hpp"
#include "aim_armor_detector/openvino_infer.hpp"
#include "aim_msgs/msg/armor_set_array.hpp"

namespace aim_armor_detector
{

class AimArmorDetectorNode : public rclcpp::Node
{
public:
  explicit AimArmorDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AimArmorDetectorNode() override;

private:
  struct CameraPipeline
  {
    std::string camera_name;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Publisher<aim_msgs::msg::ArmorSetArray>::SharedPtr armor_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub;
    std::unique_ptr<OpenvinoInfer> infer;
    std::thread worker;
    bool enabled{false};
    bool running{true};
    std::mutex mutex;
    std::condition_variable cv;
    sensor_msgs::msg::Image::ConstSharedPtr latest_msg;
    std::mutex latency_mutex;
    std::chrono::steady_clock::time_point last_latency_log_time{};
    double latency_sum_ms{0.0};
    double latency_max_ms{0.0};
    std::uint64_t latency_count{0};
    std::mutex transfer_latency_mutex;
    std::chrono::steady_clock::time_point last_transfer_latency_log_time{};
    double transfer_latency_sum_ms{0.0};
    double transfer_latency_max_ms{0.0};
    std::uint64_t transfer_latency_count{0};
  };

  void initCameraPipeline(
    CameraPipeline & pipeline,
    const std::string & prefix,
    std::optional<LightbarPcaCorrectorConfig> pca_config);
  void shutdownPipeline(CameraPipeline & pipeline);

  void onImage(CameraPipeline & pipeline, const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void processLoop(CameraPipeline & pipeline);
  void publishDetections(
    CameraPipeline & pipeline,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    const std::vector<DetectionObject> & detections);
  void publishVisualization(
    CameraPipeline & pipeline,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    const std::vector<DetectionObject> & detections);
  static cv::Scalar colorForDetection(int color_id);
  void recordImageTransferLatency(
    CameraPipeline & pipeline,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void recordInferenceLatency(CameraPipeline & pipeline, double latency_ms);
  void submitFrame(
    CameraPipeline & pipeline,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    std::size_t slot,
    bool & slot_active);
  bool takeLatestFrame(CameraPipeline & pipeline, sensor_msgs::msg::Image::ConstSharedPtr & msg);

  std::atomic<int> detect_color_{kUnknownDetectColor};
  bool enable_visualization_{false};
  bool enable_pca_correction_{true};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr team_color_sub_;

  CameraPipeline front_pipeline_;
  CameraPipeline back_pipeline_;
};

}  // namespace aim_armor_detector
