#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

#include "aim_armor_detector/detection_types.hpp"
#include "aim_armor_detector/lightbar_pca_corrector.hpp"

namespace aim_armor_detector
{

class OpenvinoInfer
{
public:
  static constexpr std::size_t kPipelineDepth = 2;
  using InferenceLatencyCallback = std::function<void(double)>;

  OpenvinoInfer(
    const std::string & model_path,
    const std::string & device_name,
    std::optional<LightbarPcaCorrectorConfig> pca_config = std::nullopt);
  void setInferenceLatencyCallback(InferenceLatencyCallback callback);
  void startAsync(const cv::Mat & image, std::size_t slot);
  std::vector<DetectionObject> getResult(cv::Size image_size, int detect_color, std::size_t slot);

private:
  std::vector<DetectionObject> postprocess(cv::Size image_size, int detect_color, std::size_t slot);
  static double sigmoid(double x);

  ov::Core core_;
  std::shared_ptr<ov::Model> model_;
  ov::CompiledModel compiled_model_;
  std::array<int64_t, 4> input_shape_{};
  std::vector<ov::InferRequest> infer_requests_;
  std::array<std::chrono::steady_clock::time_point, kPipelineDepth> infer_start_times_{};
  std::vector<cv::Mat> resized_images_;
  std::vector<ov::Tensor> input_tensors_;
  std::vector<cv::Mat> gray_images_;
  std::unique_ptr<LightbarPcaCorrector> lightbar_pca_corrector_;
  InferenceLatencyCallback latency_callback_;
};

}  // namespace aim_armor_detector
