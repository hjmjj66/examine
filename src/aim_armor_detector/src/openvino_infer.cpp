#include "aim_armor_detector/openvino_infer.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

#include <rclcpp/rclcpp.hpp>

namespace aim_armor_detector
{

OpenvinoInfer::OpenvinoInfer(
  const std::string & model_path,
  const std::string & device_name,
  std::optional<LightbarPcaCorrectorConfig> pca_config)
{
  model_ = core_.read_model(model_path);
  auto ppp = ov::preprocess::PrePostProcessor(model_);
  ppp.input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(
    ov::preprocess::ColorFormat::BGR);
  ppp.input().preprocess().convert_element_type(ov::element::f32).convert_color(
    ov::preprocess::ColorFormat::RGB).scale({255.0F, 255.0F, 255.0F});
  ppp.input().model().set_layout("NCHW");
  ppp.output().tensor().set_element_type(ov::element::f32);
  model_ = ppp.build();

  std::string actual_device = device_name;
  try {
    compiled_model_ = core_.compile_model(model_, device_name);
  } catch (const std::exception & e) {
    if (device_name != "CPU") {
      RCLCPP_WARN(
        rclcpp::get_logger("openvino_infer"),
        "compile_model with device '%s' failed: %s, falling back to CPU",
        device_name.c_str(), e.what());
      actual_device = "CPU";
      compiled_model_ = core_.compile_model(model_, "CPU");
    } else {
      throw;
    }
  }

  const auto & shape = compiled_model_.input().get_shape();
  for (std::size_t i = 0; i < input_shape_.size() && i < shape.size(); ++i) {
    input_shape_[i] = static_cast<int64_t>(shape[i]);
  }

  infer_requests_.reserve(kPipelineDepth);
  resized_images_.reserve(kPipelineDepth);
  input_tensors_.reserve(kPipelineDepth);
  gray_images_.reserve(kPipelineDepth);

  for (std::size_t i = 0; i < kPipelineDepth; ++i) {
    resized_images_.emplace_back(
      static_cast<int>(input_shape_[1]),
      static_cast<int>(input_shape_[2]),
      CV_8UC3);
    gray_images_.emplace_back();
    input_tensors_.emplace_back(
      compiled_model_.input().get_element_type(),
      compiled_model_.input().get_shape(),
      resized_images_.back().data);
    infer_requests_.push_back(compiled_model_.create_infer_request());
    infer_requests_.back().set_input_tensor(input_tensors_.back());
    const auto slot = i;
    infer_requests_.back().set_callback([this, slot](std::exception_ptr exception) {
      if (exception) {
        return;
      }
      const auto finished_at = std::chrono::steady_clock::now();
      const auto latency_ms =
        std::chrono::duration<double, std::milli>(finished_at - infer_start_times_.at(slot)).count();
      if (latency_callback_) {
        latency_callback_(latency_ms);
      }
    });
  }

  if (pca_config.has_value()) {
    lightbar_pca_corrector_ = std::make_unique<LightbarPcaCorrector>(pca_config.value());
  }
}

void OpenvinoInfer::setInferenceLatencyCallback(InferenceLatencyCallback callback)
{
  latency_callback_ = std::move(callback);
}

void OpenvinoInfer::startAsync(const cv::Mat & image, std::size_t slot)
{
  cv::resize(image, resized_images_.at(slot), resized_images_.at(slot).size());
  cv::cvtColor(resized_images_.at(slot), gray_images_.at(slot), cv::COLOR_BGR2GRAY);
  infer_start_times_.at(slot) = std::chrono::steady_clock::now();
  infer_requests_.at(slot).start_async();
}

std::vector<DetectionObject> OpenvinoInfer::getResult(cv::Size image_size, int detect_color, std::size_t slot)
{
  infer_requests_.at(slot).wait();
  return postprocess(image_size, detect_color, slot);
}

std::vector<DetectionObject> OpenvinoInfer::postprocess(cv::Size image_size, int detect_color, std::size_t slot)
{
  std::vector<DetectionObject> objects;
  const auto output = infer_requests_.at(slot).get_output_tensor(0);
  const auto output_shape = output.get_shape();
  cv::Mat output_buffer(static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]), CV_32F);
  std::memcpy(output_buffer.ptr(), output.data(), output.get_byte_size());

  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<DetectionObject> all_objects;
  boxes.reserve(static_cast<std::size_t>(output_buffer.rows));
  confidences.reserve(static_cast<std::size_t>(output_buffer.rows));
  all_objects.reserve(static_cast<std::size_t>(output_buffer.rows));
  constexpr float conf_threshold = 0.65F;
  constexpr float nms_threshold = 0.45F;
  const auto & resized_image = resized_images_.at(slot);
  const float scale_x = static_cast<float>(image_size.width) / static_cast<float>(resized_image.cols);
  const float scale_y = static_cast<float>(image_size.height) / static_cast<float>(resized_image.rows);

  for (int i = 0; i < output_buffer.rows; ++i) {
    float confidence = static_cast<float>(sigmoid(output_buffer.at<float>(i, 8)));
    if (confidence < conf_threshold) {
      continue;
    }

    const cv::Mat color_scores = output_buffer.row(i).colRange(9, 13);
    const cv::Mat class_scores = output_buffer.row(i).colRange(13, 22);
    cv::Point color_id;
    cv::Point class_id;
    double color_score = 0.0;
    double class_score = 0.0;
    cv::minMaxLoc(color_scores, nullptr, &color_score, nullptr, &color_id);
    cv::minMaxLoc(class_scores, nullptr, &class_score, nullptr, &class_id);

    // color_id 含义: 0=蓝方装甲板, 1=红方装甲板, 2/3=无效(丢弃)
    if (color_id.x == 2 || color_id.x == 3) {
      continue;
    }
    // detect_color: 0=敌方为蓝方(打蓝色), 1=敌方为红方(打红色)
    // 过滤掉非敌方颜色的装甲板
    if ((detect_color == 0 && color_id.x == 1) || (detect_color == 1 && color_id.x == 0)) {
      continue;
    }

    DetectionObject object{};
    object.label = class_id.x;
    object.color = color_id.x;
    object.probability = confidence;
    for (int j = 0; j < 8; ++j) {
      object.landmarks[j] = output_buffer.at<float>(i, j);
    }

    std::vector<cv::Point2f> points;
    points.emplace_back(object.landmarks[0], object.landmarks[1]);
    points.emplace_back(object.landmarks[6], object.landmarks[7]);
    points.emplace_back(object.landmarks[4], object.landmarks[5]);
    points.emplace_back(object.landmarks[2], object.landmarks[3]);

    float min_x = points.front().x;
    float max_x = points.front().x;
    float min_y = points.front().y;
    float max_y = points.front().y;
    for (const auto & point : points) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }
    object.rect = cv::Rect2f(min_x, min_y, max_x - min_x, max_y - min_y);
    all_objects.push_back(object);
    boxes.emplace_back(object.rect);
    confidences.push_back(static_cast<float>(class_score));
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);
  objects.reserve(indices.size());
  for (const int index : indices) {
    if (index >= 0 && static_cast<std::size_t>(index) < all_objects.size()) {
      objects.push_back(all_objects[static_cast<std::size_t>(index)]);
    }
  }

  if (lightbar_pca_corrector_) {
    lightbar_pca_corrector_->correctDetections(objects, gray_images_.at(slot));
  }

  for (auto & object : objects) {
    for (int j = 0; j < 4; ++j) {
      object.landmarks[j * 2] *= scale_x;
      object.landmarks[j * 2 + 1] *= scale_y;
    }
    object.rect = cv::Rect2f(
      object.rect.x * scale_x,
      object.rect.y * scale_y,
      object.rect.width * scale_x,
      object.rect.height * scale_y);
  }
  return objects;
}

double OpenvinoInfer::sigmoid(double x)
{
  if (x > 0.0) {
    return 1.0 / (1.0 + std::exp(-x));
  }
  return std::exp(x) / (1.0 + std::exp(x));
}

}  // namespace aim_armor_detector
