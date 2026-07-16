#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "DxImageProc.h"
#include "GxAPI.h"

namespace
{

using SteadyClock = std::chrono::steady_clock;

struct Options
{
  std::string mode;
  std::string topic{"/gx_camera_0/image_raw"};
  std::string device_sn;
  int device_index{1};
  double duration_seconds{10.0};
};

void print_usage(const char * executable)
{
  std::cerr << "Usage:\n"
            << "  " << executable << " --mode topic [--topic TOPIC] [--duration SECONDS]\n"
            << "  " << executable
            << " --mode sdk [--device-sn SERIAL | --device-index INDEX] [--duration SECONDS]\n\n"
            << "topic: measure received image rate with a serialized C++ DDS subscriber.\n"
            << "sdk:   run the driver's Raw8-to-BGR and DDS-publish pipeline against the camera. "
            << "Stop the camera driver before using this mode.\n";
}

bool parse_double(const char * value, double & output)
{
  char * end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0' || parsed <= 0.0) {
    return false;
  }
  output = parsed;
  return true;
}

bool parse_int(const char * value, int & output)
{
  char * end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (end == value || *end != '\0' || parsed < 1 ||
    parsed > std::numeric_limits<int>::max())
  {
    return false;
  }
  output = static_cast<int>(parsed);
  return true;
}

bool parse_options(int argc, char ** argv, Options & options)
{
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help" || argument == "-h") {
      return false;
    }
    if (index + 1 >= argc) {
      std::cerr << "missing value for " << argument << '\n';
      return false;
    }

    const char * value = argv[++index];
    if (argument == "--mode") {
      options.mode = value;
    } else if (argument == "--topic") {
      options.topic = value;
    } else if (argument == "--device-sn") {
      options.device_sn = value;
    } else if (argument == "--device-index") {
      if (!parse_int(value, options.device_index)) {
        std::cerr << "invalid device index: " << value << '\n';
        return false;
      }
    } else if (argument == "--duration") {
      if (!parse_double(value, options.duration_seconds)) {
        std::cerr << "invalid duration: " << value << '\n';
        return false;
      }
    } else {
      std::cerr << "unknown argument: " << argument << '\n';
      return false;
    }
  }

  if (options.mode != "topic" && options.mode != "sdk") {
    std::cerr << "--mode must be topic or sdk\n";
    return false;
  }
  return true;
}

struct IntervalStats
{
  std::uint64_t count{0};
  std::uint64_t bytes{0};
  double total_gap_seconds{0.0};
  double min_gap_seconds{std::numeric_limits<double>::infinity()};
  double max_gap_seconds{0.0};

  void record_gap(double seconds)
  {
    total_gap_seconds += seconds;
    min_gap_seconds = std::min(min_gap_seconds, seconds);
    max_gap_seconds = std::max(max_gap_seconds, seconds);
  }

  void reset()
  {
    *this = IntervalStats{};
  }
};

struct TimingStats
{
  std::uint64_t count{0};
  double total_milliseconds{0.0};
  double max_milliseconds{0.0};

  void record(SteadyClock::time_point started_at, SteadyClock::time_point finished_at)
  {
    const double milliseconds =
      std::chrono::duration<double, std::milli>(finished_at - started_at).count();
    ++count;
    total_milliseconds += milliseconds;
    max_milliseconds = std::max(max_milliseconds, milliseconds);
  }

  double average_milliseconds() const
  {
    return count == 0U ? 0.0 : total_milliseconds / static_cast<double>(count);
  }
};

class TopicProbe : public rclcpp::Node
{
public:
  TopicProbe(const std::string & topic, double duration_seconds)
  : Node("gx_camera_rate_probe"),
    duration_(std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(duration_seconds))),
    started_at_(SteadyClock::now()),
    window_started_at_(started_at_)
  {
    subscription_ = create_generic_subscription(
      topic, "sensor_msgs/msg/Image", rclcpp::SensorDataQoS(),
      [this](std::shared_ptr<rclcpp::SerializedMessage> message) { on_image(*message); });
    report_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });

    RCLCPP_INFO(
      get_logger(), "[GX_RATE_PROBE] topic mode: topic=%s duration=%.1fs",
      topic.c_str(), duration_seconds);
  }

private:
  static double seconds_between(SteadyClock::time_point later, SteadyClock::time_point earlier)
  {
    return std::chrono::duration<double>(later - earlier).count();
  }

  void on_image(const rclcpp::SerializedMessage & message)
  {
    const auto received_at = SteadyClock::now();
    ++window_arrival_.count;
    window_arrival_.bytes += message.size();

    if (last_received_at_.has_value()) {
      window_arrival_.record_gap(seconds_between(received_at, *last_received_at_));
    }
    last_received_at_ = received_at;
  }

  void print_gap(const char * name, const IntervalStats & stats) const
  {
    if (!std::isfinite(stats.min_gap_seconds)) {
      std::cout << ' ' << name << "_gap=n/a";
      return;
    }
    std::cout << ' ' << name << "_gap_ms=[" << std::fixed << std::setprecision(1)
              << stats.min_gap_seconds * 1000.0 << ',' << stats.max_gap_seconds * 1000.0
              << "] mean=" << (stats.total_gap_seconds / static_cast<double>(stats.count - 1)) * 1000.0;
  }

  void report()
  {
    const auto now = SteadyClock::now();
    const double elapsed = seconds_between(now, window_started_at_);
    const double rate = elapsed > 0.0 ? static_cast<double>(window_arrival_.count) / elapsed : 0.0;
    const double bandwidth_mib = elapsed > 0.0 ?
      static_cast<double>(window_arrival_.bytes) / elapsed / 1024.0 / 1024.0 : 0.0;

    std::cout << "[GX_RATE_PROBE] topic rx_hz=" << std::fixed << std::setprecision(2) << rate
              << " payload_mib_s=" << bandwidth_mib;
    print_gap("arrival", window_arrival_);
    std::cout << std::endl;

    window_arrival_.reset();
    window_started_at_ = now;
    if (now - started_at_ >= duration_) {
      rclcpp::shutdown();
    }
  }

  SteadyClock::duration duration_;
  SteadyClock::time_point started_at_;
  SteadyClock::time_point window_started_at_;
  std::optional<SteadyClock::time_point> last_received_at_;
  IntervalStats window_arrival_;
  rclcpp::GenericSubscription::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

void print_sdk_enum(GX_DEV_HANDLE device, const char * name, GX_FEATURE_ID_CMD feature)
{
  bool implemented = false;
  if (GXIsImplemented(device, feature, &implemented) != GX_STATUS_SUCCESS || !implemented) {
    std::cout << "[GX_RATE_PROBE] sdk " << name << "=not-supported\n";
    return;
  }

  int64_t value = 0;
  const GX_STATUS status = GXGetEnum(device, feature, &value);
  std::cout << "[GX_RATE_PROBE] sdk " << name << '='
            << (status == GX_STATUS_SUCCESS ? std::to_string(value) : "read-error") << '\n';
}

void print_sdk_float(GX_DEV_HANDLE device, const char * name, GX_FEATURE_ID_CMD feature)
{
  bool implemented = false;
  if (GXIsImplemented(device, feature, &implemented) != GX_STATUS_SUCCESS || !implemented) {
    std::cout << "[GX_RATE_PROBE] sdk " << name << "=not-supported\n";
    return;
  }

  double value = 0.0;
  const GX_STATUS status = GXGetFloat(device, feature, &value);
  std::cout << "[GX_RATE_PROBE] sdk " << name << '=';
  if (status == GX_STATUS_SUCCESS) {
    std::cout << std::fixed << std::setprecision(3) << value;
  } else {
    std::cout << "read-error";
  }
  std::cout << '\n';
}

bool get_bayer_filter(GX_DEV_HANDLE device, DX_PIXEL_COLOR_FILTER & bayer_filter)
{
  int64_t gx_filter = 0;
  if (GXGetEnum(device, GX_ENUM_PIXEL_COLOR_FILTER, &gx_filter) != GX_STATUS_SUCCESS) {
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

void print_timing(const char * name, const TimingStats & stats)
{
  std::cout << ' ' << name << "_ms(mean/max)=" << std::fixed << std::setprecision(3)
            << stats.average_milliseconds() << '/' << stats.max_milliseconds;
}

int run_sdk_probe(const Options & options)
{
  GX_STATUS status = GXInitLib();
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "[GX_RATE_PROBE] GXInitLib failed: " << status << '\n';
    return 1;
  }

  GX_DEV_HANDLE device = nullptr;
  bool stream_started = false;
  const auto cleanup = [&]() {
      if (stream_started) {
        GXStreamOff(device);
      }
      if (device != nullptr) {
        GXCloseDevice(device);
      }
      GXCloseLib();
    };

  uint32_t device_count = 0;
  status = GXUpdateDeviceList(&device_count, 1000);
  if (status != GX_STATUS_SUCCESS || device_count == 0U) {
    std::cerr << "[GX_RATE_PROBE] no camera available: status=" << status
              << " count=" << device_count << '\n';
    cleanup();
    return 1;
  }

  GX_OPEN_PARAM open_param{};
  open_param.accessMode = GX_ACCESS_EXCLUSIVE;
  std::string index_string;
  if (options.device_sn.empty()) {
    index_string = std::to_string(options.device_index);
    open_param.openMode = GX_OPEN_INDEX;
    open_param.pszContent = index_string.data();
  } else {
    open_param.openMode = GX_OPEN_SN;
    open_param.pszContent = const_cast<char *>(options.device_sn.c_str());
  }

  status = GXOpenDevice(&open_param, &device);
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "[GX_RATE_PROBE] GXOpenDevice failed: " << status
              << ". Stop gx_camera_node before sdk mode.\n";
    cleanup();
    return 1;
  }

  print_sdk_enum(device, "AcquisitionFrameRateMode", GX_ENUM_ACQUISITION_FRAME_RATE_MODE);
  print_sdk_float(device, "AcquisitionFrameRate", GX_FLOAT_ACQUISITION_FRAME_RATE);
  print_sdk_float(device, "CurrentAcquisitionFrameRate", GX_FLOAT_CURRENT_ACQUISITION_FRAME_RATE);
  print_sdk_float(device, "ExposureTime", GX_FLOAT_EXPOSURE_TIME);
  print_sdk_enum(device, "ExposureAuto", GX_ENUM_EXPOSURE_AUTO);
  print_sdk_enum(device, "TriggerMode", GX_ENUM_TRIGGER_MODE);

  DX_PIXEL_COLOR_FILTER bayer_filter = BAYERBG;
  if (!get_bayer_filter(device, bayer_filter)) {
    std::cerr << "[GX_RATE_PROBE] could not read a supported Bayer color filter\n";
    cleanup();
    return 1;
  }

  const auto node = std::make_shared<rclcpp::Node>("gx_camera_rate_probe_sdk");
  const auto image_pub = node->create_publisher<sensor_msgs::msg::Image>(
    options.topic, rclcpp::SensorDataQoS());
  RCLCPP_INFO(
    node->get_logger(), "[GX_RATE_PROBE] sdk pipeline publish topic=%s", options.topic.c_str());
  const auto discovery_deadline = SteadyClock::now() + std::chrono::seconds(3);
  while (image_pub->get_subscription_count() == 0U && SteadyClock::now() < discovery_deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (image_pub->get_subscription_count() == 0U) {
    RCLCPP_WARN(
      node->get_logger(),
      "[GX_RATE_PROBE] no subscriber matched; publish timing excludes DDS delivery work");
  }

  status = GXSetAcqusitionBufferNumber(device, 16);
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "[GX_RATE_PROBE] GXSetAcqusitionBufferNumber failed: " << status << '\n';
    cleanup();
    return 1;
  }
  status = GXStreamOn(device);
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "[GX_RATE_PROBE] GXStreamOn failed: " << status << '\n';
    cleanup();
    return 1;
  }
  stream_started = true;

  const auto start = SteadyClock::now();
  std::uint64_t successful_frames = 0;
  std::uint64_t incomplete_frames = 0;
  std::uint64_t frame_id_gaps = 0;
  std::optional<uint64_t> last_frame_id;
  TimingStats dequeue_timing;
  TimingStats allocation_timing;
  TimingStats bayer_timing;
  TimingStats channel_swap_timing;
  TimingStats publish_timing;
  while (SteadyClock::now() - start < std::chrono::duration<double>(options.duration_seconds)) {
    PGX_FRAME_BUFFER frame = nullptr;
    const auto dequeue_started_at = SteadyClock::now();
    status = GXDQBuf(device, &frame, 1000);
    dequeue_timing.record(dequeue_started_at, SteadyClock::now());
    if (status != GX_STATUS_SUCCESS || frame == nullptr) {
      std::cerr << "[GX_RATE_PROBE] GXDQBuf failed: " << status << '\n';
      continue;
    }

    if (frame->nStatus == GX_FRAME_STATUS_SUCCESS) {
      ++successful_frames;
      if (last_frame_id.has_value() && frame->nFrameID > *last_frame_id + 1U) {
        frame_id_gaps += frame->nFrameID - *last_frame_id - 1U;
      }
      last_frame_id = frame->nFrameID;

      const auto allocation_started_at = SteadyClock::now();
      auto message = std::make_unique<sensor_msgs::msg::Image>();
      message->height = static_cast<uint32_t>(frame->nHeight);
      message->width = static_cast<uint32_t>(frame->nWidth);
      message->encoding = "bgr8";
      message->is_bigendian = false;
      message->step = static_cast<uint32_t>(frame->nWidth * 3);
      message->data.resize(static_cast<size_t>(message->step) * message->height);
      allocation_timing.record(allocation_started_at, SteadyClock::now());

      const auto bayer_started_at = SteadyClock::now();
      DxRaw8toRGB24(
        static_cast<unsigned char *>(frame->pImgBuf), message->data.data(),
        frame->nWidth, frame->nHeight, RAW2RGB_NEIGHBOUR, bayer_filter, false);
      bayer_timing.record(bayer_started_at, SteadyClock::now());

      const auto channel_swap_started_at = SteadyClock::now();
      for (size_t index = 0; index < message->data.size(); index += 3) {
        std::swap(message->data[index], message->data[index + 2]);
      }
      channel_swap_timing.record(channel_swap_started_at, SteadyClock::now());

      GXQBuf(device, frame);
      frame = nullptr;

      message->header.stamp = node->now();
      const auto publish_started_at = SteadyClock::now();
      image_pub->publish(std::move(message));
      publish_timing.record(publish_started_at, SteadyClock::now());
    } else {
      ++incomplete_frames;
    }
    if (frame != nullptr) {
      GXQBuf(device, frame);
    }
  }

  const double elapsed = std::chrono::duration<double>(SteadyClock::now() - start).count();
  std::cout << "[GX_RATE_PROBE] sdk pipeline_hz=" << std::fixed << std::setprecision(2)
            << static_cast<double>(successful_frames) / elapsed
            << " frames=" << successful_frames
            << " incomplete=" << incomplete_frames
            << " frame_id_gaps=" << frame_id_gaps
            << " matched_subscribers=" << image_pub->get_subscription_count() << '\n';
  std::cout << "[GX_RATE_PROBE] sdk timing";
  print_timing("dequeue", dequeue_timing);
  print_timing("allocation", allocation_timing);
  print_timing("bayer", bayer_timing);
  print_timing("channel_swap", channel_swap_timing);
  print_timing("publish", publish_timing);
  std::cout << '\n';
  cleanup();
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    print_usage(argv[0]);
    return 0;
  }

  Options options;
  if (!parse_options(argc, argv, options)) {
    print_usage(argv[0]);
    return 2;
  }

  rclcpp::init(argc, argv);
  int exit_code = 0;
  if (options.mode == "sdk") {
    exit_code = run_sdk_probe(options);
  } else {
    rclcpp::spin(std::make_shared<TopicProbe>(options.topic, options.duration_seconds));
  }
  rclcpp::shutdown();
  return exit_code;
}
