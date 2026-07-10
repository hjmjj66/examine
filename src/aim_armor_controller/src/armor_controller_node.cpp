#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "aim_armor_controller/legacy_fire_control.hpp"
#include "aim_armor_controller/legacy_target_model.hpp"
#include "aim_armor_controller/legacy_timing.hpp"
#include "aim_armor_controller/target_state_selection.hpp"
#include "aim_msgs/msg/armor.hpp"
#include "aim_msgs/msg/outpost_state.hpp"
#include "aim_msgs/msg/selected_target_id.hpp"
#include "aim_msgs/msg/target_state_array.hpp"
#include "gimbal_driver/msg/bullet_info.hpp"
#include "gimbal_driver/msg/fire_code.hpp"
#include "gimbal_driver/msg/gimbal_angles.hpp"
#include "aim_target_msgs/msg/aim_result.hpp"

namespace
{

constexpr double kPi = aim_armor_controller::kPi;
constexpr double kGravity = 9.794;
constexpr double kCd = 0.42;
constexpr double kRho = 1.169;
constexpr std::uint8_t kDefaultOutpostId = 6;

struct GimbalState
{
  float yaw_deg{0.0F};
  float pitch_deg{0.0F};
  bool valid{false};
};

struct TargetStateArrayCache
{
  aim_msgs::msg::TargetStateArray msg;
  bool valid{false};
};

struct OutpostStateCache
{
  aim_msgs::msg::OutpostState msg;
  bool valid{false};
};

struct SelectedTargetIdCache
{
  aim_msgs::msg::SelectedTargetId msg;
  bool valid{false};
};

struct ActiveTarget
{
  aim_armor_controller::LegacyTargetModel model;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  std::string source_frame{"gimbal_world"};
  double measurement_age_sec{0.0};
  bool is_outpost{false};
  bool valid{false};
};

gimbal_driver::msg::FireCode makeFireCodeMsg(const rclcpp::Time & stamp, std::uint8_t raw)
{
  gimbal_driver::msg::FireCode msg;
  msg.header.stamp = stamp;
  msg.field_mask = gimbal_driver::msg::FireCode::FIELD_ALL;
  msg.fire_status = static_cast<std::uint8_t>(raw & 0x03U);
  msg.cap_state = static_cast<std::uint8_t>((raw >> 2U) & 0x03U);
  msg.follow_mode = ((raw >> 4U) & 0x01U) != 0U;
  msg.aim_mode = ((raw >> 5U) & 0x01U) != 0U;
  msg.rotate = static_cast<std::uint8_t>((raw >> 6U) & 0x03U);
  msg.raw = raw;
  return msg;
}

struct ArmorCandidate
{  double bx{0.0};
  double by{0.0};
  double bz{0.0};
  double pitch{0.0};
  double yaw{0.0};
  double time{0.0};
  double pitch_setpoint_deg{0.0};
  double yaw_setpoint_deg{0.0};
  double pitch_actual_want_deg{0.0};
  double yaw_actual_want_deg{0.0};
  double angle_error{0.0};
  double source_x{0.0};
  double source_y{0.0};
  double source_z{0.0};
  int armor_index{-1};
};

class ArmorControllerNode : public rclcpp::Node
{
public:
  ArmorControllerNode()
  : Node("armor_controller_node")
  {
    declare_parameter<std::string>(
      "front_0_target_states_topic", "/aim_predictor/front_0/target_states");
    declare_parameter<std::string>(
      "front_1_target_states_topic", "/aim_predictor/front_1/target_states");
    declare_parameter<std::string>(
      "back_target_states_topic", "/aim_predictor/back/target_states");
    declare_parameter<std::string>(
      "outpost_state_topic", "/aim_outpost_predictor/outpost_state");
    declare_parameter<std::string>(
      "selected_target_id_topic", "/decider/selected_target_id");
    declare_parameter<std::string>("aim_result_topic", "/ly/aim/result");
    declare_parameter<std::string>("gimbal_angles_topic", "/ly/gimbal/angles");
    declare_parameter<std::string>("bullet_speed_topic", "/ly/game/bullet");
    declare_parameter<std::string>("control_angles_topic", "/ly/control/angles");
    declare_parameter<std::string>("control_firecode_topic", "/ly/control/firecode");
    declare_parameter<std::string>("selected_armor_topic", "/controller/selected_armor");
    declare_parameter<std::string>(
      "debug_target_point_topic", "/armor_controller/debug/target_point_barrel");
    declare_parameter<std::string>(
      "debug_selected_armor_index_topic", "/armor_controller/debug/selected_armor_index");
    declare_parameter<bool>("publish_legacy_control_topics", true);
    declare_parameter<bool>("manual_fire_mode", false);
    declare_parameter<double>("publish_rate_hz", 100.0);
    declare_parameter<double>("fire_rate_hz", 10.0);
    declare_parameter<double>("bullet_speed_mps", 23.0);
    declare_parameter<double>("min_bullet_speed_mps", 20.0);
    declare_parameter<double>("bullet_speed_alpha", 0.5);
    declare_parameter<double>("tol_deltax_m", 0.2);
    declare_parameter<double>("tol_deltay_m", 0.1);
    declare_parameter<double>("bullet_mass_kg", 3.2e-3);
    declare_parameter<double>("bullet_diameter_m", 16.8e-3);
    declare_parameter<int>("max_iter", 100);
    declare_parameter<double>("tol", 1e-6);
    declare_parameter<std::string>("barrel_joint_frame", "gimbal_barrel_joint");
    declare_parameter<double>("target_tf_timeout_sec", 0.02);
    declare_parameter<double>("target_msg_timeout_sec", 0.10);
    declare_parameter<double>("armor_select_area_weight", 1.0);
    declare_parameter<double>("armor_select_angle_weight", 1.0);
    declare_parameter<bool>("include_processing_delay", true);
    declare_parameter<double>("system_response_time_sec", 0.0);
    declare_parameter<int>("max_prediction_iterations", 10);
    declare_parameter<double>("fly_time_converge_threshold_sec", 0.001);
    declare_parameter<double>("max_processing_delay_sec", 0.2);
    declare_parameter<double>("max_predict_time_sec", 1.0);
    declare_parameter<double>("shoot_yaw_tolerance_deg", 1.0);
    declare_parameter<double>("shoot_pitch_tolerance_deg", 1.0);
    declare_parameter<double>("shoot_face_tolerance_deg", 2.0);
    declare_parameter<double>("spin_center_aim_angular_velocity_threshold", 2.0);
    declare_parameter<bool>("enable_smart_selector", true);
    declare_parameter<double>("comming_angle_deg", 60.0);
    declare_parameter<double>("leaving_angle_deg", 20.0);
    declare_parameter<double>("smart_selector_max_angular_velocity", 2.0);
    declare_parameter<double>("outpost_shoot_yaw_gate_deg", 45.0);

    const std::string st = "controller_config.shoot_table_adjust.";
    declare_parameter<bool>(st + "enable", false);
    const char * names[] = {
      "intercept", "coef_z", "coef_d", "coef_z2", "coef_zd", "coef_d2"};
    for (const char * n : names) {
      declare_parameter<double>(st + "pitch." + n, 0.0);
      declare_parameter<double>(st + "yaw." + n, 0.0);
    }

    const std::string toff = "controller_config.target_offset.";
    declare_parameter<double>(toff + "x", 0.0);
    declare_parameter<double>(toff + "y", 0.0);
    declare_parameter<double>(toff + "z", 0.0);

    const auto front_0_topic = get_parameter("front_0_target_states_topic").as_string();
    const auto front_1_topic = get_parameter("front_1_target_states_topic").as_string();
    const auto back_topic = get_parameter("back_target_states_topic").as_string();
    const auto outpost_topic = get_parameter("outpost_state_topic").as_string();
    const auto selected_target_topic = get_parameter("selected_target_id_topic").as_string();
    const auto aim_result_topic = get_parameter("aim_result_topic").as_string();
    const auto gimbal_topic = get_parameter("gimbal_angles_topic").as_string();
    const auto bullet_topic = get_parameter("bullet_speed_topic").as_string();
    const auto control_angles_topic = get_parameter("control_angles_topic").as_string();
    const auto control_firecode_topic = get_parameter("control_firecode_topic").as_string();
    const auto selected_armor_topic = get_parameter("selected_armor_topic").as_string();
    const auto debug_target_point_topic = get_parameter("debug_target_point_topic").as_string();
    const auto debug_selected_armor_index_topic =
      get_parameter("debug_selected_armor_index_topic").as_string();

    publish_legacy_control_topics_ = get_parameter("publish_legacy_control_topics").as_bool();
    manual_fire_mode_ = get_parameter("manual_fire_mode").as_bool();
    const double publish_rate = get_parameter("publish_rate_hz").as_double();
    fire_rate_hz_ = get_parameter("fire_rate_hz").as_double();
    bullet_speed_ = get_parameter("bullet_speed_mps").as_double();
    min_bullet_speed_ = get_parameter("min_bullet_speed_mps").as_double();
    bullet_speed_alpha_ = get_parameter("bullet_speed_alpha").as_double();
    tol_deltax_ = get_parameter("tol_deltax_m").as_double();
    tol_deltay_ = get_parameter("tol_deltay_m").as_double();
    bullet_mass_ = get_parameter("bullet_mass_kg").as_double();
    bullet_diameter_ = get_parameter("bullet_diameter_m").as_double();
    max_iter_ = get_parameter("max_iter").as_int();
    tol_ = get_parameter("tol").as_double();
    barrel_joint_frame_ = get_parameter("barrel_joint_frame").as_string();
    target_tf_timeout_sec_ = get_parameter("target_tf_timeout_sec").as_double();
    target_msg_timeout_sec_ = get_parameter("target_msg_timeout_sec").as_double();
    armor_select_area_weight_ = get_parameter("armor_select_area_weight").as_double();
    armor_select_angle_weight_ = get_parameter("armor_select_angle_weight").as_double();
    include_processing_delay_ = get_parameter("include_processing_delay").as_bool();
    system_response_time_sec_ = get_parameter("system_response_time_sec").as_double();
    max_prediction_iterations_ = get_parameter("max_prediction_iterations").as_int();
    fly_time_converge_threshold_sec_ =
      get_parameter("fly_time_converge_threshold_sec").as_double();
    max_processing_delay_sec_ = get_parameter("max_processing_delay_sec").as_double();
    max_predict_time_sec_ = get_parameter("max_predict_time_sec").as_double();
    shoot_yaw_tolerance_deg_ = get_parameter("shoot_yaw_tolerance_deg").as_double();
    shoot_pitch_tolerance_deg_ = get_parameter("shoot_pitch_tolerance_deg").as_double();
    shoot_face_tolerance_rad_ =
      get_parameter("shoot_face_tolerance_deg").as_double() * kPi / 180.0;
    spin_center_aim_angular_velocity_threshold_ =
      get_parameter("spin_center_aim_angular_velocity_threshold").as_double();
    enable_smart_selector_ = get_parameter("enable_smart_selector").as_bool();
    comming_angle_rad_ = get_parameter("comming_angle_deg").as_double() * kPi / 180.0;
    leaving_angle_rad_ = get_parameter("leaving_angle_deg").as_double() * kPi / 180.0;
    smart_selector_max_angular_velocity_ =
      get_parameter("smart_selector_max_angular_velocity").as_double();
    outpost_shoot_yaw_gate_rad_ =
      get_parameter("outpost_shoot_yaw_gate_deg").as_double() * kPi / 180.0;

    loadShootTableParams();
    x_offset_ = get_parameter(toff + "x").as_double();
    y_offset_ = get_parameter(toff + "y").as_double();
    z_offset_ = get_parameter(toff + "z").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    aim_result_pub_ = create_publisher<aim_target_msgs::msg::AimResult>(aim_result_topic, 10);
    selected_armor_pub_ =
      create_publisher<aim_msgs::msg::Armor>(selected_armor_topic, rclcpp::SensorDataQoS());
    debug_target_point_pub_ =
      create_publisher<geometry_msgs::msg::PointStamped>(debug_target_point_topic, 10);
    debug_selected_armor_index_pub_ =
      create_publisher<std_msgs::msg::Int32>(debug_selected_armor_index_topic, 10);
    if (publish_legacy_control_topics_) {
      angle_pub_ = create_publisher<gimbal_driver::msg::GimbalAngles>(control_angles_topic, 10);
      if (!manual_fire_mode_) {
        fire_pub_ = create_publisher<gimbal_driver::msg::FireCode>(control_firecode_topic, 10);
      }
    }

    front_0_sub_ = create_subscription<aim_msgs::msg::TargetStateArray>(
      front_0_topic, rclcpp::SensorDataQoS(),
      [this](const aim_msgs::msg::TargetStateArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (msg != nullptr) {
          front_0_cache_.msg = *msg;
          front_0_cache_.valid = true;
        } else {
          front_0_cache_.valid = false;
        }
      });
    front_1_sub_ = create_subscription<aim_msgs::msg::TargetStateArray>(
      front_1_topic, rclcpp::SensorDataQoS(),
      [this](const aim_msgs::msg::TargetStateArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (msg != nullptr) {
          front_1_cache_.msg = *msg;
          front_1_cache_.valid = true;
        } else {
          front_1_cache_.valid = false;
        }
      });
    back_sub_ = create_subscription<aim_msgs::msg::TargetStateArray>(
      back_topic, rclcpp::SensorDataQoS(),
      [this](const aim_msgs::msg::TargetStateArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (msg != nullptr) {
          back_cache_.msg = *msg;
          back_cache_.valid = true;
        } else {
          back_cache_.valid = false;
        }
      });
    outpost_sub_ = create_subscription<aim_msgs::msg::OutpostState>(
      outpost_topic, rclcpp::SensorDataQoS(),
      [this](const aim_msgs::msg::OutpostState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (msg != nullptr) {
          outpost_cache_.msg = *msg;
          outpost_cache_.valid = true;
        } else {
          outpost_cache_.valid = false;
        }
      });
    selected_target_sub_ = create_subscription<aim_msgs::msg::SelectedTargetId>(
      selected_target_topic, rclcpp::SensorDataQoS(),
      [this](const aim_msgs::msg::SelectedTargetId::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (msg != nullptr) {
          selected_target_cache_.msg = *msg;
          selected_target_cache_.valid = msg->valid;
        } else {
          selected_target_cache_.valid = false;
        }
      });

    gimbal_sub_ = create_subscription<gimbal_driver::msg::GimbalAngles>(
      gimbal_topic, 10,
      [this](const gimbal_driver::msg::GimbalAngles::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        gimbal_state_.yaw_deg = msg->yaw;
        gimbal_state_.pitch_deg = msg->pitch;
        gimbal_state_.valid = true;
      });
    bullet_sub_ = create_subscription<gimbal_driver::msg::BulletInfo>(
      bullet_topic, 10,
      [this](const gimbal_driver::msg::BulletInfo::SharedPtr msg) {
        if (msg->has_initial_speed && msg->initial_speed > min_bullet_speed_) {
          if (bullet_speed_ <= 0.0) {
            bullet_speed_ = msg->initial_speed;
          } else {
            bullet_speed_ =
              msg->initial_speed * bullet_speed_alpha_ +
              (1.0 - bullet_speed_alpha_) * bullet_speed_;
          }
        }
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&ArmorControllerNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "armor_controller_node started. front_0=%s front_1=%s back=%s outpost=%s "
      "selected_id=%s aim_result=%s gimbal=%s bullet=%s frame=%s",
      front_0_topic.c_str(), front_1_topic.c_str(), back_topic.c_str(), outpost_topic.c_str(),
      selected_target_topic.c_str(), aim_result_topic.c_str(), gimbal_topic.c_str(),
      bullet_topic.c_str(), barrel_joint_frame_.c_str());
  }

private:
  void loadShootTableParams()
  {
    const std::string st = "controller_config.shoot_table_adjust.";
    shoot_table_adjust_ = get_parameter(st + "enable").as_bool();
    if (!shoot_table_adjust_) {
      return;
    }
    pitch_param_.assign(6, 0.0);
    yaw_param_.assign(6, 0.0);
    const char * keys[] = {
      "intercept", "coef_z", "coef_d", "coef_z2", "coef_zd", "coef_d2"};
    for (std::size_t i = 0; i < 6; ++i) {
      pitch_param_[i] = get_parameter(st + "pitch." + keys[i]).as_double();
      yaw_param_[i] = get_parameter(st + "yaw." + keys[i]).as_double();
    }
  }

  double fitPitch(double z_height, double horizontal_distance) const
  {
    if (!shoot_table_adjust_ || pitch_param_.size() < 6) {
      return 0.0;
    }
    const double intercept = pitch_param_[0];
    const double coef_z = pitch_param_[1];
    const double coef_d = pitch_param_[2];
    const double coef_z2 = pitch_param_[3];
    const double coef_zd = pitch_param_[4];
    const double coef_d2 = pitch_param_[5];
    const double z2 = z_height * z_height;
    const double d2 = horizontal_distance * horizontal_distance;
    const double zd = z_height * horizontal_distance;
    return intercept + coef_z * z_height + coef_d * horizontal_distance + coef_z2 * z2 +
      coef_zd * zd + coef_d2 * d2;
  }

  double fitYawTable(double z_height, double horizontal_distance) const
  {
    if (!shoot_table_adjust_ || yaw_param_.size() < 6) {
      return 0.0;
    }
    const double intercept = yaw_param_[0];
    const double coef_z = yaw_param_[1];
    const double coef_d = yaw_param_[2];
    const double coef_z2 = yaw_param_[3];
    const double coef_zd = yaw_param_[4];
    const double coef_d2 = yaw_param_[5];
    const double z2 = z_height * z_height;
    const double d2 = horizontal_distance * horizontal_distance;
    const double zd = z_height * horizontal_distance;
    return intercept + coef_z * z_height + coef_d * horizontal_distance + coef_z2 * z2 +
      coef_zd * zd + coef_d2 * d2;
  }

  bool calcPitchYaw(
    double & pitch, double & yaw, double & time,
    double target_x, double target_y, double target_z)
  {
    if (
      !std::isfinite(target_x) || !std::isfinite(target_y) || !std::isfinite(target_z) ||
      !std::isfinite(bullet_speed_) || !std::isfinite(bullet_mass_) ||
      !std::isfinite(bullet_diameter_) || bullet_speed_ <= 0.0 ||
      bullet_mass_ <= 0.0 || bullet_diameter_ <= 0.0 || max_iter_ <= 0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "calcPitchYaw: invalid ballistic inputs speed=%.6f mass=%.6f diameter=%.6f iter=%d "
        "target=(%.6f, %.6f, %.6f)",
        bullet_speed_, bullet_mass_, bullet_diameter_, max_iter_,
        target_x, target_y, target_z);
      return false;
    }

    const double distance = std::sqrt(target_x * target_x + target_y * target_y);
    if (!std::isfinite(distance)) {
      return false;
    }

    double theta = std::atan2(target_z, distance);
    double delta_z = 0.0;
    const double k1 = kCd * kRho * (kPi * bullet_diameter_ * bullet_diameter_) / 8.0 / bullet_mass_;
    if (!std::isfinite(theta) || !std::isfinite(k1) || std::abs(k1) < 1e-12) {
      return false;
    }

    for (int i = 0; i < max_iter_; ++i) {
      const double cth = std::cos(theta);
      if (!std::isfinite(cth) || std::abs(cth) < 1e-8) {
        return false;
      }
      const double t = (std::exp(k1 * distance) - 1.0) / (k1 * bullet_speed_ * cth);
      if (!std::isfinite(t)) {
        return false;
      }
      delta_z = target_z - bullet_speed_ * std::sin(theta) * t / cth +
        0.5 * kGravity * t * t / cth / cth;
      if (!std::isfinite(delta_z)) {
        return false;
      }
      if (std::abs(delta_z) < tol_) {
        time = t;
        break;
      }
      const double denominator =
        (-(bullet_speed_ * t) / (cth * cth) +
        kGravity * t * t / (bullet_speed_ * bullet_speed_) * std::sin(theta) /
        (cth * cth * cth));
      if (!std::isfinite(denominator) || std::abs(denominator) < 1e-12) {
        return false;
      }
      theta -= delta_z / denominator;
    }

    if (!std::isfinite(theta) || !std::isfinite(delta_z) || std::abs(delta_z) > tol_) {
      return false;
    }
    pitch = theta;
    yaw = std::atan2(target_y, target_x);
    return true;
  }

  bool calcPitchYawWithShootTable(
    double & pitch, double & yaw, double & time,
    double target_x, double target_y, double target_z)
  {
    if (!calcPitchYaw(pitch, yaw, time, target_x, target_y, target_z)) {
      return false;
    }
    if (!shoot_table_adjust_) {
      return true;
    }
    const double horizontal_distance = std::sqrt(target_x * target_x + target_y * target_y);
    pitch += fitPitch(target_z, horizontal_distance) * kPi / 180.0;
    yaw += fitYawTable(target_z, horizontal_distance) * kPi / 180.0;
    return true;
  }

  void publishFallback(const GimbalState & gimbal)
  {
    aim_target_msgs::msg::AimResult aim_msg;
    aim_msg.header.stamp = now();
    aim_msg.header.frame_id = barrel_joint_frame_;
    aim_msg.fire = false;
    aim_msg.yaw = gimbal.yaw_deg;
    aim_msg.pitch = gimbal.pitch_deg;
    aim_result_pub_->publish(aim_msg);

    if (publish_legacy_control_topics_ && angle_pub_) {
      gimbal_driver::msg::GimbalAngles angle_msg;
      angle_msg.yaw = gimbal.yaw_deg;
      angle_msg.pitch = gimbal.pitch_deg;
      angle_pub_->publish(angle_msg);
    }

    if (publish_legacy_control_topics_ && !manual_fire_mode_ && fire_pub_) {
      fire_pub_->publish(makeFireCodeMsg(now(), last_firecode_out_));
    }
  }

  bool transformWorldToBarrel(
    double wx, double wy, double wz, const std::string & source_frame,
    double & x, double & y, double & z, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    bool got_tf = false;
    std::string fallback_reason;
    try {
      tf_msg = tf_buffer_->lookupTransform(
        barrel_joint_frame_, source_frame, stamp,
        rclcpp::Duration::from_seconds(target_tf_timeout_sec_));
      got_tf = true;
    } catch (const tf2::TransformException & e) {
      fallback_reason = e.what();
    }

    if (!got_tf) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "target tf lookup at stamp failed (%s -> %s): %s; falling back to latest TF",
        source_frame.c_str(), barrel_joint_frame_.c_str(), fallback_reason.c_str());
      try {
        tf_msg = tf_buffer_->lookupTransform(
          barrel_joint_frame_, source_frame, rclcpp::Time(0, 0, RCL_ROS_TIME),
          rclcpp::Duration::from_seconds(target_tf_timeout_sec_));
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "target tf lookup failed (%s -> %s): %s",
          source_frame.c_str(), barrel_joint_frame_.c_str(), e.what());
        return false;
      }
    }

    const tf2::Quaternion q(
      tf_msg.transform.rotation.x,
      tf_msg.transform.rotation.y,
      tf_msg.transform.rotation.z,
      tf_msg.transform.rotation.w);
    const tf2::Matrix3x3 rot(q);
    const tf2::Vector3 p_src(wx, wy, wz);
    const tf2::Vector3 t(
      tf_msg.transform.translation.x,
      tf_msg.transform.translation.y,
      tf_msg.transform.translation.z);
    const tf2::Vector3 p_dst = rot * p_src + t;
    x = p_dst.x();
    y = p_dst.y();
    z = p_dst.z();
    return true;
  }

  static std::string resolveSourceFrame(const std_msgs::msg::Header & header)
  {
    return header.frame_id.empty() ? std::string("gimbal_world") : header.frame_id;
  }

  bool isTimedOut(const builtin_interfaces::msg::Time & stamp) const
  {
    if (target_msg_timeout_sec_ <= 0.0 || aim_armor_controller::isZeroStamp(stamp)) {
      return false;
    }
    return (now() - rclcpp::Time(stamp)).seconds() > target_msg_timeout_sec_;
  }

  double measurementAgeFromStamp(const builtin_interfaces::msg::Time & stamp) const
  {
    if (aim_armor_controller::isZeroStamp(stamp)) {
      return 0.0;
    }
    return std::max(0.0, (now() - rclcpp::Time(stamp)).seconds());
  }

  ActiveTarget resolveActiveTarget(
    const SelectedTargetIdCache & selected_target,
    const TargetStateArrayCache & front_0,
    const TargetStateArrayCache & front_1,
    const TargetStateArrayCache & back,
    const OutpostStateCache & outpost) const
  {
    ActiveTarget active;
    if (!selected_target.valid || !selected_target.msg.valid) {
      return active;
    }

    const std::uint8_t outpost_id = outpost.valid ? outpost.msg.id : kDefaultOutpostId;
    if (selected_target.msg.id == outpost_id) {
      if (!outpost.valid || !outpost.msg.tracking || isTimedOut(outpost.msg.header.stamp)) {
        return active;
      }
      active.model = aim_armor_controller::legacyTargetModelFromOutpostState(outpost.msg);
      active.stamp = rclcpp::Time(outpost.msg.header.stamp);
      active.source_frame = resolveSourceFrame(outpost.msg.header);
      active.measurement_age_sec = measurementAgeFromStamp(outpost.msg.header.stamp);
      active.is_outpost = true;
      active.valid = true;
      return active;
    }

    const auto match = aim_armor_controller::selectBestTargetStateMatch(
      selected_target.msg.id,
      front_0.valid ? &front_0.msg : nullptr,
      front_1.valid ? &front_1.msg : nullptr,
      back.valid ? &back.msg : nullptr);
    if (!match.has_value() || isTimedOut(match->header.stamp)) {
      return active;
    }

    active.model =
      aim_armor_controller::legacyTargetModelFromTargetState(match->header, match->target);
    active.stamp = rclcpp::Time(match->header.stamp);
    active.source_frame = resolveSourceFrame(match->header);
    active.measurement_age_sec = measurementAgeFromStamp(match->header.stamp);
    active.is_outpost = false;
    active.valid = true;
    return active;
  }

  bool buildCandidateFromAimPoint(
    const geometry_msgs::msg::Point & point_world,
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    const GimbalState & gimbal,
    ArmorCandidate & candidate)
  {
    const double wx = point_world.x + x_offset_;
    const double wy = point_world.y + y_offset_;
    const double wz = point_world.z + z_offset_;
    candidate.source_x = wx;
    candidate.source_y = wy;
    candidate.source_z = wz;

    if (!transformWorldToBarrel(wx, wy, wz, source_frame, candidate.bx, candidate.by, candidate.bz, stamp)) {
      return false;
    }
    if (!calcPitchYawWithShootTable(
        candidate.pitch, candidate.yaw, candidate.time,
        candidate.bx, candidate.by, candidate.bz))
    {
      return false;
    }

    candidate.pitch_setpoint_deg = candidate.pitch * 180.0 / kPi;
    candidate.yaw_setpoint_deg =
      static_cast<double>(gimbal.yaw_deg) + candidate.yaw * 180.0 / kPi;
    candidate.pitch_actual_want_deg = candidate.pitch_setpoint_deg;
    candidate.yaw_actual_want_deg = candidate.yaw_setpoint_deg;
    if (candidate.pitch_setpoint_deg < -2.0) {
      candidate.pitch_actual_want_deg -= 1.0;
    }

    const double current_pitch = static_cast<double>(gimbal.pitch_deg) * kPi / 180.0;
    const double current_yaw = static_cast<double>(gimbal.yaw_deg) * kPi / 180.0;
    const double want_pitch_rad = candidate.pitch_actual_want_deg * kPi / 180.0;
    const double want_yaw_rad = candidate.yaw_actual_want_deg * kPi / 180.0;
    const double yaw_error = std::remainder(want_yaw_rad - current_yaw, 2.0 * kPi);
    const double pitch_error = std::remainder(want_pitch_rad - current_pitch, 2.0 * kPi);
    candidate.angle_error = std::hypot(yaw_error, pitch_error);
    return true;
  }

  bool selectArmorCandidate(
    const ActiveTarget & active_target,
    const GimbalState & gimbal,
    ArmorCandidate & selected,
    aim_armor_controller::LegacyTargetModel & best_target)
  {
    const auto & target = active_target.model;
    best_target = target;

    double center_bx = 0.0;
    double center_by = 0.0;
    double center_bz = 0.0;
    if (!transformWorldToBarrel(
        target.center.x, target.center.y, target.center.z, active_target.source_frame,
        center_bx, center_by, center_bz, active_target.stamp))
    {
      return false;
    }

    double center_pitch = 0.0;
    double center_yaw = 0.0;
    double center_time = 0.0;
    if (!calcPitchYawWithShootTable(
        center_pitch, center_yaw, center_time, center_bx, center_by, center_bz))
    {
      return false;
    }

    const aim_armor_controller::LegacyTimingInput timing_input{
      active_target.measurement_age_sec,
      center_time,
      include_processing_delay_,
      system_response_time_sec_,
    };
    const auto timing = aim_armor_controller::computeLegacyPredictTime(timing_input);
    if (
      max_processing_delay_sec_ > 0.0 &&
      timing.processing_delay_sec > max_processing_delay_sec_)
    {
      return false;
    }
    const double predict_time =
      max_predict_time_sec_ > 0.0 ?
      std::min(timing.predict_time_sec, max_predict_time_sec_) :
      timing.predict_time_sec;

    auto low_speed_selection_target = target;
    aim_armor_controller::predictLegacyTarget(low_speed_selection_target, predict_time);

    int selected_idx = -1;
    aim_armor_controller::LegacyAimCandidate final_aim;

    if (active_target.is_outpost) {
      auto predicted_target = target;
      aim_armor_controller::predictLegacyTarget(predicted_target, predict_time);
      final_aim = aim_armor_controller::chooseLegacyAimPoint(
        predicted_target, &low_speed_selection_target, lock_index_, comming_angle_rad_,
        leaving_angle_rad_);
      if (!final_aim.valid) {
        return false;
      }
      selected_idx = final_aim.armor_index;
      best_target = predicted_target;
    } else {
      const bool spin_center_aim =
        std::abs(target.angular_velocity) >
        std::abs(spin_center_aim_angular_velocity_threshold_);
      auto predicted_target = target;
      aim_armor_controller::predictLegacyTarget(predicted_target, predict_time);
      auto aim_candidate = spin_center_aim ?
        aim_armor_controller::chooseLegacySpinCenterAimPoint(
          predicted_target, shoot_face_tolerance_rad_) :
        aim_armor_controller::chooseLegacyAimPoint(
          predicted_target, &low_speed_selection_target, lock_index_, comming_angle_rad_,
          leaving_angle_rad_);
      if (!aim_candidate.valid) {
        return false;
      }

      double prev_fly_time = center_time;
      for (int iter = 0; iter < std::max(1, max_prediction_iterations_); ++iter) {
        const aim_armor_controller::LegacyTimingInput iter_timing_input{
          active_target.measurement_age_sec,
          prev_fly_time,
          include_processing_delay_,
          system_response_time_sec_,
        };
        const auto iter_timing = aim_armor_controller::computeLegacyPredictTime(iter_timing_input);
        const double candidate_predict_time =
          max_predict_time_sec_ > 0.0 ?
          std::min(iter_timing.predict_time_sec, max_predict_time_sec_) :
          iter_timing.predict_time_sec;

        auto candidate_target = target;
        aim_armor_controller::predictLegacyTarget(candidate_target, candidate_predict_time);
        aim_candidate = spin_center_aim ?
          aim_armor_controller::chooseLegacySpinCenterAimPoint(
            candidate_target, shoot_face_tolerance_rad_) :
          aim_armor_controller::chooseLegacyAimPoint(
            candidate_target, &low_speed_selection_target, lock_index_, comming_angle_rad_,
            leaving_angle_rad_);
        if (!aim_candidate.valid) {
          return false;
        }

        double aim_bx = 0.0;
        double aim_by = 0.0;
        double aim_bz = 0.0;
        if (!transformWorldToBarrel(
            aim_candidate.point_world.x, aim_candidate.point_world.y, aim_candidate.point_world.z,
            active_target.source_frame, aim_bx, aim_by, aim_bz, active_target.stamp))
        {
          return false;
        }

        double iter_pitch = 0.0;
        double iter_yaw = 0.0;
        double iter_fly_time = 0.0;
        if (!calcPitchYawWithShootTable(
            iter_pitch, iter_yaw, iter_fly_time, aim_bx, aim_by, aim_bz))
        {
          return false;
        }

        selected_idx = aim_candidate.armor_index;
        final_aim = aim_candidate;
        best_target = candidate_target;
        if (std::abs(iter_fly_time - prev_fly_time) < fly_time_converge_threshold_sec_) {
          break;
        }
        prev_fly_time = iter_fly_time;
      }
    }

    if (selected_idx < 0 || !final_aim.valid) {
      return false;
    }
    if (!buildCandidateFromAimPoint(
        final_aim.point_world, active_target.source_frame, active_target.stamp, gimbal, selected))
    {
      return false;
    }
    selected.armor_index = selected_idx;
    return true;
  }

  void publishSelectedArmorDebug(
    const ActiveTarget & active_target,
    const aim_armor_controller::LegacyTargetModel & best_target,
    int armor_index)
  {
    aim_msgs::msg::Armor selected_armor;
    selected_armor.header = best_target.header;
    if (selected_armor.header.frame_id.empty()) {
      selected_armor.header.frame_id = active_target.source_frame;
    }

    const auto armors = aim_armor_controller::buildLegacyArmors(best_target);
    geometry_msgs::msg::Point center = best_target.center;
    if (armor_index >= 0 && static_cast<std::size_t>(armor_index) < armors.size()) {
      center = armors[static_cast<std::size_t>(armor_index)].position;
    }
    for (auto & corner : selected_armor.corners) {
      corner = center;
    }
    selected_armor_pub_->publish(selected_armor);
  }

  void onTimer()
  {
    TargetStateArrayCache front_0;
    TargetStateArrayCache front_1;
    TargetStateArrayCache back;
    OutpostStateCache outpost;
    SelectedTargetIdCache selected_target;
    GimbalState gimbal;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      front_0 = front_0_cache_;
      front_1 = front_1_cache_;
      back = back_cache_;
      outpost = outpost_cache_;
      selected_target = selected_target_cache_;
      gimbal = gimbal_state_;
    }

    if (!gimbal.valid) {
      return;
    }

    const auto active_target =
      resolveActiveTarget(selected_target, front_0, front_1, back, outpost);
    if (!active_target.valid) {
      publishFallback(gimbal);
      return;
    }

    ArmorCandidate selected;
    aim_armor_controller::LegacyTargetModel best_target;
    if (!selectArmorCandidate(active_target, gimbal, selected, best_target)) {
      publishFallback(gimbal);
      return;
    }

    const bool spin_center_aim =
      !active_target.is_outpost &&
      std::abs(active_target.model.angular_velocity) >
      std::abs(spin_center_aim_angular_velocity_threshold_);

    const double current_pitch = static_cast<double>(gimbal.pitch_deg) * kPi / 180.0;
    const double current_yaw = static_cast<double>(gimbal.yaw_deg) * kPi / 180.0;
    const double want_pitch_rad = selected.pitch_actual_want_deg * kPi / 180.0;
    const double want_yaw_rad = selected.yaw_actual_want_deg * kPi / 180.0;
    const double yaw_error = std::remainder(want_yaw_rad - current_yaw, 2.0 * kPi);
    const double pitch_error = std::remainder(want_pitch_rad - current_pitch, 2.0 * kPi);

    geometry_msgs::msg::PointStamped debug_point;
    debug_point.header.stamp = now();
    debug_point.header.frame_id = barrel_joint_frame_;
    debug_point.point.x = selected.bx;
    debug_point.point.y = selected.by;
    debug_point.point.z = selected.bz;
    debug_target_point_pub_->publish(debug_point);

    std_msgs::msg::Int32 debug_index;
    debug_index.data = selected.armor_index;
    debug_selected_armor_index_pub_->publish(debug_index);
    publishSelectedArmorDebug(active_target, best_target, selected.armor_index);

    const double yaw_error_deg = std::abs(yaw_error * 180.0 / kPi);
    const double pitch_error_deg = std::abs(pitch_error * 180.0 / kPi);
    bool inside_face_window = true;
    if (active_target.is_outpost && selected.armor_index >= 0) {
      const auto impact_armors = aim_armor_controller::buildLegacyArmors(best_target);
      const double selected_armor_yaw =
        static_cast<std::size_t>(selected.armor_index) < impact_armors.size() ?
        impact_armors[static_cast<std::size_t>(selected.armor_index)].yaw :
        aim_armor_controller::limitRad(
        best_target.yaw + static_cast<double>(selected.armor_index) * 2.0 * kPi / 3.0);
      inside_face_window = aim_armor_controller::allowOutpostFireByFacingGate(
        best_target.center.x, best_target.center.y, selected_armor_yaw,
        outpost_shoot_yaw_gate_rad_);
    } else if (spin_center_aim && selected.armor_index >= 0) {
      const auto armors = aim_armor_controller::buildLegacyArmors(best_target);
      if (static_cast<std::size_t>(selected.armor_index) < armors.size()) {
        const double facing_error = aim_armor_controller::legacyArmorFacingError(
          best_target, armors[static_cast<std::size_t>(selected.armor_index)]);
        inside_face_window = std::abs(facing_error) < shoot_face_tolerance_rad_;
      } else {
        inside_face_window = false;
      }
    }

    const aim_armor_controller::LegacyFireControlInput fire_control_input{
      static_cast<int>(active_target.model.id),
      yaw_error_deg,
      pitch_error_deg,
      shoot_yaw_tolerance_deg_,
      shoot_pitch_tolerance_deg_,
      inside_face_window,
      spin_center_aim,
      selected.armor_index >= 0 ?
        static_cast<std::size_t>(selected.armor_index) : 0U,
    };
    const auto fire_control_output = aim_armor_controller::evaluateLegacyFireControl(
      fire_control_input, legacy_fire_control_state_);

    if (publish_legacy_control_topics_ && angle_pub_) {
      gimbal_driver::msg::GimbalAngles angle_msg;
      angle_msg.yaw = static_cast<float>(selected.yaw_actual_want_deg);
      angle_msg.pitch = static_cast<float>(selected.pitch_actual_want_deg);
      angle_pub_->publish(angle_msg);
    }

    bool fire_this_tick = false;
    if (!manual_fire_mode_ && fire_control_output.shoot_flag && fire_rate_hz_ > 0.0) {
      const double elapsed = (now() - last_fire_time_).seconds();
      if (elapsed >= 1.0 / fire_rate_hz_) {
        last_firecode_out_ = (last_firecode_out_ == 99) ? 96 : 99;
        last_fire_time_ = now();
        fire_this_tick = true;
      }
    }

    aim_target_msgs::msg::AimResult aim_msg;
    aim_msg.header.stamp = now();
    aim_msg.header.frame_id = barrel_joint_frame_;
    aim_msg.fire = fire_this_tick;
    aim_msg.pitch = static_cast<float>(selected.pitch_actual_want_deg);
    aim_msg.yaw = static_cast<float>(selected.yaw_actual_want_deg);
    aim_result_pub_->publish(aim_msg);

    if (publish_legacy_control_topics_ && !manual_fire_mode_ && fire_pub_) {
      fire_pub_->publish(makeFireCodeMsg(now(), last_firecode_out_));
    }
  }

  std::mutex data_mutex_;
  TargetStateArrayCache front_0_cache_;
  TargetStateArrayCache front_1_cache_;
  TargetStateArrayCache back_cache_;
  OutpostStateCache outpost_cache_;
  SelectedTargetIdCache selected_target_cache_;
  GimbalState gimbal_state_;

  bool shoot_table_adjust_{false};
  std::vector<double> pitch_param_;
  std::vector<double> yaw_param_;
  double x_offset_{0.0};
  double y_offset_{0.0};
  double z_offset_{0.0};
  std::string barrel_joint_frame_{"gimbal_barrel_joint"};
  double target_tf_timeout_sec_{0.02};
  double target_msg_timeout_sec_{0.10};
  double armor_select_area_weight_{1.0};
  double armor_select_angle_weight_{1.0};
  bool include_processing_delay_{true};
  double system_response_time_sec_{0.0};
  int max_prediction_iterations_{10};
  double fly_time_converge_threshold_sec_{0.001};
  double max_processing_delay_sec_{0.2};
  double max_predict_time_sec_{1.0};
  double shoot_yaw_tolerance_deg_{1.0};
  double shoot_pitch_tolerance_deg_{1.0};
  double shoot_face_tolerance_rad_{2.0 * kPi / 180.0};
  double spin_center_aim_angular_velocity_threshold_{2.0};
  bool publish_legacy_control_topics_{true};
  bool manual_fire_mode_{false};
  bool enable_smart_selector_{true};
  double comming_angle_rad_{60.0 * kPi / 180.0};
  double leaving_angle_rad_{20.0 * kPi / 180.0};
  double smart_selector_max_angular_velocity_{2.0};
  double lock_index_{-1.0};
  double outpost_shoot_yaw_gate_rad_{45.0 * kPi / 180.0};
  double bullet_speed_{23.0};
  double min_bullet_speed_{20.0};
  double bullet_speed_alpha_{0.5};
  double tol_deltax_{0.2};
  double tol_deltay_{0.1};
  double bullet_mass_{3.2e-3};
  double bullet_diameter_{16.8e-3};
  int max_iter_{100};
  double tol_{1e-6};
  std::uint8_t last_firecode_out_{0};
  rclcpp::Time last_fire_time_{0, 0, RCL_ROS_TIME};
  double fire_rate_hz_{10.0};
  aim_armor_controller::LegacyFireControlState legacy_fire_control_state_{};

  rclcpp::Publisher<aim_target_msgs::msg::AimResult>::SharedPtr aim_result_pub_;
  rclcpp::Publisher<aim_msgs::msg::Armor>::SharedPtr selected_armor_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr debug_target_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr debug_selected_armor_index_pub_;
  rclcpp::Publisher<gimbal_driver::msg::GimbalAngles>::SharedPtr angle_pub_;
  rclcpp::Publisher<gimbal_driver::msg::FireCode>::SharedPtr fire_pub_;
  rclcpp::Subscription<aim_msgs::msg::TargetStateArray>::SharedPtr front_0_sub_;
  rclcpp::Subscription<aim_msgs::msg::TargetStateArray>::SharedPtr front_1_sub_;
  rclcpp::Subscription<aim_msgs::msg::TargetStateArray>::SharedPtr back_sub_;
  rclcpp::Subscription<aim_msgs::msg::OutpostState>::SharedPtr outpost_sub_;
  rclcpp::Subscription<aim_msgs::msg::SelectedTargetId>::SharedPtr selected_target_sub_;
  rclcpp::Subscription<gimbal_driver::msg::GimbalAngles>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<gimbal_driver::msg::BulletInfo>::SharedPtr bullet_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmorControllerNode>());
  rclcpp::shutdown();
  return 0;
}
