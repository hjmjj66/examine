#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
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
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "aim_armor_controller/control_time_alignment.hpp"
#include "aim_armor_controller/command_rate_limiter.hpp"
#include "aim_armor_controller/fire_code_state.hpp"
#include "aim_armor_controller/legacy_fire_control.hpp"
#include "aim_armor_controller/outpost_tracking_hold.hpp"
#include "aim_armor_controller/outpost_fire_gate.hpp"
#include "aim_armor_controller/legacy_target_model.hpp"
#include "aim_armor_controller/legacy_timing.hpp"
#include "aim_armor_controller/mpc_math.hpp"
#include "aim_armor_controller/mpc_aim_geometry.hpp"
#include "aim_armor_controller/mpc_planner.hpp"
#include "aim_armor_controller/mpc_target_model.hpp"
#include "aim_armor_controller/mpc_trajectory_solver.hpp"
#include "aim_armor_controller/aim_result_kinematics.hpp"
#include "aim_armor_controller/controller_trajectory_message.hpp"
#include "aim_armor_controller/target_state_selection.hpp"
#include "aim_msgs/msg/armor.hpp"
#include "gimbal_driver/msg/gimbal_state.hpp"
#include "aim_msgs/msg/outpost_state.hpp"
#include "aim_msgs/msg/selected_target_id.hpp"
#include "aim_msgs/msg/target_state_array.hpp"
#include "gimbal_driver/msg/bullet_info.hpp"
#include "gimbal_driver/msg/fire_code.hpp"
#include "gimbal_driver/msg/gimbal_angles.hpp"
#include "sentry_msgs/msg/aim_result.hpp"

namespace
{

constexpr double kPi = aim_armor_controller::kPi;
constexpr double kGravity = 9.794;
constexpr double kCd = 0.42;
constexpr double kRho = 1.169;
constexpr std::uint8_t kDefaultOutpostId = 6;

rclcpp::QoS makeRealtimeSensorQos()
{
  return rclcpp::SensorDataQoS().keep_last(1);
}

struct GimbalState
{
  float yaw_deg{0.0F};
  float pitch_deg{0.0F};
  double yaw_rad{0.0};
  double pitch_rad{0.0};
  double yaw_velocity_rad_s{0.0};
  double pitch_velocity_rad_s{0.0};
  double yaw_acceleration_rad_s2{0.0};
  double pitch_acceleration_rad_s2{0.0};
  builtin_interfaces::msg::Time stamp;
  std::chrono::steady_clock::time_point received_at{};
  bool valid{false};
};

using GimbalStateSample = aim_armor_controller::GimbalStateSample;

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
  bool tracking_hold{false};
  bool valid{false};
};

gimbal_driver::msg::FireCode makeFireCodeMsg(
  const rclcpp::Time & stamp,
  const aim_armor_controller::FireCodeState & state)
{
  gimbal_driver::msg::FireCode msg;
  msg.header.stamp = stamp;
  msg.field_mask = gimbal_driver::msg::FireCode::FIELD_ALL;
  msg.fire_status = static_cast<std::uint8_t>(state.fire_status & 0x03U);
  msg.cap_state = static_cast<std::uint8_t>(state.cap_state & 0x03U);
  msg.follow_mode = state.follow_mode;
  msg.aim_mode = state.aim_mode;
  msg.rotate = static_cast<std::uint8_t>(state.rotate & 0x03U);
  msg.raw = aim_armor_controller::encodeFireCodeRaw(state);
  return msg;
}

struct ArmorCandidate
{
  double bx{0.0};
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
    declare_parameter<std::string>("gimbal_state_topic", "/ly/gimbal/state");
    declare_parameter<std::string>("gimbal_posture_topic", "/ly/gimbal/posture");
    declare_parameter<std::string>("bullet_speed_topic", "/ly/game/bullet");
    declare_parameter<std::string>("control_angles_topic", "/ly/control/angles");
    declare_parameter<std::string>("control_trajectory_topic", "/ly/control/trajectory");
    declare_parameter<std::string>("control_firecode_topic", "/ly/control/firecode");
    declare_parameter<std::string>("selected_armor_topic", "/controller/selected_armor");
    declare_parameter<std::string>(
      "debug_target_point_topic", "/armor_controller/debug/target_point_barrel");
    declare_parameter<std::string>(
      "debug_selected_armor_index_topic", "/armor_controller/debug/selected_armor_index");
    declare_parameter<bool>("debug_aim_geometry_log", false);
    declare_parameter<bool>("publish_legacy_control_topics", true);
    declare_parameter<bool>("publish_control_angles", false);
    declare_parameter<bool>("publish_control_trajectory", false);
    declare_parameter<bool>("manual_fire_mode", false);
    declare_parameter<double>("publish_rate_hz", 100.0);
    declare_parameter<double>("fire_rate_hz", 10.0);
    declare_parameter<double>("overclock_fire_rate_hz", 20.0);
    declare_parameter<double>("bullet_speed_mps", 23.0);
    declare_parameter<double>("min_bullet_speed_mps", 20.0);
    declare_parameter<double>("fallback_bullet_speed_mps", 23.0);
    declare_parameter<bool>("use_air_resistance", true);
    declare_parameter<double>("bullet_speed_alpha", 0.5);
    declare_parameter<double>("tol_deltax_m", 0.2);
    declare_parameter<double>("tol_deltay_m", 0.1);
    declare_parameter<double>("bullet_mass_kg", 3.2e-3);
    declare_parameter<double>("bullet_diameter_m", 16.8e-3);
    declare_parameter<double>("muzzle_offset_x_m", 0.0);
    declare_parameter<int>("max_iter", 100);
    declare_parameter<double>("tol", 1e-6);
    declare_parameter<std::string>("barrel_joint_frame", "gimbal_barrel_joint");
    declare_parameter<double>("target_tf_timeout_sec", 0.02);
    declare_parameter<double>("target_msg_timeout_sec", 0.10);
    declare_parameter<double>("gimbal_state_timeout_sec", 0.10);
    declare_parameter<double>("outpost_tracking_hold_sec", 0.15);
    declare_parameter<double>("max_angle_rate_deg_per_sec", 60.0);
    declare_parameter<double>("armor_select_area_weight", 1.0);
    declare_parameter<double>("armor_select_angle_weight", 1.0);
    declare_parameter<bool>("include_processing_delay", true);
    declare_parameter<double>("system_response_time_sec", 0.0);
    declare_parameter<double>("outpost_extra_prediction_time_sec", 0.0);
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
    declare_parameter<double>("low_speed_angular_velocity_threshold", 2.0);
    declare_parameter<double>("selector_response_speed_rad_s", 0.01);
    declare_parameter<double>("selector_min_angular_velocity_rad_s", 0.6);
    declare_parameter<bool>("allow_predicted_outpost_slots", true);
    declare_parameter<double>("outpost_shoot_yaw_gate_deg", 60.0);
    declare_parameter<bool>("enable_mpc", true);
    declare_parameter<std::string>("world_frame_id", "gimbal_world");
    declare_parameter<bool>("use_current_time_for_tf", false);
    declare_parameter<double>("yaw_offset_deg", 0.0);
    declare_parameter<double>("pitch_offset_deg", 0.0);
    declare_parameter<double>("gimbal_state_history_sec", 2.0);
    declare_parameter<double>("mpc_dt", 0.01);
    declare_parameter<int>("mpc_horizon", 100);
    declare_parameter<double>("fire_thresh", 0.0035);
    declare_parameter<int>("fire_offset", 2);
    declare_parameter<double>("max_yaw_acc", 50.0);
    declare_parameter<double>("max_pitch_acc", 100.0);
    declare_parameter<double>("q_yaw_position", 9.0e6);
    declare_parameter<double>("q_yaw_velocity", 0.0);
    declare_parameter<double>("r_yaw_acceleration", 1.0);
    declare_parameter<double>("q_pitch_position", 9.0e6);
    declare_parameter<double>("q_pitch_velocity", 0.0);
    declare_parameter<double>("r_pitch_acceleration", 1.0);
    declare_parameter<int>("mpc_max_iterations", 10);

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
    const auto gimbal_state_topic = get_parameter("gimbal_state_topic").as_string();
    const auto gimbal_posture_topic = get_parameter("gimbal_posture_topic").as_string();
    const auto bullet_topic = get_parameter("bullet_speed_topic").as_string();
    const auto control_angles_topic = get_parameter("control_angles_topic").as_string();
    const auto control_trajectory_topic =
      get_parameter("control_trajectory_topic").as_string();
    const auto control_firecode_topic = get_parameter("control_firecode_topic").as_string();
    const auto selected_armor_topic = get_parameter("selected_armor_topic").as_string();
    const auto debug_target_point_topic = get_parameter("debug_target_point_topic").as_string();
    const auto debug_selected_armor_index_topic =
      get_parameter("debug_selected_armor_index_topic").as_string();
    debug_aim_geometry_log_ = get_parameter("debug_aim_geometry_log").as_bool();

    publish_legacy_control_topics_ = get_parameter("publish_legacy_control_topics").as_bool();
    publish_control_angles_ = get_parameter("publish_control_angles").as_bool();
    publish_control_trajectory_ = get_parameter("publish_control_trajectory").as_bool();
    manual_fire_mode_ = get_parameter("manual_fire_mode").as_bool();
    const double publish_rate = get_parameter("publish_rate_hz").as_double();
    fire_rate_hz_ = get_parameter("fire_rate_hz").as_double();
    overclock_fire_rate_hz_ = get_parameter("overclock_fire_rate_hz").as_double();
    bullet_speed_ = get_parameter("bullet_speed_mps").as_double();
    min_bullet_speed_ = get_parameter("min_bullet_speed_mps").as_double();
    fallback_bullet_speed_ = get_parameter("fallback_bullet_speed_mps").as_double();
    use_air_resistance_ = get_parameter("use_air_resistance").as_bool();
    bullet_speed_alpha_ = get_parameter("bullet_speed_alpha").as_double();
    tol_deltax_ = get_parameter("tol_deltax_m").as_double();
    tol_deltay_ = get_parameter("tol_deltay_m").as_double();
    bullet_mass_ = get_parameter("bullet_mass_kg").as_double();
    bullet_diameter_ = get_parameter("bullet_diameter_m").as_double();
    muzzle_offset_x_ = get_parameter("muzzle_offset_x_m").as_double();
    max_iter_ = get_parameter("max_iter").as_int();
    tol_ = get_parameter("tol").as_double();
    barrel_joint_frame_ = get_parameter("barrel_joint_frame").as_string();
    target_tf_timeout_sec_ = get_parameter("target_tf_timeout_sec").as_double();
    target_msg_timeout_sec_ = get_parameter("target_msg_timeout_sec").as_double();
    gimbal_state_timeout_sec_ = get_parameter("gimbal_state_timeout_sec").as_double();
    outpost_tracking_hold_sec_ = get_parameter("outpost_tracking_hold_sec").as_double();
    max_angle_rate_deg_per_sec_ =
      get_parameter("max_angle_rate_deg_per_sec").as_double();
    command_rate_limiter_.setMaxRateDegPerSec(max_angle_rate_deg_per_sec_);
    armor_select_area_weight_ = get_parameter("armor_select_area_weight").as_double();
    armor_select_angle_weight_ = get_parameter("armor_select_angle_weight").as_double();
    include_processing_delay_ = get_parameter("include_processing_delay").as_bool();
    system_response_time_sec_ = get_parameter("system_response_time_sec").as_double();
    outpost_extra_prediction_time_sec_ =
      get_parameter("outpost_extra_prediction_time_sec").as_double();
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
    low_speed_angular_velocity_threshold_ =
      get_parameter("low_speed_angular_velocity_threshold").as_double();
    selector_response_speed_rad_s_ =
      get_parameter("selector_response_speed_rad_s").as_double();
    selector_min_angular_velocity_rad_s_ =
      get_parameter("selector_min_angular_velocity_rad_s").as_double();
    allow_predicted_outpost_slots_ =
      get_parameter("allow_predicted_outpost_slots").as_bool();
    if (std::abs(
        smart_selector_max_angular_velocity_ -
        low_speed_angular_velocity_threshold_) > 1e-9)
    {
      RCLCPP_WARN(
        get_logger(),
        "low_speed_angular_velocity_threshold=%.3f differs from canonical "
        "smart_selector_max_angular_velocity=%.3f; using canonical value",
        low_speed_angular_velocity_threshold_, smart_selector_max_angular_velocity_);
    }
    low_speed_angular_velocity_threshold_ = smart_selector_max_angular_velocity_;
    outpost_shoot_yaw_gate_rad_ =
      get_parameter("outpost_shoot_yaw_gate_deg").as_double() * kPi / 180.0;
    enable_mpc_ = get_parameter("enable_mpc").as_bool();
    world_frame_id_ = get_parameter("world_frame_id").as_string();
    use_current_time_for_tf_ = get_parameter("use_current_time_for_tf").as_bool();
    mpc_aim_config_.use_air_resistance = use_air_resistance_;
    mpc_aim_config_.yaw_offset_rad =
      get_parameter("yaw_offset_deg").as_double() * kPi / 180.0;
    mpc_aim_config_.pitch_offset_rad =
      get_parameter("pitch_offset_deg").as_double() * kPi / 180.0;
    mpc_aim_config_.muzzle_offset_x_m = muzzle_offset_x_;
    mpc_aim_config_.selection.enable_smart_selector = enable_smart_selector_;
    mpc_aim_config_.selection.low_speed_angular_velocity_threshold =
      smart_selector_max_angular_velocity_;
    mpc_aim_config_.selection.coming_angle_rad = comming_angle_rad_;
    mpc_aim_config_.selection.leaving_angle_rad = leaving_angle_rad_;
    mpc_aim_config_.selection.selector_response_speed_rad_s =
      selector_response_speed_rad_s_;
    mpc_aim_config_.selection.selector_min_angular_velocity_rad_s =
      selector_min_angular_velocity_rad_s_;
    mpc_aim_config_.selection.allow_predicted_outpost_slots =
      allow_predicted_outpost_slots_;
    gimbal_state_history_sec_ =
      std::max(0.0, get_parameter("gimbal_state_history_sec").as_double());

    mpc_config_.yaw.dt = get_parameter("mpc_dt").as_double();
    mpc_config_.pitch.dt = mpc_config_.yaw.dt;
    mpc_config_.yaw.horizon = get_parameter("mpc_horizon").as_int();
    mpc_config_.pitch.horizon = mpc_config_.yaw.horizon;
    mpc_config_.fire_threshold = get_parameter("fire_thresh").as_double();
    mpc_config_.fire_offset = get_parameter("fire_offset").as_int();
    mpc_config_.yaw.max_acceleration = get_parameter("max_yaw_acc").as_double();
    mpc_config_.pitch.max_acceleration = get_parameter("max_pitch_acc").as_double();
    mpc_config_.yaw.q_position = get_parameter("q_yaw_position").as_double();
    mpc_config_.yaw.q_velocity = get_parameter("q_yaw_velocity").as_double();
    mpc_config_.yaw.r_acceleration = get_parameter("r_yaw_acceleration").as_double();
    mpc_config_.pitch.q_position = get_parameter("q_pitch_position").as_double();
    mpc_config_.pitch.q_velocity = get_parameter("q_pitch_velocity").as_double();
    mpc_config_.pitch.r_acceleration = get_parameter("r_pitch_acceleration").as_double();
    const int mpc_max_iterations = get_parameter("mpc_max_iterations").as_int();
    mpc_config_.yaw.max_iterations = mpc_max_iterations;
    mpc_config_.pitch.max_iterations = mpc_max_iterations;
    if (enable_mpc_) {
      if (mpc_config_.yaw.horizon < 2 || mpc_config_.yaw.horizon % 2 != 0) {
        throw std::runtime_error("mpc_horizon must be an even integer greater than or equal to 2");
      }
      mpc_planner_ = std::make_unique<aim_armor_controller::MpcPlanner>(mpc_config_);
    }

    loadShootTableParams();
    x_offset_ = get_parameter(toff + "x").as_double();
    y_offset_ = get_parameter(toff + "y").as_double();
    z_offset_ = get_parameter(toff + "z").as_double();
    mpc_aim_config_.target_offset.x = x_offset_;
    mpc_aim_config_.target_offset.y = y_offset_;
    mpc_aim_config_.target_offset.z = z_offset_;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    aim_result_pub_ = create_publisher<sentry_msgs::msg::AimResult>(aim_result_topic, 10);
    selected_armor_pub_ =
      create_publisher<aim_msgs::msg::Armor>(selected_armor_topic, rclcpp::SensorDataQoS());
    debug_target_point_pub_ =
      create_publisher<geometry_msgs::msg::PointStamped>(debug_target_point_topic, 10);
    debug_selected_armor_index_pub_ =
      create_publisher<std_msgs::msg::Int32>(debug_selected_armor_index_topic, 10);
    if (publish_control_angles_) {
      angle_pub_ = create_publisher<gimbal_driver::msg::GimbalAngles>(control_angles_topic, 10);
    }
    if (publish_legacy_control_topics_ && !manual_fire_mode_) {
      fire_pub_ = create_publisher<gimbal_driver::msg::FireCode>(control_firecode_topic, 10);
    }
    if (publish_control_trajectory_) {
      trajectory_pub_ = create_publisher<aim_armor_controller::ControllerTrajectoryMessage>(
        control_trajectory_topic, makeRealtimeSensorQos());
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

    const auto realtime_sensor_qos = makeRealtimeSensorQos();

    gimbal_state_sub_ = create_subscription<gimbal_driver::msg::GimbalState>(
      gimbal_state_topic, realtime_sensor_qos,
      [this](const gimbal_driver::msg::GimbalState::SharedPtr msg) {
        if (msg == nullptr) {
          return;
        }
        updateGimbalState(
          msg->yaw, msg->pitch, msg->yaw_omega, msg->pitch_omega,
          msg->yaw_alpha, msg->pitch_alpha, msg->header.stamp, true);
      });

    posture_sub_ = create_subscription<std_msgs::msg::UInt8>(
      gimbal_posture_topic, realtime_sensor_qos,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        if (msg != nullptr) {
          overclock_mode_ = msg->data >= 4U && msg->data <= 6U;
        }
      });

    bullet_sub_ = create_subscription<gimbal_driver::msg::BulletInfo>(
      bullet_topic, realtime_sensor_qos,
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
      "selected_id=%s aim_result=%s gimbal_state=%s posture=%s bullet=%s frame=%s",
      front_0_topic.c_str(), front_1_topic.c_str(), back_topic.c_str(), outpost_topic.c_str(),
      selected_target_topic.c_str(), aim_result_topic.c_str(), gimbal_state_topic.c_str(),
      gimbal_posture_topic.c_str(),
      bullet_topic.c_str(), barrel_joint_frame_.c_str());
  }

private:
  double activeFireRateHz() const
  {
    return overclock_mode_ ? overclock_fire_rate_hz_ : fire_rate_hz_;
  }

  void updateGimbalState(
    float yaw_deg,
    float pitch_deg,
    float yaw_omega_deg_s,
    float pitch_omega_deg_s,
    float yaw_alpha_deg_s2,
    float pitch_alpha_deg_s2,
    const builtin_interfaces::msg::Time & stamp,
    bool has_dynamics)
  {
    if (!std::isfinite(yaw_deg) || !std::isfinite(pitch_deg) ||
      (has_dynamics &&
      (!std::isfinite(yaw_omega_deg_s) || !std::isfinite(pitch_omega_deg_s) ||
      !std::isfinite(yaw_alpha_deg_s2) || !std::isfinite(pitch_alpha_deg_s2))))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ignoring non-finite gimbal feedback: yaw=%.6f pitch=%.6f "
        "yaw_omega=%.6f pitch_omega=%.6f yaw_alpha=%.6f pitch_alpha=%.6f "
        "has_dynamics=%s",
        static_cast<double>(yaw_deg), static_cast<double>(pitch_deg),
        static_cast<double>(yaw_omega_deg_s), static_cast<double>(pitch_omega_deg_s),
        static_cast<double>(yaw_alpha_deg_s2), static_cast<double>(pitch_alpha_deg_s2),
        has_dynamics ? "true" : "false");
      return;
    }

    const auto received_at = std::chrono::steady_clock::now();
    const rclcpp::Time received_ros_time = now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    gimbal_state_.yaw_deg = yaw_deg;
    gimbal_state_.pitch_deg = pitch_deg;
    gimbal_state_.yaw_rad = static_cast<double>(yaw_deg) * kPi / 180.0;
    gimbal_state_.pitch_rad = static_cast<double>(pitch_deg) * kPi / 180.0;
    if (has_dynamics) {
      gimbal_state_.yaw_velocity_rad_s =
        static_cast<double>(yaw_omega_deg_s) * kPi / 180.0;
      gimbal_state_.pitch_velocity_rad_s =
        static_cast<double>(pitch_omega_deg_s) * kPi / 180.0;
      gimbal_state_.yaw_acceleration_rad_s2 =
        static_cast<double>(yaw_alpha_deg_s2) * kPi / 180.0;
      gimbal_state_.pitch_acceleration_rad_s2 =
        static_cast<double>(pitch_alpha_deg_s2) * kPi / 180.0;
    }
    gimbal_state_.stamp = stamp;
    gimbal_state_.received_at = received_at;
    gimbal_state_.valid = true;

    GimbalStateSample sample;
    sample.stamp = aim_armor_controller::resolveControlStamp(stamp, received_ros_time);
    sample.yaw_rad = gimbal_state_.yaw_rad;
    sample.pitch_rad = gimbal_state_.pitch_rad;
    sample.yaw_velocity_rad_s = gimbal_state_.yaw_velocity_rad_s;
    sample.pitch_velocity_rad_s = gimbal_state_.pitch_velocity_rad_s;
    if (!gimbal_state_history_.empty()) {
      sample.yaw_rad = aim_armor_controller::unwrapAngleNear(
        sample.yaw_rad, gimbal_state_history_.back().yaw_rad);
    }
    gimbal_state_.yaw_rad = sample.yaw_rad;
    if (gimbal_state_history_.empty() || sample.stamp >= gimbal_state_history_.back().stamp) {
      gimbal_state_history_.push_back(sample);
      const rclcpp::Time oldest_allowed =
        sample.stamp - rclcpp::Duration::from_seconds(gimbal_state_history_sec_);
      while (!gimbal_state_history_.empty() &&
        gimbal_state_history_.front().stamp < oldest_allowed)
      {
        gimbal_state_history_.pop_front();
      }
    }
  }

  void publishControlTrajectory(
    const rclcpp::Time & stamp,
    double yaw_deg,
    double pitch_deg,
    double yaw_omega_deg_s,
    double pitch_omega_deg_s,
    double yaw_alpha_deg_s2,
    double pitch_alpha_deg_s2)
  {
    if (!trajectory_pub_) {
      return;
    }
    if (
      !std::isfinite(yaw_deg) || !std::isfinite(pitch_deg) ||
      !std::isfinite(yaw_omega_deg_s) || !std::isfinite(pitch_omega_deg_s) ||
      !std::isfinite(yaw_alpha_deg_s2) || !std::isfinite(pitch_alpha_deg_s2))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "not publishing non-finite control trajectory: yaw=%.6f pitch=%.6f "
        "yaw_omega=%.6f pitch_omega=%.6f yaw_alpha=%.6f pitch_alpha=%.6f",
        yaw_deg, pitch_deg, yaw_omega_deg_s, pitch_omega_deg_s,
        yaw_alpha_deg_s2, pitch_alpha_deg_s2);
      return;
    }
    aim_armor_controller::ControllerTrajectoryMessage msg;
    msg.header.stamp = stamp;
    msg.yaw = static_cast<float>(yaw_deg);
    msg.pitch = static_cast<float>(pitch_deg);
    msg.yaw_omega = static_cast<float>(yaw_omega_deg_s);
    msg.pitch_omega = static_cast<float>(pitch_omega_deg_s);
    msg.yaw_alpha = static_cast<float>(yaw_alpha_deg_s2);
    msg.pitch_alpha = static_cast<float>(pitch_alpha_deg_s2);
    trajectory_pub_->publish(msg);
  }

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
      !std::isfinite(bullet_diameter_) || !std::isfinite(muzzle_offset_x_) ||
      bullet_speed_ <= 0.0 ||
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

    if (std::abs(muzzle_offset_x_) > 1e-9) {
      const auto residual_at = [&](double pitch_rad, double & residual, double & flight_time) {
          const double cth = std::cos(pitch_rad);
          if (!std::isfinite(cth) || std::abs(cth) < 1e-8) {
            return false;
          }
          const double sth = std::sin(pitch_rad);
          const double flight_distance = distance - muzzle_offset_x_ * cth;
          if (!std::isfinite(flight_distance) || flight_distance <= 1e-6) {
            return false;
          }
          const double target_z_from_muzzle = target_z - muzzle_offset_x_ * sth;
          flight_time = (std::exp(k1 * flight_distance) - 1.0) /
            (k1 * bullet_speed_ * cth);
          if (!std::isfinite(flight_time)) {
            return false;
          }
          residual = target_z_from_muzzle - bullet_speed_ * sth * flight_time / cth +
            0.5 * kGravity * flight_time * flight_time / cth / cth;
          return std::isfinite(residual);
        };

      bool solved = false;
      for (int i = 0; i < max_iter_; ++i) {
        if (!residual_at(theta, delta_z, time)) {
          return false;
        }
        if (std::abs(delta_z) < tol_) {
          solved = true;
          break;
        }
        constexpr double kDerivativeStep = 1e-5;
        double residual_plus = 0.0;
        double residual_minus = 0.0;
        double unused_time = 0.0;
        if (
          !residual_at(theta + kDerivativeStep, residual_plus, unused_time) ||
          !residual_at(theta - kDerivativeStep, residual_minus, unused_time))
        {
          return false;
        }
        const double derivative = (residual_plus - residual_minus) / (2.0 * kDerivativeStep);
        if (!std::isfinite(derivative) || std::abs(derivative) < 1e-12) {
          return false;
        }
        theta = std::clamp(theta - delta_z / derivative, -1.4, 1.4);
      }

      if (!solved || !std::isfinite(theta) || !std::isfinite(delta_z)) {
        return false;
      }
      pitch = theta;
      yaw = std::atan2(target_y, target_x);
      return true;
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

  void publishFallback(const GimbalState & gimbal, const std::string & reason)
  {
    const rclcpp::Time stamp =
      aim_armor_controller::resolveControlStamp(gimbal.stamp, now());
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "fallback: reason=%s gimbal=(%.3f, %.3f) control_stamp=%.9f",
      reason.c_str(), static_cast<double>(gimbal.yaw_deg),
      static_cast<double>(gimbal.pitch_deg), stamp.seconds());
    const auto command = limitCommandAngles(
      static_cast<double>(gimbal.yaw_deg), static_cast<double>(gimbal.pitch_deg),
      gimbal, stamp);
    if (!std::isfinite(command.yaw_deg) || !std::isfinite(command.pitch_deg)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "not publishing fallback with non-finite command: reason=%s yaw=%.6f pitch=%.6f",
        reason.c_str(), command.yaw_deg, command.pitch_deg);
      return;
    }

    sentry_msgs::msg::AimResult aim_msg;
    aim_msg.header.stamp = stamp;
    aim_msg.header.frame_id = barrel_joint_frame_;
    aim_msg.follow = false;
    aim_msg.fire = false;
    aim_msg.yaw = static_cast<float>(command.yaw_deg);
    aim_msg.pitch = static_cast<float>(command.pitch_deg);
    aim_armor_controller::setAimResultKinematics(
      aim_msg, command.yaw_deg, command.pitch_deg, 0.0, 0.0, 0.0, 0.0);
    aim_result_pub_->publish(aim_msg);

    if (publish_control_angles_ && angle_pub_) {
      gimbal_driver::msg::GimbalAngles angle_msg;
      angle_msg.header.stamp = stamp;
      angle_msg.yaw = static_cast<float>(command.yaw_deg);
      angle_msg.pitch = static_cast<float>(command.pitch_deg);
      angle_pub_->publish(angle_msg);
    }
    publishControlTrajectory(
      stamp, command.yaw_deg, command.pitch_deg, 0.0, 0.0, 0.0, 0.0);

    if (publish_legacy_control_topics_ && !manual_fire_mode_ && fire_pub_) {
      fire_pub_->publish(makeFireCodeMsg(now(), fire_code_state_));
    }
  }

  aim_armor_controller::CommandAngles limitCommandAngles(
    double desired_yaw_deg, double desired_pitch_deg,
    const GimbalState & gimbal, const rclcpp::Time & stamp)
  {
    if (
      !std::isfinite(desired_yaw_deg) || !std::isfinite(desired_pitch_deg) ||
      !std::isfinite(gimbal.yaw_deg) || !std::isfinite(gimbal.pitch_deg))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "not updating command rate limiter with non-finite angles: desired=(%.6f, %.6f) "
        "gimbal=(%.6f, %.6f)",
        desired_yaw_deg, desired_pitch_deg,
        static_cast<double>(gimbal.yaw_deg), static_cast<double>(gimbal.pitch_deg));
      return aim_armor_controller::CommandAngles{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    }

    if (!command_rate_limiter_initialized_) {
      command_rate_limiter_.reset(
        static_cast<double>(gimbal.yaw_deg), static_cast<double>(gimbal.pitch_deg));
      last_command_limit_stamp_ = stamp;
      command_rate_limiter_initialized_ = true;
    }

    const double dt_sec = std::max(0.0, (stamp - last_command_limit_stamp_).seconds());
    last_command_limit_stamp_ = stamp;
    return command_rate_limiter_.update(desired_yaw_deg, desired_pitch_deg, dt_sec);
  }

  bool transformWorldToBarrel(
    double wx, double wy, double wz, const std::string & source_frame,
    double & x, double & y, double & z, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(
        barrel_joint_frame_, source_frame, stamp,
        rclcpp::Duration::from_seconds(target_tf_timeout_sec_));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "control-time TF lookup failed (%s -> %s at %.9f): %s",
        source_frame.c_str(), barrel_joint_frame_.c_str(), stamp.seconds(), e.what());
      return false;
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

  bool isTimedOut(
    const builtin_interfaces::msg::Time & stamp,
    const rclcpp::Time & control_stamp) const
  {
    if (target_msg_timeout_sec_ <= 0.0 || aim_armor_controller::isZeroStamp(stamp)) {
      return false;
    }
    return aim_armor_controller::ageAtControlStamp(stamp, control_stamp) >
           target_msg_timeout_sec_;
  }

  std::string activeTargetFailureReason(
    const SelectedTargetIdCache & selected_target,
    const TargetStateArrayCache & front_0,
    const TargetStateArrayCache & front_1,
    const TargetStateArrayCache & back,
    const OutpostStateCache & outpost,
    const rclcpp::Time & control_stamp) const
  {
    if (!selected_target.valid) {
      return "selected_target_cache_missing";
    }
    if (!selected_target.msg.valid) {
      return "selected_target_message_invalid";
    }

    const std::uint8_t outpost_id = outpost.valid ? outpost.msg.id : kDefaultOutpostId;
    if (selected_target.msg.id == outpost_id) {
      if (!outpost.valid) {
        return "outpost_state_missing";
      }
      if (!outpost.msg.tracking) {
        return "outpost_not_tracking";
      }
      if (!outpost.msg.converged) {
        return "outpost_not_converged";
      }
      if (isTimedOut(outpost.msg.header.stamp, control_stamp)) {
        return "outpost_message_timeout";
      }
      return "outpost_active_target_resolution_failed";
    }

    const auto match = aim_armor_controller::selectBestTargetStateMatch(
      selected_target.msg.id,
      front_0.valid ? &front_0.msg : nullptr,
      front_1.valid ? &front_1.msg : nullptr,
      back.valid ? &back.msg : nullptr);
    if (!match.has_value()) {
      return "selected_target_not_found_in_target_states";
    }
    if (isTimedOut(match->header.stamp, control_stamp)) {
      return "selected_target_message_timeout";
    }
    return "active_target_resolution_failed";
  }

  double measurementAgeFromStamp(
    const builtin_interfaces::msg::Time & stamp,
    const rclcpp::Time & control_stamp) const
  {
    return aim_armor_controller::ageAtControlStamp(stamp, control_stamp);
  }

  ActiveTarget resolveActiveTarget(
    const SelectedTargetIdCache & selected_target,
    const TargetStateArrayCache & front_0,
    const TargetStateArrayCache & front_1,
    const TargetStateArrayCache & back,
    const OutpostStateCache & outpost,
    const rclcpp::Time & control_stamp) const
  {
    ActiveTarget active;
    if (!selected_target.valid || !selected_target.msg.valid) {
      return active;
    }

    const std::uint8_t outpost_id = outpost.valid ? outpost.msg.id : kDefaultOutpostId;
    if (selected_target.msg.id == outpost_id) {
      if (!outpost.valid || !outpost.msg.tracking || !outpost.msg.converged ||
        isTimedOut(outpost.msg.header.stamp, control_stamp))
      {
        return active;
      }
      active.model = aim_armor_controller::legacyTargetModelFromOutpostState(outpost.msg);
      active.stamp = rclcpp::Time(outpost.msg.header.stamp);
      active.source_frame = resolveSourceFrame(outpost.msg.header);
      active.measurement_age_sec = measurementAgeFromStamp(outpost.msg.header.stamp, control_stamp);
      active.is_outpost = true;
      active.valid = true;
      return active;
    }

    const auto match = aim_armor_controller::selectBestTargetStateMatch(
      selected_target.msg.id,
      front_0.valid ? &front_0.msg : nullptr,
      front_1.valid ? &front_1.msg : nullptr,
      back.valid ? &back.msg : nullptr);
    if (
      !match.has_value() ||
      !aim_armor_controller::isTargetStateReadyForControl(match->target) ||
      isTimedOut(match->header.stamp, control_stamp))
    {
      return active;
    }

    active.model =
      aim_armor_controller::legacyTargetModelFromTargetState(match->header, match->target);
    active.stamp = rclcpp::Time(match->header.stamp);
    active.source_frame = resolveSourceFrame(match->header);
    active.measurement_age_sec = measurementAgeFromStamp(match->header.stamp, control_stamp);
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
    const rclcpp::Time & control_stamp,
    ArmorCandidate & selected,
    aim_armor_controller::LegacyTargetModel & best_target,
    std::string & failure_reason)
  {
    failure_reason.clear();
    const auto & target = active_target.model;
    best_target = target;

    double center_bx = 0.0;
    double center_by = 0.0;
    double center_bz = 0.0;
    if (!transformWorldToBarrel(
        target.center.x, target.center.y, target.center.z, active_target.source_frame,
        center_bx, center_by, center_bz, control_stamp))
    {
      failure_reason = "center_tf_lookup_failed";
      return false;
    }

    double center_pitch = 0.0;
    double center_yaw = 0.0;
    double center_time = 0.0;
    if (!calcPitchYawWithShootTable(
        center_pitch, center_yaw, center_time, center_bx, center_by, center_bz))
    {
      failure_reason = "center_ballistic_solve_failed";
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
      failure_reason = "processing_delay_exceeded_max";
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
        predicted_target, &low_speed_selection_target, lock_index_, enable_smart_selector_,
        smart_selector_max_angular_velocity_, comming_angle_rad_,
        leaving_angle_rad_, selector_response_speed_rad_s_,
        selector_min_angular_velocity_rad_s_, allow_predicted_outpost_slots_);
      if (!final_aim.valid) {
        failure_reason = "outpost_armor_selection_failed_no_visible_slot";
        return false;
      }
      selected_idx = final_aim.armor_index;
      best_target = predicted_target;
    } else {
      auto predicted_target = target;
      aim_armor_controller::predictLegacyTarget(predicted_target, predict_time);
      auto aim_candidate = aim_armor_controller::chooseLegacyAimPoint(
        predicted_target, &low_speed_selection_target, lock_index_, enable_smart_selector_,
        smart_selector_max_angular_velocity_, comming_angle_rad_,
        leaving_angle_rad_, selector_response_speed_rad_s_,
        selector_min_angular_velocity_rad_s_, allow_predicted_outpost_slots_);
      if (!aim_candidate.valid) {
        failure_reason = "normal_armor_selection_failed";
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
        aim_candidate = aim_armor_controller::chooseLegacyAimPoint(
          candidate_target, &low_speed_selection_target, lock_index_, enable_smart_selector_,
          smart_selector_max_angular_velocity_, comming_angle_rad_,
          leaving_angle_rad_, selector_response_speed_rad_s_,
          selector_min_angular_velocity_rad_s_, allow_predicted_outpost_slots_);
        if (!aim_candidate.valid) {
          failure_reason = "normal_iterative_armor_selection_failed";
          return false;
        }

        double aim_bx = 0.0;
        double aim_by = 0.0;
        double aim_bz = 0.0;
        if (!transformWorldToBarrel(
            aim_candidate.point_world.x, aim_candidate.point_world.y, aim_candidate.point_world.z,
            active_target.source_frame, aim_bx, aim_by, aim_bz, control_stamp))
        {
          failure_reason = "candidate_tf_lookup_failed";
          return false;
        }

        double iter_pitch = 0.0;
        double iter_yaw = 0.0;
        double iter_fly_time = 0.0;
        if (!calcPitchYawWithShootTable(
            iter_pitch, iter_yaw, iter_fly_time, aim_bx, aim_by, aim_bz))
        {
          failure_reason = "candidate_ballistic_solve_failed";
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
      failure_reason = "candidate_invalid_after_selection";
      return false;
    }
    if (active_target.is_outpost) {
      const auto armors = aim_armor_controller::buildLegacyArmors(best_target);
      if (selected_idx >= static_cast<int>(armors.size())) {
        failure_reason = "selected_armor_index_out_of_range";
        return false;
      }
      const double facing_error = std::abs(aim_armor_controller::legacyArmorFacingError(
        best_target, armors[static_cast<std::size_t>(selected_idx)]));
      if (outpost_shoot_yaw_gate_rad_ > 0.0 &&
        facing_error > outpost_shoot_yaw_gate_rad_) {
        failure_reason = "outpost_facing_gate_rejected";
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "fallback detail: outpost facing_error=%.3f deg gate=%.3f deg armor_index=%d",
          facing_error * 180.0 / kPi,
          outpost_shoot_yaw_gate_rad_ * 180.0 / kPi, selected_idx);
        return false;
      }
    }
    if (!buildCandidateFromAimPoint(
        final_aim.point_world, active_target.source_frame, control_stamp, gimbal, selected))
    {
      failure_reason = "final_candidate_build_failed";
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

  static aim_armor_controller::TargetModel makeMpcTargetModel(
    const aim_armor_controller::LegacyTargetModel & source,
    bool is_outpost)
  {
    aim_armor_controller::TargetModel target;
    target.header = source.header;
    target.id = source.id;
    target.tracking = true;
    target.converged = true;
    target.jumped = source.jumped;
    target.center = source.center;
    target.velocity = source.velocity;
    target.yaw = source.yaw;
    target.angular_velocity = source.angular_velocity;
    target.radius = source.radius;
    target.low_height_offset = source.low_height_offset;
    target.high_height_offset = source.high_height_offset;
    target.radius_offset = source.radius_offset;
    target.height_offset = source.height_offset;
    target.armor_count = is_outpost ? 3 : 4;
    target.has_primary_armor = source.has_primary_armor;
    target.primary_slot = source.primary_slot;
    target.visible_slots = source.visible_slots;
    return target;
  }

  static aim_armor_controller::LegacyTargetModel makeLegacyDebugTarget(
    const aim_armor_controller::TargetModel & source)
  {
    aim_armor_controller::LegacyTargetModel target;
    target.header = source.header;
    target.id = source.id;
    target.jumped = source.jumped;
    target.center = source.center;
    target.velocity = source.velocity;
    target.yaw = source.yaw;
    target.angular_velocity = source.angular_velocity;
    target.radius = source.radius;
    target.low_height_offset = source.low_height_offset;
    target.high_height_offset = source.high_height_offset;
    target.radius_offset = source.radius_offset;
    target.height_offset = source.height_offset;
    target.armor_count = source.armor_count;
    target.has_primary_armor = source.has_primary_armor;
    target.primary_slot = source.primary_slot;
    target.visible_slots = source.visible_slots;
    return target;
  }

  static geometry_msgs::msg::Point applyMpcTargetOffset(
    const geometry_msgs::msg::Point & point,
    const aim_armor_controller::AimComputationConfig & config)
  {
    geometry_msgs::msg::Point adjusted = point;
    adjusted.x += config.target_offset.x;
    adjusted.y += config.target_offset.y;
    adjusted.z += config.target_offset.z;
    return adjusted;
  }

  bool lookupMpcTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform)
  {
    try {
      const auto tf_stamp =
        use_current_time_for_tf_ ? tf2::TimePointZero : tf2_ros::fromMsg(stamp);
      transform = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf_stamp,
        tf2::durationFromSec(target_tf_timeout_sec_));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "MPC tf lookup %s <- %s failed: %s",
        target_frame.c_str(), source_frame.c_str(), ex.what());
      return false;
    }
  }

  static bool transformMpcPoint(
    const geometry_msgs::msg::PointStamped & source,
    const geometry_msgs::msg::TransformStamped & transform,
    geometry_msgs::msg::PointStamped & target)
  {
    tf2::doTransform(source, target, transform);
    return true;
  }

  bool transformMpcAimPointToWorld(
    const geometry_msgs::msg::Point & point,
    const std::string & source_frame,
    const rclcpp::Time & stamp,
    const geometry_msgs::msg::TransformStamped * target_to_world,
    geometry_msgs::msg::PointStamped & world_point) const
  {
    geometry_msgs::msg::PointStamped source_point;
    source_point.header.stamp = stamp;
    source_point.header.frame_id = source_frame;
    source_point.point = point;
    if (source_frame == world_frame_id_) {
      world_point = source_point;
      world_point.header.frame_id = world_frame_id_;
      return true;
    }
    if (target_to_world == nullptr) {
      return false;
    }
    return transformMpcPoint(source_point, *target_to_world, world_point);
  }

  double computeMpcDelayTime(
    const aim_armor_controller::TargetModel & target,
    const std::string & target_frame_id,
    const geometry_msgs::msg::TransformStamped * target_to_world,
    const geometry_msgs::msg::TransformStamped & world_to_gun,
    const rclcpp::Time & measurement_time,
    const rclcpp::Time & processing_time,
    double bullet_speed,
    bool is_outpost)
  {
    const auto solve_world_point =
      [&](const geometry_msgs::msg::Point & point) {
        aim_armor_controller::MpcTrajectorySolution traj;
        geometry_msgs::msg::PointStamped point_world;
        const geometry_msgs::msg::Point adjusted_point =
          applyMpcTargetOffset(point, mpc_aim_config_);
        if (!transformMpcAimPointToWorld(
            adjusted_point, target_frame_id, measurement_time, target_to_world, point_world))
        {
          traj.unsolvable = true;
          return traj;
        }
        return aim_armor_controller::solveMpcTrajectory(
          bullet_speed,
          point_world.point.x - world_to_gun.transform.translation.x,
          point_world.point.y - world_to_gun.transform.translation.y,
          point_world.point.z - world_to_gun.transform.translation.z,
          use_air_resistance_, muzzle_offset_x_);
      };

    double fly_time = 0.0;
    const auto center_traj = solve_world_point(target.center);
    if (!center_traj.unsolvable) {
      fly_time = center_traj.fly_time;
    }

    if (is_outpost) {
      const auto selected_index = aim_armor_controller::chooseOutpostArmorIndex(target);
      if (selected_index.has_value()) {
        const auto selected_armor =
          aim_armor_controller::makeAimCandidate(target, *selected_index);
        if (selected_armor.valid) {
          const auto armor_traj = solve_world_point(selected_armor.point_world);
          if (!armor_traj.unsolvable) {
            fly_time = armor_traj.fly_time;
          }
        }
      }
    }

    const double raw_processing_delay =
      aim_armor_controller::ageAtControlStamp(target.header.stamp, processing_time);
    double processing_delay = include_processing_delay_ ? raw_processing_delay : 0.0;
    if (max_processing_delay_sec_ > 0.0) {
      processing_delay = std::min(processing_delay, max_processing_delay_sec_);
    }
    const double outpost_extra_delay = is_outpost ? outpost_extra_prediction_time_sec_ : 0.0;
    const double predict_time =
      fly_time + processing_delay + system_response_time_sec_ + outpost_extra_delay;
    return max_predict_time_sec_ > 0.0 ? std::min(predict_time, max_predict_time_sec_) :
           predict_time;
  }

  void processMpcTarget(
    const ActiveTarget & active_target,
    const GimbalState & gimbal,
    const rclcpp::Time & control_stamp,
    const std::deque<GimbalStateSample> & gimbal_history)
  {
    if (!mpc_planner_) {
      publishFallback(gimbal, "mpc_planner_missing");
      return;
    }

    double bullet_speed = bullet_speed_;
    if (bullet_speed < min_bullet_speed_) {
      bullet_speed = fallback_bullet_speed_;
    }

    const auto target = makeMpcTargetModel(active_target.model, active_target.is_outpost);
    const rclcpp::Time measurement_time(target.header.stamp);
    const auto tf_lookup_stamp = use_current_time_for_tf_ ?
      builtin_interfaces::msg::Time{} :
      target.header.stamp;

    std::optional<geometry_msgs::msg::TransformStamped> target_to_world;
    const geometry_msgs::msg::TransformStamped * target_to_world_ptr = nullptr;
    if (active_target.source_frame != world_frame_id_) {
      geometry_msgs::msg::TransformStamped transform;
      if (!lookupMpcTransform(
          world_frame_id_, active_target.source_frame, tf_lookup_stamp, transform))
      {
        publishFallback(gimbal, "mpc_target_tf_lookup_failed");
        return;
      }
      target_to_world = transform;
      target_to_world_ptr = &target_to_world.value();
    }

    geometry_msgs::msg::TransformStamped world_to_gun;
    if (!lookupMpcTransform(
        world_frame_id_, barrel_joint_frame_, tf_lookup_stamp, world_to_gun))
    {
      publishFallback(gimbal, "mpc_gun_tf_lookup_failed");
      return;
    }

    const bool high_speed_armor_selection =
      !active_target.is_outpost && target.jumped &&
      std::abs(target.angular_velocity) >
      mpc_aim_config_.selection.low_speed_angular_velocity_threshold;
    const double delay_time = computeMpcDelayTime(
      target, active_target.source_frame, target_to_world_ptr, world_to_gun,
      measurement_time, control_stamp, bullet_speed, active_target.is_outpost);

    double aim_lock_index = lock_index_;
    if (high_speed_armor_selection && !high_speed_armor_lock_active_) {
      aim_lock_index = -1.0;
    }

    const auto aim = aim_armor_controller::computeAimAngles(
      target, active_target.source_frame, world_frame_id_, target_to_world_ptr, world_to_gun,
      measurement_time, bullet_speed, delay_time, aim_lock_index, mpc_aim_config_,
      transformMpcPoint);
    if (!aim.valid) {
      std::string reason = "mpc_aim_compute_failed";
      if (!aim.failure_reason.empty()) {
        reason += ":";
        reason += aim.failure_reason;
      }
      publishFallback(gimbal, reason);
      return;
    }

    const std::vector<GimbalStateSample> gimbal_history_vector(
      gimbal_history.begin(), gimbal_history.end());
    const auto reference = aim_armor_controller::buildMpcReference(
      target, active_target.source_frame, world_frame_id_, target_to_world_ptr, world_to_gun,
      measurement_time, bullet_speed, delay_time, mpc_config_.yaw.dt, mpc_config_.yaw.horizon,
      aim.yaw_rad, aim_lock_index, mpc_aim_config_, transformMpcPoint, gimbal_history_vector);
    if (!reference.has_value()) {
      publishFallback(gimbal, "mpc_reference_build_failed");
      return;
    }
    lock_index_ = aim_lock_index;
    high_speed_armor_lock_active_ = high_speed_armor_selection;

    aim_armor_controller::MpcPlan plan;
    try {
      plan = mpc_planner_->solve(*reference);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "MPC solve failed: %s", ex.what());
      publishFallback(gimbal, "mpc_solve_exception");
      return;
    }
    if (!plan.valid) {
      publishFallback(gimbal, "mpc_solve_not_converged");
      return;
    }

    const double plan_relative_yaw_rad = plan.yaw;
    plan.yaw = aim_armor_controller::mpcLimitRad(plan.yaw + aim.yaw_rad);
    const double output_yaw_rad =
      aim_armor_controller::unwrapAngleNear(plan.yaw, gimbal.yaw_rad);
    const double output_pitch_rad = plan.pitch;
    const double output_yaw_deg = output_yaw_rad * 180.0 / kPi;
    const double output_pitch_deg = output_pitch_rad * 180.0 / kPi;
    if (
      !std::isfinite(output_yaw_deg) || !std::isfinite(output_pitch_deg) ||
      !std::isfinite(plan.yaw_velocity) || !std::isfinite(plan.pitch_velocity) ||
      !std::isfinite(plan.yaw_acceleration) || !std::isfinite(plan.pitch_acceleration))
    {
      publishFallback(gimbal, "mpc_output_non_finite");
      return;
    }
    publishControlTrajectory(
      control_stamp, output_yaw_deg, output_pitch_deg,
      plan.yaw_velocity * 180.0 / kPi, plan.pitch_velocity * 180.0 / kPi,
      plan.yaw_acceleration * 180.0 / kPi, plan.pitch_acceleration * 180.0 / kPi);

    double debug_bx = 0.0;
    double debug_by = 0.0;
    double debug_bz = 0.0;
    geometry_msgs::msg::PointStamped aim_world;
    const rclcpp::Time aim_time =
      measurement_time + rclcpp::Duration::from_seconds(delay_time);
    if (transformMpcAimPointToWorld(
        aim.candidate.point_world, active_target.source_frame, aim_time,
        target_to_world_ptr, aim_world))
    {
      const bool has_barrel_debug = transformWorldToBarrel(
        aim_world.point.x, aim_world.point.y, aim_world.point.z, world_frame_id_,
        debug_bx, debug_by, debug_bz, control_stamp);
      if (has_barrel_debug) {
        geometry_msgs::msg::PointStamped debug_point;
        debug_point.header.stamp = control_stamp;
        debug_point.header.frame_id = barrel_joint_frame_;
        debug_point.point.x = debug_bx;
        debug_point.point.y = debug_by;
        debug_point.point.z = debug_bz;
        debug_target_point_pub_->publish(debug_point);
      }
      if (debug_aim_geometry_log_) {
        const double solver_x =
          aim_world.point.x - world_to_gun.transform.translation.x;
        const double solver_y =
          aim_world.point.y - world_to_gun.transform.translation.y;
        const double solver_z =
          aim_world.point.z - world_to_gun.transform.translation.z;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[DEBUG-muzzle-aim] mode=mpc src=%s id=%u selected=%d muzzle_x=%.3f "
          "bullet=%.3f delay=%.4f target_center=(%.3f, %.3f, %.3f) "
          "target_velocity=(%.3f, %.3f, %.3f) target_yaw=%.3f target_omega=%.3f "
          "aim_world=(%.3f, %.3f, %.3f) gun_world=(%.3f, %.3f, %.3f) "
          "solver_xyz=(%.3f, %.3f, %.3f) solver_d=%.3f "
          "raw_yaw=%.3f raw_pitch=%.3f fly_time=%.4f "
          "aim_yaw=%.3f aim_pitch=%.3f plan_rel_yaw=%.3f "
          "output=(%.3f, %.3f) gimbal=(%.3f, %.3f) "
          "barrel_debug=(%.3f, %.3f, %.3f) barrel_debug_valid=%s",
          active_target.source_frame.c_str(),
          static_cast<unsigned int>(active_target.model.id),
          aim.candidate.armor_index,
          muzzle_offset_x_,
          bullet_speed,
          delay_time,
          aim.predicted_target.center.x,
          aim.predicted_target.center.y,
          aim.predicted_target.center.z,
          aim.predicted_target.velocity.x,
          aim.predicted_target.velocity.y,
          aim.predicted_target.velocity.z,
          aim.predicted_target.yaw,
          aim.predicted_target.angular_velocity,
          aim_world.point.x,
          aim_world.point.y,
          aim_world.point.z,
          world_to_gun.transform.translation.x,
          world_to_gun.transform.translation.y,
          world_to_gun.transform.translation.z,
          solver_x,
          solver_y,
          solver_z,
          std::hypot(solver_x, solver_y),
          aim.trajectory.yaw * 180.0 / kPi,
          aim.trajectory.pitch * 180.0 / kPi,
          aim.trajectory.fly_time,
          aim.yaw_rad * 180.0 / kPi,
          aim.pitch_rad * 180.0 / kPi,
          plan_relative_yaw_rad * 180.0 / kPi,
          output_yaw_deg,
          output_pitch_deg,
          static_cast<double>(gimbal.yaw_deg),
          static_cast<double>(gimbal.pitch_deg),
          debug_bx,
          debug_by,
          debug_bz,
          has_barrel_debug ? "true" : "false");
      }
    }
    const auto debug_target = makeLegacyDebugTarget(aim.predicted_target);
    publishSelectedArmorDebug(active_target, debug_target, aim.candidate.armor_index);

    if (publish_control_angles_ && angle_pub_) {
      gimbal_driver::msg::GimbalAngles angle_msg;
      angle_msg.header.stamp = control_stamp;
      angle_msg.yaw = static_cast<float>(output_yaw_deg);
      angle_msg.pitch = static_cast<float>(output_pitch_deg);
      angle_pub_->publish(angle_msg);
    }

    const bool fire_allowed = aim.candidate.fire_allowed;
    double outpost_facing_error_deg = 0.0;
    bool outpost_facing_allowed = true;
    if (active_target.is_outpost) {
      const double outpost_facing_error_rad = aim_armor_controller::outpostArmorFacingError(
        aim.predicted_target.center.x, aim.predicted_target.center.y,
        aim.candidate.armor_yaw_world);
      outpost_facing_error_deg = outpost_facing_error_rad * 180.0 / kPi;
      outpost_facing_allowed =
        outpost_shoot_yaw_gate_rad_ <= 0.0 ||
        std::abs(outpost_facing_error_rad) <= outpost_shoot_yaw_gate_rad_;
    }

    const double active_fire_rate_hz = activeFireRateHz();
    bool fire_this_tick = false;
    if (
      !manual_fire_mode_ && !active_target.tracking_hold && fire_allowed &&
      outpost_facing_allowed && plan.fire && active_fire_rate_hz > 0.0)
    {
      const double elapsed = (now() - last_fire_time_).seconds();
      if (elapsed >= 1.0 / active_fire_rate_hz) {
        aim_armor_controller::toggleFireStatus(fire_code_state_);
        last_fire_time_ = now();
        fire_this_tick = true;
      }
    }
    if (!fire_this_tick) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "fire gate blocked: mode=mpc manual=%s tracking_hold=%s candidate_fire=%s "
        "outpost_facing=%s plan_fire=%s fire_rate_hz=%.3f fire_error=%.6f "
        "fire_thresh=%.6f outpost_facing_error_deg=%.3f "
        "outpost_gate_deg=%.3f selected_armor=%d is_outpost=%s",
        manual_fire_mode_ ? "true" : "false",
        active_target.tracking_hold ? "true" : "false",
        fire_allowed ? "true" : "false",
        outpost_facing_allowed ? "true" : "false",
        plan.fire ? "true" : "false",
        active_fire_rate_hz,
        plan.fire_error,
        mpc_config_.fire_threshold,
        outpost_facing_error_deg,
        outpost_shoot_yaw_gate_rad_ * 180.0 / kPi,
        aim.candidate.armor_index,
        active_target.is_outpost ? "true" : "false");
    }

    fire_code_state_.aim_mode = true;
    fire_code_state_.rotate = 1U;

    sentry_msgs::msg::AimResult aim_msg;
    aim_msg.header.stamp = control_stamp;
    aim_msg.header.frame_id = barrel_joint_frame_;
    aim_msg.follow = true;
    aim_msg.fire = fire_this_tick;
    aim_msg.pitch = static_cast<float>(output_pitch_deg);
    aim_msg.yaw = static_cast<float>(output_yaw_deg);
    aim_armor_controller::setAimResultKinematics(
      aim_msg, output_yaw_deg, output_pitch_deg,
      plan.yaw_velocity * 180.0 / kPi,
      plan.pitch_velocity * 180.0 / kPi,
      plan.yaw_acceleration * 180.0 / kPi,
      plan.pitch_acceleration * 180.0 / kPi);
    aim_result_pub_->publish(aim_msg);

    if (publish_legacy_control_topics_ && !manual_fire_mode_ && fire_pub_) {
      fire_pub_->publish(makeFireCodeMsg(now(), fire_code_state_));
    }
  }

  void onTimer()
  {
    TargetStateArrayCache front_0;
    TargetStateArrayCache front_1;
    TargetStateArrayCache back;
    OutpostStateCache outpost;
    SelectedTargetIdCache selected_target;
    GimbalState gimbal;
    std::deque<GimbalStateSample> gimbal_history;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      front_0 = front_0_cache_;
      front_1 = front_1_cache_;
      back = back_cache_;
      outpost = outpost_cache_;
      selected_target = selected_target_cache_;
      gimbal = gimbal_state_;
      gimbal_history = gimbal_state_history_;
    }

    const auto current_steady_time = std::chrono::steady_clock::now();
    const rclcpp::Time current_time = now();
    if (!gimbal.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "fallback: reason=gimbal_feedback_missing");
      return;
    }
    if (!aim_armor_controller::isFeedbackFresh(
        gimbal.received_at, current_steady_time, gimbal_state_timeout_sec_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "fallback: reason=gimbal_feedback_stale");
      return;
    }

    const rclcpp::Time control_stamp =
      aim_armor_controller::resolveControlStamp(gimbal.stamp, current_time);
    ActiveTarget active_target =
      resolveActiveTarget(selected_target, front_0, front_1, back, outpost, control_stamp);
    const std::uint8_t outpost_id = outpost.valid ? outpost.msg.id : kDefaultOutpostId;
    const bool selected_outpost =
      selected_target.valid && selected_target.msg.valid && selected_target.msg.id == outpost_id;
    if (active_target.valid && active_target.is_outpost) {
      last_valid_outpost_target_ = active_target;
      last_valid_outpost_control_stamp_ = control_stamp;
      has_last_valid_outpost_target_ = true;
    } else {
      const double elapsed_sec = has_last_valid_outpost_target_ ?
        std::max(0.0, (control_stamp - last_valid_outpost_control_stamp_).seconds()) :
        std::numeric_limits<double>::infinity();
      const bool outpost_has_visible_slot =
        outpost.valid && std::any_of(
        outpost.msg.visible_slots.begin(), outpost.msg.visible_slots.end(),
        [](bool visible) { return visible; });
      if (
        outpost_has_visible_slot &&
        aim_armor_controller::shouldHoldOutpostTarget(
          selected_outpost, has_last_valid_outpost_target_, elapsed_sec, outpost_tracking_hold_sec_) &&
        !isTimedOut(last_valid_outpost_target_.model.header.stamp, control_stamp))
      {
        active_target = last_valid_outpost_target_;
        active_target.measurement_age_sec =
          measurementAgeFromStamp(active_target.model.header.stamp, control_stamp);
        active_target.tracking_hold = true;
      } else if (!selected_outpost || !outpost_has_visible_slot ||
        elapsed_sec > outpost_tracking_hold_sec_) {
        has_last_valid_outpost_target_ = false;
      }
    }
    if (!active_target.valid) {
      high_speed_armor_lock_active_ = false;
      lock_target_valid_ = false;
      lock_index_ = -1.0;
      publishFallback(
        gimbal,
        activeTargetFailureReason(
          selected_target, front_0, front_1, back, outpost, control_stamp));
      return;
    }

    if (
      !lock_target_valid_ ||
      lock_target_id_ != active_target.model.id ||
      lock_target_is_outpost_ != active_target.is_outpost)
    {
      lock_index_ = -1.0;
      high_speed_armor_lock_active_ = false;
      lock_target_id_ = active_target.model.id;
      lock_target_is_outpost_ = active_target.is_outpost;
      lock_target_valid_ = true;
    }

    if (enable_mpc_) {
      processMpcTarget(active_target, gimbal, control_stamp, gimbal_history);
      return;
    }

    ArmorCandidate selected;
    aim_armor_controller::LegacyTargetModel best_target;
    std::string failure_reason;
    if (!selectArmorCandidate(
        active_target, gimbal, control_stamp, selected, best_target, failure_reason))
    {
      publishFallback(
        gimbal,
        failure_reason.empty() ? "armor_candidate_selection_failed" : failure_reason);
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
    const auto command = limitCommandAngles(
      selected.yaw_actual_want_deg, selected.pitch_actual_want_deg, gimbal, control_stamp);
    if (!std::isfinite(command.yaw_deg) || !std::isfinite(command.pitch_deg)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "fallback: reason=legacy_command_non_finite selected=(%.6f, %.6f)",
        selected.yaw_actual_want_deg, selected.pitch_actual_want_deg);
      publishFallback(gimbal, "legacy_command_non_finite");
      return;
    }

    if (publish_control_angles_ && angle_pub_) {
      gimbal_driver::msg::GimbalAngles angle_msg;
      angle_msg.header.stamp = control_stamp;
      angle_msg.yaw = static_cast<float>(command.yaw_deg);
      angle_msg.pitch = static_cast<float>(command.pitch_deg);
      angle_pub_->publish(angle_msg);
    }
    publishControlTrajectory(
      control_stamp, command.yaw_deg, command.pitch_deg, 0.0, 0.0, 0.0, 0.0);

    // The legacy direct-control path explicitly enables aim mode and the
    // existing low rotate level. These fields are independent of the
    // fire_status 0 <-> 3 toggle.
    fire_code_state_.aim_mode = true;
    fire_code_state_.rotate = 1U;

    const double active_fire_rate_hz = activeFireRateHz();
    bool fire_this_tick = false;
    if (
      !manual_fire_mode_ && !active_target.tracking_hold &&
      fire_control_output.shoot_flag && active_fire_rate_hz > 0.0)
    {
      const double elapsed = (now() - last_fire_time_).seconds();
      if (elapsed >= 1.0 / active_fire_rate_hz) {
        aim_armor_controller::toggleFireStatus(fire_code_state_);
        last_fire_time_ = now();
        fire_this_tick = true;
      }
    }
    if (!fire_this_tick) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "fire gate blocked: mode=legacy manual=%s tracking_hold=%s shoot_flag=%s "
        "gimbal_ready=%s yaw_error_deg=%.3f pitch_error_deg=%.3f "
        "yaw_tol_deg=%.3f pitch_tol_deg=%.3f inside_face_window=%s "
        "fire_rate_hz=%.3f selected_armor=%d is_outpost=%s",
        manual_fire_mode_ ? "true" : "false",
        active_target.tracking_hold ? "true" : "false",
        fire_control_output.shoot_flag ? "true" : "false",
        fire_control_output.gimbal_ready ? "true" : "false",
        yaw_error_deg,
        pitch_error_deg,
        shoot_yaw_tolerance_deg_,
        shoot_pitch_tolerance_deg_,
        inside_face_window ? "true" : "false",
        active_fire_rate_hz,
        selected.armor_index,
        active_target.is_outpost ? "true" : "false");
    }

    sentry_msgs::msg::AimResult aim_msg;
    aim_msg.header.stamp = control_stamp;
    aim_msg.header.frame_id = barrel_joint_frame_;
    // A successful target/angle solve is the follow contract. Fire is gated
    // independently below and must not be used as the follow signal.
    aim_msg.follow = true;
    aim_msg.fire = fire_this_tick;
    aim_msg.pitch = static_cast<float>(command.pitch_deg);
    aim_msg.yaw = static_cast<float>(command.yaw_deg);
    aim_armor_controller::setAimResultKinematics(
      aim_msg, command.yaw_deg, command.pitch_deg, 0.0, 0.0, 0.0, 0.0);
    aim_result_pub_->publish(aim_msg);

    if (publish_legacy_control_topics_ && !manual_fire_mode_ && fire_pub_) {
      fire_pub_->publish(makeFireCodeMsg(now(), fire_code_state_));
    }
  }

  std::mutex data_mutex_;
  TargetStateArrayCache front_0_cache_;
  TargetStateArrayCache front_1_cache_;
  TargetStateArrayCache back_cache_;
  OutpostStateCache outpost_cache_;
  SelectedTargetIdCache selected_target_cache_;
  GimbalState gimbal_state_;
  std::deque<GimbalStateSample> gimbal_state_history_;
  ActiveTarget last_valid_outpost_target_;
  rclcpp::Time last_valid_outpost_control_stamp_{0, 0, RCL_ROS_TIME};
  bool has_last_valid_outpost_target_{false};

  bool shoot_table_adjust_{false};
  std::vector<double> pitch_param_;
  std::vector<double> yaw_param_;
  double x_offset_{0.0};
  double y_offset_{0.0};
  double z_offset_{0.0};
  std::string barrel_joint_frame_{"gimbal_barrel_joint"};
  double target_tf_timeout_sec_{0.02};
  double target_msg_timeout_sec_{0.10};
  double gimbal_state_timeout_sec_{0.10};
  double outpost_tracking_hold_sec_{0.15};
  double max_angle_rate_deg_per_sec_{60.0};
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
  bool publish_control_angles_{false};
  bool publish_control_trajectory_{false};
  bool manual_fire_mode_{false};
  bool enable_smart_selector_{true};
  double comming_angle_rad_{60.0 * kPi / 180.0};
  double leaving_angle_rad_{20.0 * kPi / 180.0};
  double smart_selector_max_angular_velocity_{2.0};
  double low_speed_angular_velocity_threshold_{2.0};
  double selector_response_speed_rad_s_{0.01};
  double selector_min_angular_velocity_rad_s_{0.6};
  bool allow_predicted_outpost_slots_{true};
  double lock_index_{-1.0};
  bool high_speed_armor_lock_active_{false};
  bool lock_target_valid_{false};
  std::uint8_t lock_target_id_{0};
  bool lock_target_is_outpost_{false};
  double outpost_shoot_yaw_gate_rad_{60.0 * kPi / 180.0};
  bool enable_mpc_{true};
  std::string world_frame_id_{"gimbal_world"};
  bool use_current_time_for_tf_{false};
  bool debug_aim_geometry_log_{false};
  aim_armor_controller::AimComputationConfig mpc_aim_config_;
  double gimbal_state_history_sec_{2.0};
  aim_armor_controller::MpcPlannerConfig mpc_config_;
  std::unique_ptr<aim_armor_controller::MpcPlanner> mpc_planner_;
  double bullet_speed_{23.0};
  double min_bullet_speed_{20.0};
  double fallback_bullet_speed_{23.0};
  bool use_air_resistance_{true};
  double bullet_speed_alpha_{0.5};
  double outpost_extra_prediction_time_sec_{0.0};
  double tol_deltax_{0.2};
  double tol_deltay_{0.1};
  double bullet_mass_{3.2e-3};
  double bullet_diameter_{16.8e-3};
  double muzzle_offset_x_{0.0};
  int max_iter_{100};
  double tol_{1e-6};
  aim_armor_controller::FireCodeState fire_code_state_{};
  rclcpp::Time last_fire_time_{0, 0, RCL_ROS_TIME};
  double fire_rate_hz_{10.0};
  double overclock_fire_rate_hz_{20.0};
  bool overclock_mode_{false};
  aim_armor_controller::LegacyFireControlState legacy_fire_control_state_{};
  aim_armor_controller::CommandRateLimiter command_rate_limiter_;
  rclcpp::Time last_command_limit_stamp_{0, 0, RCL_ROS_TIME};
  bool command_rate_limiter_initialized_{false};

  rclcpp::Publisher<sentry_msgs::msg::AimResult>::SharedPtr aim_result_pub_;
  rclcpp::Publisher<aim_armor_controller::ControllerTrajectoryMessage>::SharedPtr trajectory_pub_;
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
  rclcpp::Subscription<gimbal_driver::msg::GimbalState>::SharedPtr gimbal_state_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr posture_sub_;
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
