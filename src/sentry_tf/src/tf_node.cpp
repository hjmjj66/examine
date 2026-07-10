#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "gimbal_driver/msg/gimbal_angles.hpp"

namespace
{

class SentryTfNode : public rclcpp::Node
{
public:
  SentryTfNode()
  : Node("sentry_tf_node")
  {
    declare_parameter<std::string>("gimbal_topic", "/ly/gimbal/angles");
    declare_parameter<std::string>("world_frame",  "gimbal_world");
    declare_parameter<std::string>("yaw_frame",  "gimbal_small_yaw");
    declare_parameter<std::string>("barrel_joint_frame",  "gimbal_barrel_joint");
    declare_parameter<std::string>("barrel_frame",  "gimbal_barrel");
    declare_parameter<double>("barrel_offset_z", 0.0);

    world_frame_  = get_parameter("world_frame").as_string();
    yaw_frame_ = get_parameter("yaw_frame").as_string();
    barrel_joint_frame_ = get_parameter("barrel_joint_frame").as_string();
    barrel_frame_ = get_parameter("barrel_frame").as_string();
    barrel_offset_z_ = get_parameter("barrel_offset_z").as_double();
    const auto gimbal_topic = get_parameter("gimbal_topic").as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    publishBarrelOffsetStatic();
    
    gimbal_sub_ = create_subscription<gimbal_driver::msg::GimbalAngles>(
      gimbal_topic,
      rclcpp::SensorDataQoS(),
      [this](const gimbal_driver::msg::GimbalAngles::SharedPtr msg) {
        BroadcastGimbalTf(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "sentry_tf_node started. topic=%s, chain: %s -> %s and %s --static(+Z)-> %s --pitch-> %s "
      "(barrel_offset_z=%.3f m)",
      gimbal_topic.c_str(), yaw_frame_.c_str(), world_frame_.c_str(), yaw_frame_.c_str(),
      barrel_joint_frame_.c_str(), barrel_frame_.c_str(), barrel_offset_z_);
  }

private:
  void publishBarrelOffsetStatic()
  {
    geometry_msgs::msg::TransformStamped s;
    s.header.stamp.sec = 0;
    s.header.stamp.nanosec = 0U;
    s.header.frame_id = yaw_frame_;
    s.child_frame_id = barrel_joint_frame_;
    s.transform.translation.x = 0.0;
    s.transform.translation.y = 0.0;
    s.transform.translation.z = barrel_offset_z_;
    s.transform.rotation.w = 1.0;
    s.transform.rotation.x = 0.0;
    s.transform.rotation.y = 0.0;
    s.transform.rotation.z = 0.0;
    static_tf_broadcaster_->sendTransform(s);
  }

  void BroadcastGimbalTf(const gimbal_driver::msg::GimbalAngles & msg)
  {
    const double yaw_rad = static_cast<double>(msg.yaw) * M_PI / 180.0;
    const double pitch_rad = -static_cast<double>(msg.pitch) * M_PI / 180.0;

    tf2::Quaternion q_yaw;
    q_yaw.setRotation(tf2::Vector3(0.0, 0.0, 1.0), yaw_rad);
    q_yaw.normalize();

    tf2::Quaternion q_pitch;
    q_pitch.setRotation(tf2::Vector3(0.0, 1.0, 0.0), pitch_rad);
    q_pitch.normalize();

    const rclcpp::Time t = msg.header.stamp;

    const tf2::Quaternion q_yaw_inverse = q_yaw.inverse();

    geometry_msgs::msg::TransformStamped ts_world;
    ts_world.header.stamp = t;
    ts_world.header.frame_id = yaw_frame_;
    ts_world.child_frame_id = world_frame_;
    ts_world.transform.translation.x = 0.0;
    ts_world.transform.translation.y = 0.0;
    ts_world.transform.translation.z = 0.0;
    ts_world.transform.rotation.x = q_yaw_inverse.x();
    ts_world.transform.rotation.y = q_yaw_inverse.y();
    ts_world.transform.rotation.z = q_yaw_inverse.z();
    ts_world.transform.rotation.w = q_yaw_inverse.w();

    geometry_msgs::msg::TransformStamped ts_pitch;
    ts_pitch.header.stamp = t;
    ts_pitch.header.frame_id = barrel_joint_frame_;
    ts_pitch.child_frame_id = barrel_frame_;
    ts_pitch.transform.translation.x = 0.0;
    ts_pitch.transform.translation.y = 0.0;
    ts_pitch.transform.translation.z = 0.0;
    ts_pitch.transform.rotation.x = q_pitch.x();
    ts_pitch.transform.rotation.y = q_pitch.y();
    ts_pitch.transform.rotation.z = q_pitch.z();
    ts_pitch.transform.rotation.w = q_pitch.w();

    tf_broadcaster_->sendTransform({ts_world, ts_pitch});
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Subscription<gimbal_driver::msg::GimbalAngles>::SharedPtr gimbal_sub_;
  std::string world_frame_;
  std::string yaw_frame_;
  std::string barrel_joint_frame_;
  std::string barrel_frame_;
  double barrel_offset_z_{0.0};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SentryTfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
