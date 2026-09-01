#include "tracker/tracker_node.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "aim_msgs/msg/armor_pose_observation.hpp"
#include "aim_msgs/msg/armor_pose_set_array.hpp"
#include "aim_msgs/msg/target_state_array.hpp"

namespace
{

using namespace std::chrono_literals;

class TrackerNodeTest : public ::testing::Test
{
protected:
  static std::shared_ptr<tracker::TrackerNode> makeTestNode(int64_t min_detections = 1)
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override(
      "min_consecutive_detections_to_track", min_detections);
    return std::make_shared<tracker::TrackerNode>(options);
  }

  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  static builtin_interfaces::msg::Time stamp(std::int32_t seconds)
  {
    builtin_interfaces::msg::Time result;
    result.sec = seconds;
    return result;
  }

  static aim_msgs::msg::ArmorPoseSet makeSet(
    std::uint8_t id, double x, double y, double z)
  {
    aim_msgs::msg::ArmorPoseSet result;
    result.id = id;
    aim_msgs::msg::ArmorPoseObservation observation;
    observation.pose.position.x = x;
    observation.pose.position.y = y;
    observation.pose.position.z = z;
    observation.pose.orientation.w = 1.0;
    observation.camera_pose = observation.pose;
    observation.armor_class.class_id = 0;
    for (auto & corner : observation.corners) {
      corner.x = 640.0;
      corner.y = 480.0;
    }
    result.observations.push_back(observation);
    return result;
  }

  static aim_msgs::msg::ArmorPoseSetArray makeMessage(
    std::int32_t seconds, std::initializer_list<aim_msgs::msg::ArmorPoseSet> sets)
  {
    aim_msgs::msg::ArmorPoseSetArray result;
    result.header.stamp = stamp(seconds);
    result.header.frame_id = "gimbal_world";
    result.armor_pose_sets.assign(sets.begin(), sets.end());
    return result;
  }

  static void spinUntil(
    rclcpp::Executor & executor, const std::function<bool()> & predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some(10ms);
    }
  }
};

TEST_F(TrackerNodeTest, DeclaresStableTopicsAndPerCameraNoiseScales)
{
  auto node = makeTestNode(10);

  EXPECT_EQ(
    node->get_parameter("front_0_armor_pose_set_topic").as_string(),
    "/aim_solver/front_0/armor_pose_sets");
  EXPECT_EQ(
    node->get_parameter("front_1_armor_pose_set_topic").as_string(),
    "/aim_solver/front_1/armor_pose_sets");
  EXPECT_EQ(
    node->get_parameter("back_armor_pose_set_topic").as_string(),
    "/aim_solver/back/armor_pose_sets");
  EXPECT_EQ(
    node->get_parameter("fused_target_state_topic").as_string(),
    "/aim_predictor/fused/target_states");
  EXPECT_EQ(node->get_parameter("window_size").as_int(), 30);
  EXPECT_EQ(node->get_parameter("min_consecutive_detections_to_track").as_int(), 10);
  EXPECT_DOUBLE_EQ(node->get_parameter("front_0_noise_scale").as_double(), 1.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("front_1_noise_scale").as_double(), 1.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("back_noise_scale").as_double(), 1.0);
}

TEST_F(TrackerNodeTest, FusesSourcesByIdAndFiltersOutpost)
{
  auto node = makeTestNode();
  auto input = node->create_publisher<aim_msgs::msg::ArmorPoseSetArray>(
    "/aim_solver/front_0/armor_pose_sets", rclcpp::QoS(10));
  auto output = node->create_subscription<aim_msgs::msg::TargetStateArray>(
    "/aim_predictor/fused/target_states", rclcpp::QoS(10),
    [node](const aim_msgs::msg::TargetStateArray::SharedPtr message) {
      node->set_parameter(rclcpp::Parameter(
          "test_last_output_count", static_cast<int64_t>(message->targets.size())));
    });
  node->declare_parameter<int64_t>("test_last_output_count", 0);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  input->publish(makeMessage(10, {makeSet(3, 1.0, 0.0, 1.0), makeSet(6, 2.0, 0.0, 1.0)}));
  spinUntil(executor, [&node]() {
    return node->get_parameter("test_last_output_count").as_int() > 0;
  });

  EXPECT_EQ(node->get_parameter("test_last_output_count").as_int(), 1);
  (void)output;
}

TEST_F(TrackerNodeTest, EmptyInputPublishesPredictionAndOldInputIsIgnored)
{
  auto node = makeTestNode();
  auto input = node->create_publisher<aim_msgs::msg::ArmorPoseSetArray>(
    "/aim_solver/front_0/armor_pose_sets", rclcpp::QoS(10));
  std::size_t output_count = 0;
  aim_msgs::msg::TargetState last_state;
  auto output = node->create_subscription<aim_msgs::msg::TargetStateArray>(
    "/aim_predictor/fused/target_states", rclcpp::QoS(10),
    [&output_count, &last_state](const aim_msgs::msg::TargetStateArray::SharedPtr message) {
      ++output_count;
      if (!message->targets.empty()) {
        last_state = message->targets.front();
      }
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  input->publish(makeMessage(20, {makeSet(4, 1.0, 2.0, 3.0)}));
  spinUntil(executor, [&output_count]() { return output_count >= 1; });
  const auto state_after_measurement = last_state;
  input->publish(makeMessage(21, {}));
  spinUntil(executor, [&output_count]() { return output_count >= 2; });
  EXPECT_EQ(last_state.id, state_after_measurement.id);
  EXPECT_TRUE(last_state.tracking);
  EXPECT_GE(last_state.predicted_armors.size(), 4U);

  const auto count_before_old = output_count;
  input->publish(makeMessage(19, {makeSet(4, 99.0, 99.0, 99.0)}));
  executor.spin_some(20ms);
  EXPECT_EQ(output_count, count_before_old);
  (void)output;
}

TEST_F(TrackerNodeTest, MapsTargetStateFieldsAndSharesOneTargetAcrossCameras)
{
  auto node = makeTestNode();
  auto front0 = node->create_publisher<aim_msgs::msg::ArmorPoseSetArray>(
    "/aim_solver/front_0/armor_pose_sets", rclcpp::QoS(10));
  auto front1 = node->create_publisher<aim_msgs::msg::ArmorPoseSetArray>(
    "/aim_solver/front_1/armor_pose_sets", rclcpp::QoS(10));
  std::vector<aim_msgs::msg::TargetState> states;
  auto output = node->create_subscription<aim_msgs::msg::TargetStateArray>(
    "/aim_predictor/fused/target_states", rclcpp::QoS(10),
    [&states](const aim_msgs::msg::TargetStateArray::SharedPtr message) {
      states = message->targets;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  front0->publish(makeMessage(30, {makeSet(5, 1.0, 2.0, 3.0)}));
  spinUntil(executor, [&states]() { return !states.empty(); });
  ASSERT_EQ(states.size(), 1U);
  const auto first_state = states.front();
  front1->publish(makeMessage(31, {makeSet(5, 1.1, 2.0, 3.0)}));
  spinUntil(executor, [&states]() { return !states.empty(); });

  ASSERT_EQ(states.size(), 1U);
  EXPECT_EQ(first_state.id, 5);
  EXPECT_TRUE(first_state.tracking);
  EXPECT_EQ(first_state.predicted_armors.size(), 4U);
  EXPECT_NEAR(first_state.center.x, 1.2, 1e-6);
  EXPECT_NEAR(first_state.center.y, 2.0, 1e-6);
  EXPECT_NEAR(first_state.center.z, 3.0, 1e-6);
  EXPECT_DOUBLE_EQ(first_state.velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(first_state.velocity.y, 0.0);
  EXPECT_DOUBLE_EQ(first_state.velocity.z, 0.0);
  EXPECT_DOUBLE_EQ(first_state.yaw, 0.0);
  EXPECT_DOUBLE_EQ(first_state.angular_velocity, 0.0);
  EXPECT_DOUBLE_EQ(first_state.radius, 0.2);
  EXPECT_DOUBLE_EQ(first_state.radius_offset, -0.01);
  EXPECT_DOUBLE_EQ(first_state.height_offset, 0.02);
  (void)output;
}

TEST_F(TrackerNodeTest, RemovesTrackerAfterTheConfiguredLossTimeout)
{
  auto node = makeTestNode();
  auto input = node->create_publisher<aim_msgs::msg::ArmorPoseSetArray>(
    "/aim_solver/front_0/armor_pose_sets", rclcpp::QoS(10));
  std::size_t output_count = 0;
  std::size_t last_target_count = 0;
  auto output = node->create_subscription<aim_msgs::msg::TargetStateArray>(
    "/aim_predictor/fused/target_states", rclcpp::QoS(10),
    [&output_count, &last_target_count](
      const aim_msgs::msg::TargetStateArray::SharedPtr message) {
      ++output_count;
      last_target_count = message->targets.size();
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  input->publish(makeMessage(40, {makeSet(8, 1.0, 0.0, 1.0)}));
  spinUntil(executor, [&output_count]() { return output_count >= 1; });
  input->publish(makeMessage(41, {}));
  spinUntil(executor, [&output_count]() { return output_count >= 2; });

  EXPECT_EQ(last_target_count, 0U);
  (void)output;
}

}  // namespace
