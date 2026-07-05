#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "aim_armor_detector/aim_armor_detector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aim_armor_detector::AimArmorDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
