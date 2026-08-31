#include "tracker/tracker_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tracker::TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
