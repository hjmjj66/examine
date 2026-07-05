#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "aim_predictor/aim_predictor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aim_predictor::AimPredictorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
