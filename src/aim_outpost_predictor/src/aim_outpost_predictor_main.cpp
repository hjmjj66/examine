#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "aim_outpost_predictor/aim_outpost_predictor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aim_outpost_predictor::AimOutpostPredictorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
