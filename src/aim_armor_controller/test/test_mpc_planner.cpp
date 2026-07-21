#include "aim_armor_controller/mpc_planner.hpp"

#include <cassert>
#include <cmath>

int main()
{
  aim_armor_controller::MpcPlannerConfig config;
  config.yaw.horizon = 10;
  config.pitch.horizon = 10;
  config.yaw.max_iterations = 20;
  config.pitch.max_iterations = 20;

  aim_armor_controller::MpcPlanner planner(config);
  aim_armor_controller::MpcReference reference;
  reference.yaw = Eigen::MatrixXd::Zero(2, 11);
  reference.pitch = Eigen::MatrixXd::Zero(2, 11);
  reference.yaw.row(0).setConstant(0.1);
  reference.pitch.row(0).setConstant(-0.05);

  const auto plan = planner.solve(reference);
  assert(plan.valid);
  assert(plan.fire);
  assert(std::isfinite(plan.yaw));
  assert(std::isfinite(plan.yaw_velocity));
  assert(std::isfinite(plan.yaw_acceleration));
  assert(std::isfinite(plan.pitch));
  assert(std::isfinite(plan.pitch_velocity));
  assert(std::isfinite(plan.pitch_acceleration));
  return 0;
}
