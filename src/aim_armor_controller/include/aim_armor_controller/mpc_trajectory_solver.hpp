#ifndef AIM_ARMOR_CONTROLLER__MPC_TRAJECTORY_SOLVER_HPP_
#define AIM_ARMOR_CONTROLLER__MPC_TRAJECTORY_SOLVER_HPP_

#include <string>

namespace aim_armor_controller
{

struct MpcTrajectorySolution
{
  bool unsolvable{false};
  std::string failure_reason;
  double fly_time{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

MpcTrajectorySolution solveMpcTrajectory(
  double bullet_speed,
  double target_x,
  double target_y,
  double target_z,
  bool use_air_resistance = true);

}  // namespace aim_armor_controller

#endif  // AIM_ARMOR_CONTROLLER__MPC_TRAJECTORY_SOLVER_HPP_
