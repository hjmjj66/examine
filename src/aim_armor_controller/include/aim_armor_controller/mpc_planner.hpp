#ifndef AIM_ARMOR_CONTROLLER__MPC_PLANNER_HPP_
#define AIM_ARMOR_CONTROLLER__MPC_PLANNER_HPP_

#include <memory>

#include <Eigen/Dense>

namespace aim_armor_controller
{

struct AxisMpcConfig
{
  double dt{0.01};
  int horizon{100};
  double max_acceleration{50.0};
  double q_position{9.0e6};
  double q_velocity{0.0};
  double r_acceleration{1.0};
  int max_iterations{10};
};

struct MpcPlannerConfig
{
  AxisMpcConfig yaw;
  AxisMpcConfig pitch;
  double fire_threshold{0.0035};
  int fire_offset{2};
};

struct MpcReference
{
  Eigen::MatrixXd yaw;
  Eigen::MatrixXd pitch;
};

struct MpcPlan
{
  bool valid{false};
  bool fire{false};
  double fire_error{0.0};
  double yaw{0.0};
  double yaw_velocity{0.0};
  double yaw_acceleration{0.0};
  double pitch{0.0};
  double pitch_velocity{0.0};
  double pitch_acceleration{0.0};
};

class MpcPlanner
{
public:
  explicit MpcPlanner(const MpcPlannerConfig & config);
  ~MpcPlanner();

  MpcPlan solve(const MpcReference & reference);

private:
  struct AxisSolver;

  AxisMpcConfig yaw_config_;
  AxisMpcConfig pitch_config_;
  double fire_threshold_{0.0035};
  int fire_offset_{2};
  std::unique_ptr<AxisSolver> yaw_solver_;
  std::unique_ptr<AxisSolver> pitch_solver_;
};

}  // namespace aim_armor_controller

#endif  // AIM_ARMOR_CONTROLLER__MPC_PLANNER_HPP_
