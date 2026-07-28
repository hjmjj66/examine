#include "aim_armor_controller/mpc_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "tinympc/tiny_api.hpp"

namespace aim_armor_controller
{

namespace
{

void destroyTinySolver(TinySolver * solver)
{
  if (solver == nullptr) {
    return;
  }
  delete solver->solution;
  delete solver->cache;
  delete solver->settings;
  delete solver->work;
  delete solver;
}

void validateAxisConfig(const AxisMpcConfig & config, const char * axis_name)
{
  if (config.dt <= 0.0) {
    throw std::runtime_error(std::string(axis_name) + " MPC dt must be positive");
  }
  if (config.horizon < 3) {
    throw std::runtime_error(std::string(axis_name) + " MPC horizon must be at least 3");
  }
  if (config.max_acceleration <= 0.0) {
    throw std::runtime_error(std::string(axis_name) + " max_acceleration must be positive");
  }
}

}  // namespace

struct MpcPlanner::AxisSolver
{
  AxisSolver(const AxisMpcConfig & config, const char * axis_name)
  : config(config)
  {
    validateAxisConfig(config, axis_name);

    const Eigen::MatrixXd A{{1.0, config.dt}, {0.0, 1.0}};
    const Eigen::MatrixXd B{{0.0}, {config.dt}};
    const Eigen::VectorXd f{{0.0, 0.0}};
    const Eigen::Matrix2d Q = Eigen::Vector2d(
      config.q_position, config.q_velocity).asDiagonal();
    Eigen::Matrix<double, 1, 1> R;
    R(0, 0) = config.r_acceleration;

    TinySolver * raw_solver = nullptr;
    const int knot_count = config.horizon + 1;
    const int setup_status = tiny_setup(
      &raw_solver, A, B, f, Q, R, 1.0, 2, 1, knot_count, 0);
    solver.reset(raw_solver);
    if (setup_status != 0 || !solver) {
      throw std::runtime_error(std::string(axis_name) + " TinyMPC setup failed");
    }

    const Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, knot_count, -1e17);
    const Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, knot_count, 1e17);
    const Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(
      1, config.horizon, -config.max_acceleration);
    const Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(
      1, config.horizon, config.max_acceleration);
    const int bound_status = tiny_set_bound_constraints(
      solver.get(), x_min, x_max, u_min, u_max);
    if (bound_status != 0) {
      throw std::runtime_error(std::string(axis_name) + " TinyMPC bound setup failed");
    }

    solver->settings->max_iter = config.max_iterations;
  }

  AxisMpcConfig config;
  std::unique_ptr<TinySolver, void (*)(TinySolver *)> solver{nullptr, destroyTinySolver};
};

MpcPlanner::MpcPlanner(const MpcPlannerConfig & config)
: yaw_config_(config.yaw),
  pitch_config_(config.pitch),
  fire_threshold_(config.fire_threshold),
  fire_offset_(config.fire_offset),
  yaw_solver_(std::make_unique<AxisSolver>(config.yaw, "yaw")),
  pitch_solver_(std::make_unique<AxisSolver>(config.pitch, "pitch"))
{
  if (yaw_config_.horizon != pitch_config_.horizon) {
    throw std::runtime_error("yaw and pitch MPC horizons must match for fire control");
  }
  if (fire_threshold_ <= 0.0) {
    throw std::runtime_error("MPC fire_threshold must be positive");
  }
  const int fire_index = yaw_config_.horizon / 2 + fire_offset_;
  if (fire_offset_ < 0 || fire_index > yaw_config_.horizon) {
    throw std::runtime_error("MPC fire_offset puts fire control index outside the horizon");
  }
}

MpcPlanner::~MpcPlanner() = default;

MpcPlan MpcPlanner::solve(const MpcReference & reference)
{
  if (reference.yaw.rows() != 2 || reference.yaw.cols() != yaw_config_.horizon + 1 ||
    reference.pitch.rows() != 2 || reference.pitch.cols() != pitch_config_.horizon + 1)
  {
    throw std::runtime_error("MPC reference shape does not match configured horizon");
  }

  Eigen::VectorXd x0(2);
  x0 << reference.yaw(0, 0), reference.yaw(1, 0);
  tiny_set_x0(yaw_solver_->solver.get(), x0);
  yaw_solver_->solver->work->Xref = reference.yaw;
  tiny_solve(yaw_solver_->solver.get());

  x0 << reference.pitch(0, 0), reference.pitch(1, 0);
  tiny_set_x0(pitch_solver_->solver.get(), x0);
  pitch_solver_->solver->work->Xref = reference.pitch;
  tiny_solve(pitch_solver_->solver.get());

  const int axis_index = yaw_config_.horizon / 2;
  const int fire_index = axis_index + fire_offset_;

  MpcPlan plan;
  plan.valid = true;
  plan.fire_error = std::hypot(
    reference.yaw(0, fire_index) - yaw_solver_->solver->work->x(0, fire_index),
    reference.pitch(0, fire_index) - pitch_solver_->solver->work->x(0, fire_index));
  plan.fire = plan.fire_error < fire_threshold_;
  plan.yaw = yaw_solver_->solver->work->x(0, axis_index);
  plan.yaw_velocity = yaw_solver_->solver->work->x(1, axis_index);
  plan.yaw_acceleration = yaw_solver_->solver->work->u(0, axis_index);
  plan.pitch = pitch_solver_->solver->work->x(0, axis_index);
  plan.pitch_velocity = pitch_solver_->solver->work->x(1, axis_index);
  plan.pitch_acceleration = pitch_solver_->solver->work->u(0, axis_index);
  return plan;
}

}  // namespace aim_armor_controller
