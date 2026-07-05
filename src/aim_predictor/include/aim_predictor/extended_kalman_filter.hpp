#pragma once

#include <Eigen/Dense>

#include <cstddef>
#include <deque>
#include <functional>

namespace aim_predictor
{

class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter() = default;
  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0,
    const Eigen::MatrixXd & p0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add);

  Eigen::VectorXd predict(
    const Eigen::MatrixXd & f,
    const Eigen::MatrixXd & q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> transition);

  Eigen::VectorXd update(
    const Eigen::VectorXd & z,
    const Eigen::MatrixXd & h,
    const Eigen::MatrixXd & r,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> observation,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract);

  Eigen::VectorXd x;
  Eigen::MatrixXd p;
  std::deque<int> recent_nis_failures{0};
  std::size_t window_size{100};

private:
  Eigen::MatrixXd identity_;
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add_;
};

}  // namespace aim_predictor
