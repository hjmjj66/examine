#include "aim_predictor/extended_kalman_filter.hpp"

#include <utility>

namespace aim_predictor
{

ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0,
  const Eigen::MatrixXd & p0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), p(p0), identity_(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add_(std::move(x_add))
{
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & f,
  const Eigen::MatrixXd & q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> transition)
{
  p = f * p * f.transpose() + q;
  x = transition(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z,
  const Eigen::MatrixXd & h,
  const Eigen::MatrixXd & r,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> observation,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  const Eigen::MatrixXd innovation_cov = h * p * h.transpose() + r;
  const Eigen::MatrixXd k = p * h.transpose() * innovation_cov.inverse();
  p = (identity_ - k * h) * p * (identity_ - k * h).transpose() + k * r * k.transpose();

  const Eigen::VectorXd innovation = z_subtract(z, observation(x));
  x = x_add_(x, k * innovation);

  const double nis = innovation.transpose() * innovation_cov.inverse() * innovation;
  recent_nis_failures.push_back(nis > 9.4877 ? 1 : 0);
  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }
  return x;
}

}  // namespace aim_predictor
