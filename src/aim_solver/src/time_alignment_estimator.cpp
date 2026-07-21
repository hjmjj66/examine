#include "aim_solver/time_alignment_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aim_solver
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

TimeAlignmentEstimator::TimeAlignmentEstimator()
: TimeAlignmentEstimator(Config{})
{
}

TimeAlignmentEstimator::TimeAlignmentEstimator(const Config & config)
: config_(config), offset_sec_(config.initial_offset_sec)
{
  if (config_.search_step_sec <= 0.0) {
    config_.search_step_sec = 0.002;
  }
  if (config_.max_offset_sec < config_.min_offset_sec) {
    std::swap(config_.max_offset_sec, config_.min_offset_sec);
  }
  offset_sec_ = std::clamp(
    offset_sec_, config_.min_offset_sec, config_.max_offset_sec);
  config_.window_size = std::max<std::size_t>(config_.window_size, 2U);
  config_.min_samples = std::clamp(
    config_.min_samples, std::size_t{2U}, config_.window_size);
}

void TimeAlignmentEstimator::addSample(const Sample & sample)
{
  if (!std::isfinite(sample.stamp_sec) || !std::isfinite(sample.point[0]) ||
    !std::isfinite(sample.point[1]) || !std::isfinite(sample.point[2]))
  {
    return;
  }

  samples_.push_back(sample);
  if (samples_.size() > config_.window_size) {
    samples_.erase(
      samples_.begin(), samples_.begin() +
      static_cast<std::ptrdiff_t>(samples_.size() - config_.window_size));
  }
}

void TimeAlignmentEstimator::resetSamples()
{
  samples_.clear();
  has_update_time_ = false;
  last_update_sec_ = 0.0;
  last_estimate_ = Estimate{};
  last_estimate_.offset_sec = offset_sec_;
  last_estimate_.offset_drift_sec_per_sec = offset_drift_sec_per_sec_;
  last_estimate_.reason = "target_changed_waiting_for_samples";
}

TimeAlignmentEstimator::Estimate TimeAlignmentEstimator::update(
  double now_sec,
  double motion_rad,
  const BearingEvaluator & evaluator)
{
  if (!std::isfinite(now_sec)) {
    last_estimate_.reason = "invalid_update_time";
    last_estimate_.frozen = true;
    return last_estimate_;
  }

  const double dt_sec = has_update_time_ ? std::max(0.0, now_sec - last_update_sec_) : 0.0;
  if (has_update_time_ && dt_sec < config_.update_period_sec) {
    last_estimate_.offset_sec = offset_sec_;
    last_estimate_.offset_drift_sec_per_sec = offset_drift_sec_per_sec_;
    last_estimate_.motion_rad = motion_rad;
    last_estimate_.updated = false;
    last_estimate_.frozen = true;
    last_estimate_.reason = "update_rate_limited";
    return last_estimate_;
  }
  last_estimate_.offset_sec = offset_sec_;
  last_estimate_.offset_drift_sec_per_sec = offset_drift_sec_per_sec_;
  last_estimate_.motion_rad = motion_rad;
  last_estimate_.updated = false;
  last_estimate_.frozen = true;
  last_estimate_.valid_samples = 0U;

  if (samples_.size() < config_.min_samples) {
    last_estimate_.reason = "waiting_for_samples";
    return last_estimate_;
  }
  if (motion_rad < config_.min_motion_rad) {
    last_estimate_.reason = "insufficient_gimbal_motion";
    return last_estimate_;
  }

  double best_offset = offset_sec_;
  Score best_score{std::numeric_limits<double>::infinity(), 0U};
  const auto consider = [&](double candidate) {
      const Score score = scoreAt(candidate, evaluator);
      if (score.valid_samples >= config_.min_samples && score.value < best_score.value) {
        best_score = score;
        best_offset = candidate;
      }
    };
  const double coarse_step = std::max(0.02, config_.search_step_sec * 10.0);
  const double search_min = has_measurement_ ?
    std::max(config_.min_offset_sec, offset_sec_ - config_.local_search_half_width_sec) :
    config_.min_offset_sec;
  const double search_max = has_measurement_ ?
    std::min(config_.max_offset_sec, offset_sec_ + config_.local_search_half_width_sec) :
    config_.max_offset_sec;
  for (double candidate = search_min;
    candidate <= search_max + coarse_step * 0.5;
    candidate += coarse_step)
  {
    consider(candidate);
  }
  if (std::isfinite(best_score.value)) {
    const double refine_min = std::max(
      search_min, best_offset - coarse_step);
    const double refine_max = std::min(
      search_max, best_offset + coarse_step);
    for (double candidate = refine_min;
      candidate <= refine_max + config_.search_step_sec * 0.5;
      candidate += config_.search_step_sec)
    {
      consider(candidate);
    }
  }

  const Score current_score = scoreAt(offset_sec_, evaluator);
  const Score zero_score = scoreAt(0.0, evaluator);
  last_estimate_.score_rad2 = best_score.value;
  last_estimate_.score_at_current_rad2 = current_score.value;
  last_estimate_.score_at_zero_rad2 = zero_score.value;
  last_estimate_.valid_samples = best_score.valid_samples;

  if (best_score.valid_samples < config_.min_samples ||
    !std::isfinite(best_score.value))
  {
    last_estimate_.reason = "insufficient_tf_samples";
    return last_estimate_;
  }

  const bool sufficiently_better =
    !std::isfinite(current_score.value) ||
    current_score.value - best_score.value >= config_.min_score_improvement_rad2;
  if (!sufficiently_better) {
    last_estimate_.reason = "score_not_identifiable";
    return last_estimate_;
  }

  predict(dt_sec);
  const double measurement = std::clamp(
    best_offset,
    std::max(config_.min_offset_sec, offset_sec_ - config_.max_offset_step_sec),
    std::min(config_.max_offset_sec, offset_sec_ + config_.max_offset_step_sec));
  updateKalman(measurement, config_.measurement_noise);
  last_update_sec_ = now_sec;
  has_update_time_ = true;
  has_measurement_ = true;

  last_estimate_.offset_sec = offset_sec_;
  last_estimate_.offset_drift_sec_per_sec = offset_drift_sec_per_sec_;
  last_estimate_.updated = true;
  last_estimate_.frozen = false;
  last_estimate_.converged = std::abs(current_score.value - best_score.value) >
    config_.min_score_improvement_rad2;
  last_estimate_.reason = "updated_from_static_target";
  return last_estimate_;
}

TimeAlignmentEstimator::Score TimeAlignmentEstimator::scoreAt(
  double offset_sec,
  const BearingEvaluator & evaluator) const
{
  double sum_sin = 0.0;
  double sum_cos = 0.0;
  std::vector<double> bearings;
  bearings.reserve(samples_.size());
  for (const auto & sample : samples_) {
    const auto bearing = evaluator(offset_sec, sample);
    if (!bearing.has_value() || !std::isfinite(*bearing)) {
      continue;
    }
    bearings.push_back(*bearing);
    sum_sin += std::sin(*bearing);
    sum_cos += std::cos(*bearing);
  }

  if (bearings.empty()) {
    return {};
  }
  const double mean = std::atan2(sum_sin, sum_cos);
  double error_sum = 0.0;
  for (const double bearing : bearings) {
    const double error = wrapAngle(bearing - mean);
    error_sum += error * error;
  }
  return {error_sum / static_cast<double>(bearings.size()), bearings.size()};
}

void TimeAlignmentEstimator::predict(double dt_sec)
{
  if (dt_sec <= 0.0) {
    return;
  }
  offset_sec_ += offset_drift_sec_per_sec_ * dt_sec;
  offset_sec_ = std::clamp(offset_sec_, config_.min_offset_sec, config_.max_offset_sec);

  covariance_offset_ +=
    2.0 * dt_sec * covariance_cross_ + dt_sec * dt_sec * covariance_drift_ +
    config_.process_noise_offset;
  covariance_cross_ += dt_sec * covariance_drift_;
  covariance_drift_ += config_.process_noise_drift;
}

void TimeAlignmentEstimator::updateKalman(double measurement_sec, double measurement_variance)
{
  const double innovation = measurement_sec - offset_sec_;
  const double innovation_variance =
    std::max(1e-12, covariance_offset_ + measurement_variance);
  const double gain_offset = covariance_offset_ / innovation_variance;
  const double gain_drift = covariance_cross_ / innovation_variance;
  offset_sec_ += gain_offset * innovation;
  offset_drift_sec_per_sec_ += gain_drift * innovation;
  offset_sec_ = std::clamp(offset_sec_, config_.min_offset_sec, config_.max_offset_sec);
  offset_drift_sec_per_sec_ = std::clamp(
    offset_drift_sec_per_sec_,
    config_.min_offset_drift_sec_per_sec,
    config_.max_offset_drift_sec_per_sec);

  covariance_offset_ = std::max(1e-12, (1.0 - gain_offset) * covariance_offset_);
  covariance_cross_ = (1.0 - gain_offset) * covariance_cross_;
  covariance_drift_ = std::max(1e-12, covariance_drift_ - gain_drift * covariance_cross_);
}

double TimeAlignmentEstimator::wrapAngle(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

}  // namespace aim_solver
