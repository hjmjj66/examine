#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace aim_solver
{

class TimeAlignmentEstimator
{
public:
  struct Config
  {
    double initial_offset_sec{0.0};
    double min_offset_sec{-0.6};
    double max_offset_sec{0.2};
    double search_step_sec{0.002};
    std::size_t window_size{24U};
    std::size_t min_samples{8U};
    double min_motion_rad{0.03};
    double min_score_improvement_rad2{1e-5};
    double update_period_sec{0.10};
    double process_noise_offset{1e-5};
    double process_noise_drift{1e-5};
    double measurement_noise{2.5e-5};
    double max_offset_step_sec{0.03};
    double local_search_half_width_sec{0.12};
    double min_offset_drift_sec_per_sec{-0.05};
    double max_offset_drift_sec_per_sec{0.05};
  };

  struct Sample
  {
    double stamp_sec{0.0};
    std::array<double, 3> point{};
  };

  struct Estimate
  {
    double offset_sec{0.0};
    double offset_drift_sec_per_sec{0.0};
    double score_rad2{0.0};
    double score_at_zero_rad2{0.0};
    double score_at_current_rad2{0.0};
    double motion_rad{0.0};
    std::size_t valid_samples{0U};
    bool converged{false};
    bool updated{false};
    bool frozen{true};
    std::string reason{"waiting_for_samples"};
  };

  using BearingEvaluator = std::function<std::optional<double>(
        double offset_sec, const Sample & sample)>;

  TimeAlignmentEstimator();
  explicit TimeAlignmentEstimator(const Config & config);

  void addSample(const Sample & sample);

  // Clears target-specific samples while preserving the learned global offset.
  void resetSamples();

  [[nodiscard]] Estimate update(
    double now_sec,
    double motion_rad,
    const BearingEvaluator & evaluator);

  [[nodiscard]] double offsetSec() const
  {
    return offset_sec_;
  }

  [[nodiscard]] Estimate status() const
  {
    return last_estimate_;
  }

  [[nodiscard]] const std::vector<Sample> & samples() const
  {
    return samples_;
  }

private:
  struct Score
  {
    double value{0.0};
    std::size_t valid_samples{0U};
  };

  [[nodiscard]] Score scoreAt(
    double offset_sec,
    const BearingEvaluator & evaluator) const;
  void predict(double dt_sec);
  void updateKalman(double measurement_sec, double measurement_variance);
  static double wrapAngle(double angle_rad);

  Config config_;
  std::vector<Sample> samples_;
  double offset_sec_{0.0};
  double offset_drift_sec_per_sec_{0.0};
  double covariance_offset_{0.04};
  double covariance_drift_{0.01};
  double covariance_cross_{0.0};
  double last_update_sec_{0.0};
  bool has_update_time_{false};
  bool has_measurement_{false};
  Estimate last_estimate_;
};

}  // namespace aim_solver
