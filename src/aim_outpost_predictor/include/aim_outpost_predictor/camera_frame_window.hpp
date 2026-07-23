#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace aim_outpost_predictor
{

template<typename Frame>
class CameraFrameWindow
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  CameraFrameWindow(std::chrono::nanoseconds window, std::size_t capacity)
  : window_(std::max(window, std::chrono::nanoseconds::zero())),
    capacity_(std::max<std::size_t>(1U, capacity))
  {
  }

  bool enqueue(Frame frame)
  {
    bool dropped = false;
    if (pending_.size() >= capacity_) {
      pending_.pop_front();
      dropped = true;
    }
    pending_.push_back(std::move(frame));
    if (!window_deadline_.has_value()) {
      window_deadline_ = pending_.front().arrival_time + window_;
    }
    return dropped;
  }

  [[nodiscard]] bool empty() const
  {
    return pending_.empty();
  }

  [[nodiscard]] std::optional<TimePoint> nextDeadline() const
  {
    return window_deadline_;
  }

  std::optional<std::vector<Frame>> takeReady(TimePoint now)
  {
    if (pending_.empty() || !window_deadline_.has_value() || now < *window_deadline_) {
      return std::nullopt;
    }

    const TimePoint deadline = *window_deadline_;
    std::vector<Frame> batch;
    while (!pending_.empty() && pending_.front().arrival_time <= deadline) {
      batch.push_back(std::move(pending_.front()));
      pending_.pop_front();
    }

    if (pending_.empty()) {
      window_deadline_.reset();
    } else {
      window_deadline_ = pending_.front().arrival_time + window_;
    }

    std::stable_sort(
      batch.begin(), batch.end(),
      [](const Frame & lhs, const Frame & rhs) {
        if (lhs.stamp < rhs.stamp) {
          return true;
        }
        if (rhs.stamp < lhs.stamp) {
          return false;
        }
        return lhs.arrival_sequence < rhs.arrival_sequence;
      });
    return batch;
  }

private:
  std::chrono::nanoseconds window_;
  std::size_t capacity_;
  std::deque<Frame> pending_;
  std::optional<TimePoint> window_deadline_;
};

}  // namespace aim_outpost_predictor
