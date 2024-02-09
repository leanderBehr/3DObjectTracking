#pragma once

#include <chrono>
#include <optional>

namespace m3t {

using steady_time_point = std::chrono::steady_clock::time_point;

class Clock {
 public:
  explicit Clock(steady_time_point start_time, steady_time_point end_time)
      : clock_min_(start_time),
        clock_max_(end_time),
        clock_started_(clock_min_) {}

  steady_time_point current_time() {
    using namespace std::literals;

    auto real_now = std::chrono::steady_clock::now();

    if (!real_started_) return clock_started_;

    auto real_passed = real_started_ ? real_now - *real_started_ : 0ns;
    auto clock_now = clock_started_ + real_passed;

    if (clock_now >= clock_max_) {
      clock_started_ = clock_min_;
      real_started_ = real_now;
      return clock_min_;
    }

    return clock_now;
  }

  void start(steady_time_point time = std::chrono::steady_clock::now()) {
    real_started_ = time;
  }

  void stop() { stop_at(current_time()); }

  void stop_at(steady_time_point time) {
    clock_started_ = time;
    real_started_ = std::nullopt;
  }

  void set_to(steady_time_point time) {
    clock_started_ = time;
    if (real_started_) real_started_ = std::chrono::steady_clock::now();
  }

  steady_time_point get_clock_min() const { return clock_min_; }

 private:
  std::optional<steady_time_point> real_started_{};

  steady_time_point clock_min_{};
  steady_time_point clock_max_{};
  steady_time_point clock_started_{};
};
}  // namespace m3t