#pragma once
#include <chrono>

namespace chimera {
namespace utils {

/**
 * @brief Get current time in seconds (steady_clock for monotonic time)
 * @return Seconds since epoch (arbitrary but monotonic)
 */
inline double nowSec() {
  using Clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

/**
 * @brief Simple timer for measuring elapsed time
 */
class Timer {
 public:
  Timer() : t0_(0.0) {}

  /** Start the timer */
  void tic() { t0_ = nowSec(); }

  /** Get elapsed time in seconds since last tic() */
  double toc() const { return nowSec() - t0_; }

  /** Get elapsed time in milliseconds since last tic() */
  double tocMs() const { return toc() * 1000.0; }

 private:
  double t0_;
};

}  // namespace utils
}  // namespace chimera
