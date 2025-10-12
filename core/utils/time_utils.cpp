#include "core/utils/time_utils.h"
#include "core/utils/logging.h"

#include <algorithm>
#include <cmath>
#include <thread>

namespace aion {

// --------- Static storage ---------
std::atomic<bool> TimeUtils::initialized_{false};
std::mutex        TimeUtils::init_mutex_;
TimeUtils::MonoPoint TimeUtils::mono_start_{};
TimeUtils::WallPoint TimeUtils::wall_start_{};

// --------- TimeUtils impl ---------
void TimeUtils::initialize() {
  bool expected = false;
  if (initialized_.compare_exchange_strong(expected, true)) {
    std::lock_guard<std::mutex> lock(init_mutex_);
    mono_start_ = MonoClock::now();
    wall_start_ = WallClock::now();
    LOG_INFO("TimeUtils initialized (anchors set)");
  }
}

double TimeUtils::now() { return wallNow(); }

double TimeUtils::wallNow() {
  auto tp = nowWallTimePoint();
  return toSeconds(tp);
}

TimeUtils::WallPoint TimeUtils::nowWallTimePoint() {
  return WallClock::now();
}

double TimeUtils::toSeconds(const WallPoint& tp) {
  const auto since_epoch = tp.time_since_epoch();
  return std::chrono::duration_cast<DurationD>(since_epoch).count();
}

TimeUtils::WallPoint TimeUtils::fromSeconds(double unix_seconds) {
  const auto dur = DurationD(unix_seconds);
  return WallClock::time_point(std::chrono::duration_cast<WallClock::duration>(dur));
}

double TimeUtils::getMonotonicTime() {
  if (!initialized_) initialize();
  return std::chrono::duration_cast<DurationD>(MonoClock::now() - mono_start_).count();
}

TimeUtils::MonoPoint TimeUtils::nowMonoTimePoint() {
  return MonoClock::now();
}

double TimeUtils::toMonotonicSeconds(const MonoPoint& tp) {
  if (!initialized_) initialize();
  return std::chrono::duration_cast<DurationD>(tp - mono_start_).count();
}

bool TimeUtils::isValidTimestamp(double unix_timestamp, double max_age_s) {
  const double current = wallNow();

  if (unix_timestamp > current + 0.001) {
    LOG_WARN("Timestamp {:.6f} is in the future (now {:.6f})", unix_timestamp, current);
    return false;
  }

  const double age = current - unix_timestamp;
  if (age > max_age_s) {
    LOG_WARN("Timestamp {:.6f} too old: age {:.3f}s > max {:.3f}s", unix_timestamp, age, max_age_s);
    return false;
  }
  return true;
}

double TimeUtils::computeLatency(double t_meas_unix, double t_recv_unix) {
  return t_recv_unix - t_meas_unix;
}

void TimeUtils::preciseSleep(double seconds) {
  if (seconds <= 0.0) return;

  // Use steady clock for deadline.
  auto target = MonoClock::now() + DurationD(seconds);
  constexpr double spin_threshold = 0.001; // 1 ms

  for (;;) {
    auto now = MonoClock::now();
    if (now >= target) break;

    double remaining = std::chrono::duration_cast<DurationD>(target - now).count();
    if (remaining > spin_threshold) {
      // Sleep most of it; leave ~1ms margin
      auto sleep_for = std::chrono::duration_cast<MonoClock::duration>(DurationD(remaining - spin_threshold));
      std::this_thread::sleep_for(sleep_for);
    } else {
      // Short spin/yield
      std::this_thread::yield();
    }
  }
}

TimeUtils::MonoPoint TimeUtils::getSystemStartMono() {
  if (!initialized_) initialize();
  return mono_start_;
}

TimeUtils::WallPoint TimeUtils::getSystemStartWall() {
  if (!initialized_) initialize();
  return wall_start_;
}

// --------- Timestamp ---------
Timestamp Timestamp::createNow(double meas_time_unix) {
  return Timestamp(meas_time_unix, TimeUtils::wallNow());
}

bool Timestamp::isValid(double max_latency) const {
  if (!(t_meas > 0.0 && t_recv > 0.0)) return false;

  const double lat = latency();
  if (lat < 0.0) {
    LOG_WARN("Negative latency: meas={:.6f}, recv={:.6f}", t_meas, t_recv);
    return false;
  }
  if (lat > max_latency) {
    LOG_WARN("Excessive latency: {:.3f}s > {:.3f}s", lat, max_latency);
    return false;
  }

  return TimeUtils::isValidTimestamp(t_meas) && TimeUtils::isValidTimestamp(t_recv);
}

// --------- RateController ---------
RateController::RateController(double rate_hz) : rate_hz_(rate_hz) {
  period_s_ = (rate_hz_ > 0.0) ? (1.0 / rate_hz_) : 0.0;
  reset();
}

void RateController::reset() {
  TimeUtils::initialize();
  next_time_  = TimeUtils::nowMonoTimePoint();
  start_time_ = next_time_;
  last_time_  = next_time_;

  std::lock_guard<std::mutex> lock(stats_mutex_);
  cycles_   = 0;
  overruns_ = 0;
  sum_dt_   = 0.0;
  min_dt_   = std::numeric_limits<double>::max();
  max_dt_   = 0.0;
}

void RateController::sleep() {
  auto now = TimeUtils::nowMonoTimePoint();

  // Stats: dt since previous cycle boundary (approx)
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    if (cycles_ > 0) {
      const double dt = std::chrono::duration_cast<TimeUtils::DurationD>(now - last_time_).count();
      sum_dt_ += dt;
      if (dt < min_dt_) min_dt_ = dt;
      if (dt > max_dt_) max_dt_ = dt;
    }
    last_time_ = now;
    ++cycles_;
  }

  // Schedule next wake-up
  next_time_ += std::chrono::duration_cast<TimeUtils::MonoClock::duration>(TimeUtils::DurationD(period_s_));

  // Overrun handling
  if (now > next_time_) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++overruns_;
    const double lag = std::chrono::duration_cast<TimeUtils::DurationD>(now - next_time_).count();
    // If severely behind, re-align to now to avoid drift explosion
    if (lag > 10.0 * period_s_) {
      LOG_WARN("RateController severely behind by {:.3f}s; realigning", lag);
      next_time_ = now;
    }
    return;
  }

  // Sleep precisely until next cycle boundary
  std::this_thread::sleep_until(next_time_);
}

double RateController::getActualRate() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  if (cycles_ <= 1 || sum_dt_ <= 0.0) return 0.0;
  // Average period from collected deltas
  const double mean_dt = sum_dt_ / static_cast<double>(cycles_ - 1);
  return (mean_dt > 0.0) ? (1.0 / mean_dt) : 0.0;
}

bool RateController::isHealthy(double tolerance) const {
  const double actual = getActualRate();
  return actual >= (rate_hz_ * tolerance);
}

RateController::Stats RateController::getStats() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  Stats s;
  s.target_rate_hz = rate_hz_;
  s.cycles   = cycles_;
  s.overruns = overruns_;
  if (cycles_ > 1) {
    s.mean_dt = sum_dt_ / static_cast<double>(cycles_ - 1);
    s.min_dt  = (min_dt_ == std::numeric_limits<double>::max()) ? 0.0 : min_dt_;
    s.max_dt  = max_dt_;
    s.actual_rate_hz = (s.mean_dt > 0.0) ? (1.0 / s.mean_dt) : 0.0;
  }
  return s;
}

} // namespace aion
