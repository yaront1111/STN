#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>

namespace aion {

/**
 * Time utilities for AION
 * - Wall clock (system time, UNIX epoch) for logging & timestamps
 * - Monotonic clock (steady) for scheduling & latency
 */
class TimeUtils {
 public:
  using MonoClock  = std::chrono::steady_clock;  // monotonic, not wall epoch
  using WallClock  = std::chrono::system_clock;  // UNIX epoch
  using MonoPoint  = MonoClock::time_point;
  using WallPoint  = WallClock::time_point;
  using DurationD  = std::chrono::duration<double>;

  // ---------- Initialization ----------
  // Call once at startup (idempotent). Records base wall/mono anchors.
  static void initialize();

  // ---------- Wall clock (UNIX epoch seconds) ----------
  static double now();                  // alias for wallNow()
  static double wallNow();              // seconds since UNIX epoch
  static WallPoint nowWallTimePoint();  // WallClock::now()
  static double toSeconds(const WallPoint& tp);     // UNIX epoch seconds
  static WallPoint fromSeconds(double unix_seconds); // UNIX epoch → WallPoint

  // ---------- Monotonic clock (for scheduling/latency) ----------
  static double getMonotonicTime();          // seconds since initialize()
  static MonoPoint nowMonoTimePoint();       // MonoClock::now()
  static double toMonotonicSeconds(const MonoPoint& tp); // seconds since initialize()

  // ---------- Helpers ----------
  // Check that a wall-clock timestamp is not in the future and not older than max_age_s.
  static bool isValidTimestamp(double unix_timestamp, double max_age_s = 1.0);

  // Compute latency between measurement and receive times (both wall-clock seconds).
  static double computeLatency(double t_meas_unix, double t_recv_unix);

  // Sleep with good precision using steady clock (combines sleep_until + short spin).
  static void preciseSleep(double seconds);

  // Access system start anchors
  static MonoPoint getSystemStartMono();
  static WallPoint getSystemStartWall();

 private:
  static std::atomic<bool> initialized_;
  static std::mutex init_mutex_;

  static MonoPoint mono_start_;
  static WallPoint wall_start_;
};

/**
 * Timestamp for sensor IO (wall-clock domain).
 * - t_meas: when the sensor captured the sample (UNIX seconds)
 * - t_recv: when we received it (UNIX seconds)
 */
struct Timestamp {
  double t_meas{0.0};
  double t_recv{0.0};

  Timestamp() = default;
  Timestamp(double meas, double recv) : t_meas(meas), t_recv(recv) {}

  // Create with current receive time (wall clock).
  static Timestamp createNow(double meas_time_unix);

  // Positive means delay between capture and receive.
  double latency() const { return t_recv - t_meas; }

  // Validate basic sanity and max latency bound (seconds).
  bool isValid(double max_latency = 0.1) const;
};

/**
 * RateController — keep a loop at ~rate_hz using steady clock.
 */
class RateController {
 public:
  explicit RateController(double rate_hz);

  // Sleep until the next cycle boundary (updates stats).
  void sleep();

  // Reset phase and stats.
  void reset();

  // Actual achieved rate (Hz), averaged.
  double getActualRate() const;

  // Did we meet target within tolerance? (e.g., 0.9 = 90%)
  bool isHealthy(double tolerance = 0.9) const;

  struct Stats {
    double target_rate_hz{0.0};
    double actual_rate_hz{0.0};
    double min_dt{0.0};
    double max_dt{0.0};
    double mean_dt{0.0};
    uint64_t cycles{0};
    uint64_t overruns{0};
  };

  Stats getStats() const;

 private:
  double rate_hz_{0.0};
  double period_s_{0.0};

  TimeUtils::MonoPoint next_time_{};
  TimeUtils::MonoPoint start_time_{};
  TimeUtils::MonoPoint last_time_{};

  // Stats
  mutable std::mutex stats_mutex_;
  uint64_t cycles_{0};
  uint64_t overruns_{0};
  double sum_dt_{0.0};
  double min_dt_{std::numeric_limits<double>::max()};
  double max_dt_{0.0};
};

}  // namespace aion
