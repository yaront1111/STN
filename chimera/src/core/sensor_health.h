// CHIMERA Sensor Health Monitoring System
// Per-sensor health scoring for robust mode switching and fault tolerance
// Implements specifications from docs/commercialization/ROBUSTNESS.md

#pragma once

#include <deque>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <Eigen/Dense>

#include "utils/data_loaders.h"

namespace chimera {
namespace core {

// =================== Rolling Window Helper ===================
template<typename T>
class RollingWindow {
 public:
  explicit RollingWindow(size_t max_size) : max_size_(max_size) {}

  void push(T value) {
    window_.push_back(value);
    if (window_.size() > max_size_) {
      window_.pop_front();
    }
  }

  bool isFull() const { return window_.size() >= max_size_; }
  size_t size() const { return window_.size(); }

  double mean() const {
    if (window_.empty()) return 0.0;
    double sum = std::accumulate(window_.begin(), window_.end(), 0.0);
    return sum / window_.size();
  }

  double stddev() const {
    if (window_.size() < 2) return 0.0;
    double m = mean();
    double variance = 0.0;
    for (const auto& v : window_) {
      double diff = v - m;
      variance += diff * diff;
    }
    variance /= window_.size();
    return std::sqrt(variance);
  }

  const std::deque<T>& data() const { return window_; }

 private:
  size_t max_size_;
  std::deque<T> window_;
};

// =================== Base Health Structure ===================
struct SensorHealthBase {
  double score = 1.0;      // Health score [0.0, 1.0] - 1.0 = perfect
  std::string reason;      // Human-readable degradation reason

  // Helper: check if sensor is healthy (above threshold)
  bool isHealthy(double threshold = 0.5) const { return score >= threshold; }
};

// =================== LiDAR Health Monitor ===================
class LidarHealthMonitor {
 public:
  struct Health : public SensorHealthBase {
    bool is_stuck = false;
    bool out_of_range = false;
    double noise_level = 0.0;
  };

  struct Config {
    double min_range = 0.3;    // meters
    double max_range = 50.0;   // meters
    double stuck_eps = 0.01;   // stddev threshold for stuck detection (10mm)
    double stuck_range_spread = 0.005; // max-min threshold for stuck (5mm)
    double noise_thresh = 0.5; // meters - high noise threshold
    size_t stuck_window = 8;   // samples for stuck detection
  };

  LidarHealthMonitor() : cfg_(), plateau_guard_(cfg_.stuck_window) {}
  explicit LidarHealthMonitor(const Config& cfg)
      : cfg_(cfg), plateau_guard_(cfg.stuck_window) {}

  Health evaluate(const chimera::utils::LidarSample& sample) {
    Health h;
    h.score = 1.0;

    // Check range validity
    if (sample.range_mean < cfg_.min_range || sample.range_mean > cfg_.max_range) {
      h.out_of_range = true;
      h.score *= 0.1;
      h.reason += "out_of_range ";
    }

    // Stuck detection (dual-gate: variance AND range spread)
    plateau_guard_.push(sample.range_mean);
    if (plateau_guard_.isFull()) {
      const auto& window = plateau_guard_.data();

      // Gate 1: Low variance (stddev < 10mm)
      double variance = computeVariance(window);
      double variance_thresh = cfg_.stuck_eps * cfg_.stuck_eps;
      bool low_variance = (variance < variance_thresh);

      // Gate 2: Low range spread (max - min < 5mm)
      auto minmax = std::minmax_element(window.begin(), window.end());
      double range_spread = *minmax.second - *minmax.first;
      bool low_spread = (range_spread < cfg_.stuck_range_spread);

      // Stuck if BOTH gates trigger (dual-gate logic)
      if (low_variance && low_spread) {
        h.is_stuck = true;
        h.score = 0.0;
        h.reason += "stuck_plateau ";
      }
    }

    // Noise level (using abs(range_mean - range_min) as proxy)
    h.noise_level = std::abs(sample.range_mean - sample.range_min);
    if (h.noise_level > cfg_.noise_thresh) {
      h.score *= 0.5;
      h.reason += "high_noise ";
    }

    return h;
  }

  void reset() { plateau_guard_ = RollingWindow<double>(cfg_.stuck_window); }

 private:
  Config cfg_;
  RollingWindow<double> plateau_guard_;

  double computeVariance(const std::deque<double>& v) const {
    if (v.size() < 2) return 0.0;
    double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    double var = 0.0;
    for (double x : v) {
      double d = x - mean;
      var += d * d;
    }
    return var / v.size();
  }
};

// =================== Baro Health Monitor ===================
class BaroHealthMonitor {
 public:
  struct Health : public SensorHealthBase {
    double noise_level = 0.0;
    double drift_rate = 0.0;  // m/s
    bool rapid_change = false;
  };

  struct Config {
    size_t noise_window = 20;         // samples (1 sec @ 20 Hz)
    double noise_thresh = 0.5;        // meters stddev
    double rapid_change_thresh = 5.0; // m/s
    size_t max_history = 100;
  };

  BaroHealthMonitor() : cfg_(), noise_window_(cfg_.noise_window) {}
  explicit BaroHealthMonitor(const Config& cfg)
      : cfg_(cfg), noise_window_(cfg.noise_window) {}

  Health evaluate(const chimera::utils::BaroSample& sample) {
    Health h;
    h.score = 1.0;

    // Update noise estimator
    noise_window_.push(sample.altitude);
    h.noise_level = noise_window_.stddev();

    if (h.noise_level > cfg_.noise_thresh) {
      h.score *= 0.7;
      h.reason += "high_noise ";
    }

    // Detect rapid changes (pressure step)
    if (!history_.empty()) {
      double dz = sample.altitude - history_.back().altitude;
      double dt = sample.t - history_.back().t;
      if (dt > 1e-6) {  // Avoid division by zero
        double rate = std::abs(dz / dt);
        if (rate > cfg_.rapid_change_thresh) {
          h.rapid_change = true;
          h.score *= 0.3;
          h.reason += "rapid_change ";
        }
      }
    }

    // Track drift rate (simple finite difference for now)
    h.drift_rate = estimateDriftRate();

    history_.push_back(sample);
    if (history_.size() > cfg_.max_history) {
      history_.pop_front();
    }

    return h;
  }

  void reset() {
    noise_window_ = RollingWindow<double>(cfg_.noise_window);
    history_.clear();
  }

 private:
  Config cfg_;
  RollingWindow<double> noise_window_;
  std::deque<chimera::utils::BaroSample> history_;

  double estimateDriftRate() const {
    if (history_.size() < 10) return 0.0;
    // Simple linear regression slope over last N samples
    size_t n = std::min(history_.size(), size_t(20));
    double sum_t = 0.0, sum_z = 0.0, sum_tz = 0.0, sum_tt = 0.0;
    auto it = history_.rbegin();
    for (size_t i = 0; i < n; ++i, ++it) {
      double t = it->t;
      double z = it->altitude;
      sum_t += t;
      sum_z += z;
      sum_tz += t * z;
      sum_tt += t * t;
    }
    double denominator = n * sum_tt - sum_t * sum_t;
    if (std::abs(denominator) < 1e-9) return 0.0;
    return (n * sum_tz - sum_t * sum_z) / denominator;
  }
};

// =================== Mag Health Monitor ===================
class MagHealthMonitor {
 public:
  struct Health : public SensorHealthBase {
    double norm = 1.0;
    double norm_deviation = 0.0;
    bool interference = false;
  };

  struct Config {
    double median_norm = 1.0;         // Expected norm from boot
    double gate_frac = 0.30;          // ±30% gating
    double spike_thresh = 0.2;        // Norm change threshold
    size_t max_history = 50;
  };

  MagHealthMonitor() : cfg_() {}
  explicit MagHealthMonitor(const Config& cfg) : cfg_(cfg) {}

  Health evaluate(const chimera::utils::MagSample& sample) {
    Health h;
    h.score = 1.0;
    h.norm = sample.mag.norm();

    // Norm gating
    h.norm_deviation = std::abs(h.norm - cfg_.median_norm);
    double gate_abs = cfg_.median_norm * cfg_.gate_frac;

    if (h.norm_deviation > gate_abs) {
      h.interference = true;
      h.score = 0.0;
      h.reason += "norm_gate ";
    }

    // Spike detection
    if (!history_.empty()) {
      double prev_norm = history_.back().second;
      double dnorm = std::abs(h.norm - prev_norm);
      if (dnorm > cfg_.spike_thresh) {
        h.interference = true;
        h.score *= 0.2;
        h.reason += "spike ";
      }
    }

    history_.push_back({sample.t, h.norm});
    if (history_.size() > cfg_.max_history) {
      history_.pop_front();
    }

    return h;
  }

  void reset() { history_.clear(); }
  void setMedianNorm(double norm) { cfg_.median_norm = norm; }

 private:
  Config cfg_;
  std::deque<std::pair<double, double>> history_;  // (timestamp, norm)
};

// =================== Optical Flow Health Monitor ===================
class OpticalFlowHealthMonitor {
 public:
  struct Health : public SensorHealthBase {
    double texture_quality = 1.0;
    bool saturated = false;
    bool low_texture = false;
    uint32_t rejected_rays = 0;
  };

  struct Config {
    double texture_thresh = 50.0;     // Minimum texture variance
    double saturation_thresh = 200.0; // px/s
  };

  OpticalFlowHealthMonitor() : cfg_() {}
  explicit OpticalFlowHealthMonitor(const Config& cfg) : cfg_(cfg) {}

  Health evaluate(const chimera::utils::OpticalFlowSample& sample) {
    Health h;
    h.score = 1.0;

    // Compute average texture quality across all rays
    double avg_texture = 0.0;
    int valid_rays = 0;
    for (const auto& ray : sample.rays) {
      avg_texture += ray.texture_var;
      valid_rays++;
    }
    h.texture_quality = valid_rays > 0 ? avg_texture / valid_rays : 0.0;

    // Texture gating
    if (h.texture_quality < cfg_.texture_thresh) {
      h.low_texture = true;
      h.score *= (h.texture_quality / cfg_.texture_thresh);  // Proportional penalty
      h.reason += "low_texture ";
    }

    // Saturation check (check any ray)
    for (const auto& ray : sample.rays) {
      double flow_mag = std::sqrt(ray.flow_u * ray.flow_u + ray.flow_v * ray.flow_v);
      if (flow_mag > cfg_.saturation_thresh) {
        h.saturated = true;
        h.score *= 0.1;
        h.reason += "saturated ";
        h.rejected_rays++;
      }
    }

    return h;
  }

  void reset() {}

 private:
  Config cfg_;
};

// =================== IMU Health Monitor ===================
class ImuHealthMonitor {
 public:
  struct Health : public SensorHealthBase {
    bool gyro_saturated = false;
    bool accel_saturated = false;
    double temperature = 25.0;
    bool temp_out_of_range = false;
  };

  struct Config {
    double gyro_sat_thresh = 8.0;     // rad/s (~460 deg/s)
    double accel_sat_thresh = 150.0;  // m/s^2 (~15g)
    double temp_min = -10.0;          // Celsius
    double temp_max = 60.0;           // Celsius
  };

  explicit ImuHealthMonitor() : cfg_() {}
  explicit ImuHealthMonitor(const Config& cfg) : cfg_(cfg) {}

  Health evaluate(const chimera::utils::ImuSample& sample) {
    Health h;
    h.score = 1.0;
    // Note: ImuSample doesn't have temperature field, use default 25.0C
    h.temperature = 25.0;

    // Saturation checks
    if (sample.gyro.cwiseAbs().maxCoeff() > cfg_.gyro_sat_thresh) {
      h.gyro_saturated = true;
      h.score = 0.0;
      h.reason += "gyro_sat ";
    }

    if (sample.accel.cwiseAbs().maxCoeff() > cfg_.accel_sat_thresh) {
      h.accel_saturated = true;
      h.score = 0.0;
      h.reason += "accel_sat ";
    }

    // Temperature range check (currently always pass since temp=25.0C)
    if (h.temperature < cfg_.temp_min || h.temperature > cfg_.temp_max) {
      h.temp_out_of_range = true;
      h.score *= 0.8;
      h.reason += "temp_range ";
    }

    return h;
  }

  void reset() {}

 private:
  Config cfg_;
};

// =================== Altitude Source Selection ===================
enum class AltitudeSource {
  NONE,
  LIDAR,
  BARO,
  BARO_TAKEOVER
};

inline const char* toString(AltitudeSource src) {
  switch (src) {
    case AltitudeSource::NONE: return "none";
    case AltitudeSource::LIDAR: return "lidar";
    case AltitudeSource::BARO: return "baro";
    case AltitudeSource::BARO_TAKEOVER: return "baro_takeover";
    default: return "unknown";
  }
}

class AltitudeSourceSelector {
 public:
  struct Config {
    double takeover_thresh_m = 50.0;  // Divergence threshold for takeover
  };

  AltitudeSourceSelector() : cfg_() {}
  explicit AltitudeSourceSelector(const Config& cfg) : cfg_(cfg) {}

  AltitudeSource select(
      const LidarHealthMonitor::Health& lidar_h,
      const BaroHealthMonitor::Health& baro_h,
      double z_pred,      // Current predicted Z (NED frame, down=+Z)
      double z_baro) {    // Baro altitude (NED frame)

    // Priority: LiDAR > Baro > Takeover

    // LiDAR preferred if healthy
    if (lidar_h.score > 0.5 && !lidar_h.is_stuck) {
      return AltitudeSource::LIDAR;
    }

    // Check for divergence (trigger takeover)
    if (std::abs(z_pred - z_baro) > cfg_.takeover_thresh_m) {
      return AltitudeSource::BARO_TAKEOVER;
    }

    // Fall back to baro if healthy
    if (baro_h.score > 0.3) {
      return AltitudeSource::BARO;
    }

    // No good altitude source!
    return AltitudeSource::NONE;
  }

 private:
  Config cfg_;
};

// =================== Mode Transition Tracking ===================
struct ModeTransition {
  double timestamp = 0.0;
  AltitudeSource from = AltitudeSource::NONE;
  AltitudeSource to = AltitudeSource::NONE;
  std::string reason;
};

}  // namespace core
}  // namespace chimera
