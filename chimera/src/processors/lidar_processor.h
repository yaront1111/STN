#pragma once

#include "processors/i_sensor_processor.h"
#include "core/flight_computer.h"
#include "factors/altimeter_range_factor.h"

#include <deque>
#include <optional>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/NoiseModel.h>

namespace chimera {
namespace processors {

using gtsam::symbol_shorthand::X;  // Pose

// =================== Range Stuck Guard (moved from FlightComputer) ===================
class RangeStuckGuard {
public:
  RangeStuckGuard(std::size_t N, double thresh_var, double thresh_spread = 0.005)
      : N_(N), thresh_var_(thresh_var), thresh_spread_(thresh_spread) {}

  void add(double z){ win.push_back(z); if(win.size()>N_) win.pop_front(); }

  bool isStuck() const {
    if(win.size() < N_) return false;

    // Gate 1: Low variance
    double m=0.0; for(double x: win) m+=x; m/=win.size();
    double v=0.0; for(double x: win){ double d=x-m; v+=d*d; } v/=win.size();
    bool low_variance = (v < thresh_var_);

    // Gate 2: Low range spread (max - min)
    double min_val = *std::min_element(win.begin(), win.end());
    double max_val = *std::max_element(win.begin(), win.end());
    double range_spread = max_val - min_val;
    bool low_spread = (range_spread < thresh_spread_);

    // Dual-gate: Both conditions must be true
    return (low_variance && low_spread);
  }

  bool isFull() const { return win.size() >= N_; }

  void reset(){ win.clear(); }

  std::size_t N;  // Made public for flight_computer.h access

private:
  std::size_t N_;
  double thresh_var_;
  double thresh_spread_;
  std::deque<double> win;
};

// =================== LiDAR Processor ===================
class LidarProcessor : public ISensorProcessor {
public:
  struct Config {
    double lidar_min = 0.1;
    double lidar_max = 120.0;
    double sigma_validated = 0.05;
    double sigma_coldstart = 0.2;
    double stuck_var_thresh = 0.0001;  // Updated: 0.01m stddev threshold
    double stuck_range_spread = 0.005;  // Added: 5mm range spread threshold
    std::size_t stuck_window_size = 8;
    double match_window_s = 0.15;

    // Soft outlier guard
    bool use_soft_guard = true;
    double soft_guard_threshold_m = 50.0;
    int soft_guard_keyframe_limit = 30;
    double soft_guard_sigma_fallback = 5.0;
  };

  LidarProcessor(
    const std::vector<chimera::core::LidarSample>& samples,
    const Config& cfg,
    chimera::core::ContractMonitor& monitor,
    int& update_counter
  ) : samples_(samples),
      cfg_(cfg),
      monitor_(monitor),
      update_counter_(update_counter),
      stuck_guard_(cfg.stuck_window_size, cfg.stuck_var_thresh) {}

  SensorProcessingResult process(
    const SensorProcessingContext& ctx,
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& init
  ) override {
    SensorProcessingResult result;

    // Find nearest LiDAR sample
    auto lidar = nearestByTime(samples_, ctx.time_absolute, cfg_.match_window_s);
    if(!lidar){
      return result;  // No sample available
    }

    result.sensor_available = true;
    result.measured_altitude = lidar->range_mean;

    // Range validity check
    if(lidar->range_mean < cfg_.lidar_min || lidar->range_mean > cfg_.lidar_max){
      return result;  // Out of valid range
    }

    // Stuck detection
    stuck_guard_.add(lidar->range_mean);
    if(stuck_guard_.isStuck()){
      stuck_detected_ = true;
      return result;  // LiDAR marked as stuck
    }

    if(stuck_detected_){
      return result;  // Already stuck, don't use
    }

    result.sensor_healthy = true;

    // Determine sigma (cold-start vs validated)
    double sigma = ctx.boot_done ? cfg_.sigma_validated : cfg_.sigma_coldstart;

    // Soft outlier guard (prevent catastrophic steps during early convergence)
    if(cfg_.use_soft_guard && ctx.keyframe <= cfg_.soft_guard_keyframe_limit){
      const double predicted_z = std::abs(ctx.ukf_pose.z());
      const double dz = std::abs(predicted_z - lidar->range_mean);
      if(dz > cfg_.soft_guard_threshold_m){
        sigma = cfg_.soft_guard_sigma_fallback;
      }
    }

    // Add altimeter range factor
    auto noise = gtsam::noiseModel::Isotropic::Sigma(1, sigma);

    // Use Tukey robustifier for better outlier rejection
    auto tukey = gtsam::noiseModel::mEstimator::Tukey::Create(4.6851);
    auto robust = gtsam::noiseModel::Robust::Create(tukey, noise);

    graph.emplace_shared<chimera::factors::AltimeterRangeFactor>(
      ctx.key_pose, lidar->range_mean, robust);

    result.factor_added = true;
    result.altitude_constrained = true;
    result.sigma = sigma;

    // Update monitoring
    monitor_.updateAltitude(ctx.time_relative);
    update_counter_++;

    return result;
  }

  const char* name() const override { return "LiDAR"; }

  bool isAvailable(double time_absolute) const override {
    auto sample = nearestByTime(samples_, time_absolute, cfg_.match_window_s);
    return sample != nullptr;
  }

  bool isStuck() const { return stuck_detected_; }
  void resetStuckGuard() { stuck_guard_.reset(); stuck_detected_ = false; }

private:
  const std::vector<chimera::core::LidarSample>& samples_;
  Config cfg_;
  chimera::core::ContractMonitor& monitor_;
  int& update_counter_;
  RangeStuckGuard stuck_guard_;
  bool stuck_detected_ = false;

  // Helper: Find nearest sample by time
  template<typename T>
  static const T* nearestByTime(const std::vector<T>& v, double t, double tol){
    if(v.empty()) return nullptr;
    std::size_t best=0;
    double d= std::abs(v[0].t - t);
    for(std::size_t i=1; i<v.size(); ++i){
      double di= std::abs(v[i].t - t);
      if(di<d){ d=di; best=i; }
    }
    return (d<=tol)? &v[best] : nullptr;
  }
};

}  // namespace processors
}  // namespace chimera
