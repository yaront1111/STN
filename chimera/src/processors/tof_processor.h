#pragma once

#include "processors/i_range_altimeter_processor.h"
#include "core/flight_computer.h"
#include "factors/altimeter_range_factor.h"

#include <deque>
#include <cmath>
#include <algorithm>

namespace chimera {
namespace processors {

using gtsam::symbol_shorthand::X;  // Pose

// =================== ToF Sample (extends LidarSample for compatibility) ===================
struct TofSample {
  double t;
  double range_min;
  double range_mean;
  double range_max;
  int num_points;  // For multi-zone ToF, use 1 for single-zone
  double confidence;  // 0.0-1.0, sensor-specific quality metric
  int status;  // Sensor-specific status code
};

// =================== ToF Stuck Guard (adapted for ToF characteristics) ===================
class TofStuckGuard {
public:
  TofStuckGuard(std::size_t N, double thresh_var) : N_(N), thresh_var_(thresh_var) {}

  void add(double z, double conf){
    win.push_back({z, conf});
    if(win.size() > N_) win.pop_front();
  }

  bool isStuck() const {
    if(win.size() < N_) return false;

    // Compute variance of range measurements
    double m=0.0;
    for(const auto& s: win) m += s.range;
    m /= win.size();

    double v=0.0;
    for(const auto& s: win){
      double d = s.range - m;
      v += d*d;
    }
    v /= win.size();

    // Also check confidence - if all samples have low confidence, might be stuck
    double avg_conf = 0.0;
    for(const auto& s: win) avg_conf += s.confidence;
    avg_conf /= win.size();

    return (v < thresh_var_) || (avg_conf < 0.3);
  }

  bool isFull() const { return win.size() >= N_; }
  void reset(){ win.clear(); }

private:
  struct Sample { double range; double confidence; };
  std::size_t N_;
  double thresh_var_;
  std::deque<Sample> win;
};

// =================== ToF Processor ===================
class TofProcessor : public IRangeAltimeterProcessor {
public:
  struct Config {
    double tof_min = 0.05;   // ToF minimum range (shorter than LiDAR)
    double tof_max = 4.0;    // ToF maximum range (shorter than LiDAR)
    double sigma_base = 0.10;  // Base sigma (noisier than LiDAR)
    double sigma_scale_factor = 0.02;  // Scale sigma with range (sigma = base + scale*range)
    double confidence_threshold = 0.5;  // Minimum confidence to use measurement
    double stuck_var_thresh = 0.001;  // Higher than LiDAR (more noise tolerance)
    std::size_t stuck_window_size = 8;
    double match_window_s = 0.10;

    // Soft outlier guard
    bool use_soft_guard = true;
    double soft_guard_threshold_m = 20.0;  // Tighter than LiDAR (shorter range)
    int soft_guard_keyframe_limit = 30;
    double soft_guard_sigma_fallback = 5.0;
  };

  TofProcessor(
    const std::vector<TofSample>& samples,
    const Config& cfg,
    chimera::core::ContractMonitor& monitor,
    int& update_counter
  ) : samples_(samples),
      cfg_(cfg),
      monitor_(monitor),
      update_counter_(update_counter),
      stuck_guard_(cfg.stuck_window_size, cfg.stuck_var_thresh) {}

  RangeAltimeterResult processRange(
    const SensorProcessingContext& ctx,
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& init
  ) override {
    RangeAltimeterResult result;

    // Find nearest ToF sample
    auto tof = nearestByTime(samples_, ctx.time_absolute, cfg_.match_window_s);
    if(!tof){
      return result;  // No sample available
    }

    result.sensor_available = true;
    result.range_mean = tof->range_mean;
    result.range_min = tof->range_min;
    result.range_max = tof->range_max;
    result.num_points = tof->num_points;
    result.measured_altitude = tof->range_mean;

    // Confidence check
    if(tof->confidence < cfg_.confidence_threshold){
      confidence_ = tof->confidence;
      return result;  // Low confidence
    }

    // Range validity check
    if(tof->range_mean < cfg_.tof_min || tof->range_mean > cfg_.tof_max){
      result.in_valid_range = false;
      return result;  // Out of valid range
    }

    result.in_valid_range = true;

    // Stuck detection
    stuck_guard_.add(tof->range_mean, tof->confidence);
    if(stuck_guard_.isStuck()){
      stuck_detected_ = true;
      result.stuck_detected = true;
      return result;  // ToF marked as stuck
    }

    if(stuck_detected_){
      result.stuck_detected = true;
      return result;  // Already stuck, don't use
    }

    result.sensor_healthy = true;
    confidence_ = tof->confidence;

    // Adaptive sigma based on range and confidence
    double sigma = cfg_.sigma_base + cfg_.sigma_scale_factor * tof->range_mean;
    sigma /= std::sqrt(tof->confidence);  // Lower confidence → higher sigma

    // Soft outlier guard
    if(cfg_.use_soft_guard && ctx.keyframe <= cfg_.soft_guard_keyframe_limit){
      const double predicted_z = std::abs(ctx.ukf_pose.z());
      const double dz = std::abs(predicted_z - tof->range_mean);
      if(dz > cfg_.soft_guard_threshold_m){
        sigma = cfg_.soft_guard_sigma_fallback;
        result.soft_guard_triggered = true;
      }
    }

    // Add altimeter range factor
    auto noise = gtsam::noiseModel::Isotropic::Sigma(1, sigma);

    // Use Tukey robustifier (better for outliers)
    auto tukey = gtsam::noiseModel::mEstimator::Tukey::Create(4.6851);
    auto robust = gtsam::noiseModel::Robust::Create(tukey, noise);

    graph.emplace_shared<chimera::factors::AltimeterRangeFactor>(
      ctx.key_pose, tof->range_mean, robust);

    result.factor_added = true;
    result.altitude_constrained = true;
    result.sigma = sigma;

    // Update monitoring
    monitor_.updateAltitude(ctx.time_relative);
    update_counter_++;

    return result;
  }

  const char* name() const override { return "ToF"; }

  bool isAvailable(double time_absolute) const override {
    auto sample = nearestByTime(samples_, time_absolute, cfg_.match_window_s);
    return sample != nullptr;
  }

  // IRangeAltimeterProcessor interface
  double getMinRange() const override { return cfg_.tof_min; }
  double getMaxRange() const override { return cfg_.tof_max; }
  double getTypicalSigma() const override { return cfg_.sigma_base; }

  bool isStuck() const override { return stuck_detected_; }
  void resetStuckGuard() override { stuck_guard_.reset(); stuck_detected_ = false; }

  double getConfidence() const override { return confidence_; }

private:
  const std::vector<TofSample>& samples_;
  Config cfg_;
  chimera::core::ContractMonitor& monitor_;
  int& update_counter_;
  TofStuckGuard stuck_guard_;
  bool stuck_detected_ = false;
  double confidence_ = 1.0;

  // Helper: Find nearest sample by time
  static const TofSample* nearestByTime(const std::vector<TofSample>& v, double t, double tol){
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
