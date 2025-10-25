#pragma once

#include "processors/i_sensor_processor.h"
#include "core/flight_computer.h"
#include "factors/baro_factor.h"

#include <deque>
#include <optional>
#include <algorithm>
#include <cmath>

namespace chimera {
namespace processors {

// =================== Robust Baro Filter ===================
class RobustBaroFilter {
public:
  static double qmad(const std::vector<double>& v){
    if(v.empty()) return 0.0;
    std::vector<double> d=v;
    std::nth_element(d.begin(), d.begin()+d.size()/2, d.end());
    double med = d[d.size()/2];
    std::vector<double> r; r.reserve(d.size());
    for(double x: d) r.push_back(std::abs(x-med));
    std::nth_element(r.begin(), r.begin()+r.size()/2, r.end());
    return 1.4826 * r[r.size()/2];
  }

  void add(double t, double msl){ buf.emplace_back(t, msl); }

  bool computeAGL(double t_now, double baro0_msl, double boot_z0_agl,
                  double& agl_out, double& sigma_out){
    const double WINDOW=2.0;
    while(!buf.empty() && (t_now - buf.front().first) > WINDOW)
      buf.pop_front();

    if(buf.size() < 3 || !std::isfinite(baro0_msl)) return false;

    std::vector<double> msl; msl.reserve(buf.size());
    for(auto& kv: buf) msl.push_back(kv.second);

    std::vector<double> d=msl;
    std::nth_element(d.begin(), d.begin()+d.size()/2, d.end());
    double med = d[d.size()/2];
    double sig = qmad(msl);
    sigma_out = std::clamp(sig*1.5, 0.8, 2.5);  // Increased for noisy baro

    // Use MEDIAN msl, not latest (more robust to noise)
    agl_out = (med - baro0_msl) + boot_z0_agl;
    return (agl_out > -20.0 && agl_out < 500.0);
  }

private:
  std::deque<std::pair<double, double>> buf;
};

// =================== Baro Processor ===================
class BaroProcessor : public ISensorProcessor {
public:
  struct Config {
    double match_window_s = 0.20;
    double min_agl = -20.0;
    double max_agl = 500.0;
  };

  BaroProcessor(
    const std::vector<chimera::core::BaroSample>& samples,
    const Config& cfg,
    chimera::core::ContractMonitor& monitor,
    int& update_counter,
    double& baro0_msl,
    double boot_z0_agl
  ) : samples_(samples),
      cfg_(cfg),
      monitor_(monitor),
      update_counter_(update_counter),
      baro0_msl_(baro0_msl),
      boot_z0_agl_(boot_z0_agl) {}

  SensorProcessingResult process(
    const SensorProcessingContext& ctx,
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& init
  ) override {
    SensorProcessingResult result;

    // Find nearest baro sample
    auto baro = nearestByTime(samples_, ctx.time_absolute, cfg_.match_window_s);
    if(!baro){
      return result;  // No sample available
    }

    result.sensor_available = true;

    // Add to filter buffer
    filter_.add(ctx.time_absolute, baro->altitude);

    // Compute robust AGL
    double agl, sigma;
    if(!filter_.computeAGL(ctx.time_absolute, baro0_msl_, boot_z0_agl_, agl, sigma)){
      return result;  // Not enough data or invalid
    }

    result.sensor_healthy = true;
    result.measured_altitude = agl;
    result.sigma = sigma;

    // Add baro factor
    auto noise = gtsam::noiseModel::Isotropic::Sigma(1, sigma);
    auto huber = gtsam::noiseModel::mEstimator::Huber::Create(1.5);
    auto robust = gtsam::noiseModel::Robust::Create(huber, noise);

    graph.emplace_shared<chimera::factors::BaroFactor>(ctx.key_pose, agl, robust);

    result.factor_added = true;
    result.altitude_constrained = true;

    // Update monitoring
    monitor_.updateAltitude(ctx.time_relative);
    update_counter_++;

    return result;
  }

  const char* name() const override { return "Baro"; }

  bool isAvailable(double time_absolute) const override {
    auto sample = nearestByTime(samples_, time_absolute, cfg_.match_window_s);
    return sample != nullptr;
  }

private:
  const std::vector<chimera::core::BaroSample>& samples_;
  Config cfg_;
  chimera::core::ContractMonitor& monitor_;
  int& update_counter_;
  double& baro0_msl_;
  double boot_z0_agl_;
  RobustBaroFilter filter_;

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
