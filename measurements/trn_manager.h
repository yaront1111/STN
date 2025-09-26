#pragma once

#include <memory>
#include <mutex>
#include <queue>
#include <random>

#include "core/fusion/state.h"
#include "core/utils/time_utils.h"
#include "terrain/terrain_service.h"
#include "trn_measurement.h"   // TRNAltMeasurement + TRNProcessor

namespace measurements {

using State = ::aion::State;

// TRN manager configuration (chi2 threshold, not “sigma”)
struct TRNManagerConfig {
  bool   enabled            = true;
  double update_rate_hz     = 10.0;       // how often to accept radar pings
  double chi2_gate_1d       = 6.63;       // ≈99% for 1 dof
  double min_slope_deg      = 0.5;        // info-bearing terrain
  double min_agl_m          = 2.0;
  double max_agl_m          = 1500.0;

  // DEM uncertainty model
  double dem_sigma_base_m   = 5.0;
  double dem_sigma_slope_k  = 0.1;        // +k * slope_deg

  // Radar noise (std dev)
  double radar_sigma_m      = 1.0;

  // NED origin (used for AGL / geodetic)
  double origin_lat_deg     = 32.0853;
  double origin_lon_deg     = 34.7818;
  double origin_alt_m       = 20.0;       // MSL altitude of NED origin
  std::string dem_path      = "data/dem/srtm";
};

class TRNManager {
public:
  explicit TRNManager(const TRNManagerConfig& cfg);

  bool initialize();  // load terrain, optional preload

  // Feed a radar ping; if it passes gates, a measurement is queued
  bool processRadarPing(double timestamp_sec, double measured_agl_m, bool valid=true);
  bool processRadarPing(double timestamp_sec, double measured_agl_m, const State& state, bool valid=true);

  // UKF integration: pull next measurement (MeasurementBase) if available
  bool hasMeasurement() const;
  std::unique_ptr<aion::MeasurementBase> nextMeasurement();

  // Simple simulator for testing (returns a noisy AGL)
  double simulateAGLFromState(const State& true_state);

  // Accessors
  std::shared_ptr<terrain::TerrainService> terrain() { return terrain_service_; }
  const TRNManagerConfig& config() const { return cfg_; }

  struct Stats {
    uint64_t pings_in{0};
    uint64_t accepted{0};
    uint64_t gated{0};
    double   last_accept_time{0.0};
    double   avg_rate_hz{0.0};
  };
  const Stats& stats() const { return stats_; }
  void resetStats();

private:
  // Build a measurement (or nullptr) from a ping + current nav state
  std::unique_ptr<aion::MeasurementBase>
  buildMeasurement(double t, double measured_agl_m, const State& state);

  // NED→lat/lon
  static inline double mPerDegLat() { return 111111.0; }
  static inline double mPerDegLon(double lat_deg) { return 111111.0 * std::cos(lat_deg * M_PI / 180.0); }
  void nedToGeodetic(const Eigen::Vector3d& ned, double& lat_deg, double& lon_deg) const;

private:
  TRNManagerConfig cfg_;
  std::shared_ptr<terrain::TerrainService> terrain_service_;
  TRNConfig trn_cfg_;              // forwarded to TRNProcessor
  TRNProcessor trn_proc_;          // does gating & measurement construction

  // queue of ready-to-use UKF measurements
  mutable std::mutex q_mtx_;
  std::queue<std::unique_ptr<aion::MeasurementBase>> q_;

  // rate limiting
  double last_ping_time_{0.0};

  // stats
  Stats stats_;

  // simulator noise
  std::mt19937 rng_;
  std::normal_distribution<double> radar_noise_;
};

} // namespace measurements
