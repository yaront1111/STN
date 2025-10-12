#pragma once

#include <Eigen/Dense>
#include <memory>
#include <optional>

#include "core/fusion/sr_ukf.h"          // aion::MeasurementBase, IMU types
#include "core/fusion/state.h"           // aion::State
#include "core/utils/config.h"           // for SystemConfig origin if desired
#include "terrain/terrain_service.h"     // terrain queries

namespace measurements {

using State = ::aion::State;

// Radar altimeter measurement data
struct RadarAltimeterData {
  double timestamp{0.0};          // Time in seconds
  double measured_altitude{0.0};  // AGL (m)
  double noise_std{1.0};          // Radar noise sigma (m)
  bool   valid{false};            // validity flag
};

// TRN configuration (roughly matches your YAML plan)
struct TRNConfig {
  // Gating
  double min_slope_deg              = 0.5;    // reject flat terrain
  double chi2_gate_1d               = 6.63;   // ~99% for 1 dof (NOT sigma)
  double min_agl_m                  = 2.0;    // usable AGL range
  double max_agl_m                  = 1500.0;

  // DEM & sensor noise (std devs, meters)
  double radar_noise_m              = 1.0;    // base radar noise
  double dem_sigma_base_m           = 5.0;    // DEM base sigma
  double dem_sigma_slope_coeff_mpd  = 0.1;    // extra sigma per deg slope

  // Local frame origin (NED)
  double origin_lat_deg             = 32.0853;
  double origin_lon_deg             = 34.7818;
  double origin_alt_m               = 20.0;   // MSL of NED origin (meters)
};

// One concrete UKF measurement: radar AGL vs terrain
class TRNAltMeasurement final : public aion::MeasurementBase {
public:
  TRNAltMeasurement(double t_sec,
                    double measured_agl_m,
                    double terrain_elev_m,
                    double origin_alt_m,
                    double radar_sigma_m,
                    double dem_sigma_m)
  : t_(t_sec),
    z_agl_(measured_agl_m),
    terrain_elev_m_(terrain_elev_m),
    origin_alt_m_(origin_alt_m),
    r_var_((radar_sigma_m*radar_sigma_m) + (dem_sigma_m*dem_sigma_m)) {}

  // Predict radar AGL:  h(x) = (origin_alt - z_down) - terrain_elev
  Eigen::VectorXd predict(const State& state) const override {
    Eigen::VectorXd h(1);
    const double msl_alt_m = origin_alt_m_ - state.position.z(); // NED: +down
    h(0) = msl_alt_m - terrain_elev_m_;
    return h;
  }

  Eigen::VectorXd getValue() const override {
    Eigen::VectorXd z(1);
    z(0) = z_agl_;
    return z;
  }

  Eigen::MatrixXd getCovariance() const override {
    Eigen::MatrixXd R(1,1);
    R(0,0) = r_var_;
    return R;
  }

  int getDimension() const override { return 1; }
  double getTime() const override { return t_; }

private:
  double t_;
  double z_agl_;
  double terrain_elev_m_;
  double origin_alt_m_;
  double r_var_;
};

// Processor: turns raw radar pings into UKF measurements with gating
class TRNProcessor {
public:
  TRNProcessor(std::shared_ptr<terrain::TerrainService> terrain,
               const TRNConfig& cfg)
  : terrain_(std::move(terrain)), cfg_(cfg),
    min_slope_tangent_(std::tan(cfg.min_slope_deg * M_PI / 180.0)) {}

  // Returns a ready-to-use measurement or null if rejected by gates.
  std::unique_ptr<aion::MeasurementBase>
  makeMeasurement(const RadarAltimeterData& radar,
                  const State& state);

private:
  // Helpers
  static inline double metersPerDegLat() { return 111132.92; }
  static inline double metersPerDegLon(double lat_deg) {
    return 111412.84 * std::cos(lat_deg * M_PI / 180.0);
  }

  // Convert local NED to lat/lon using fixed-scale approximation at origin
  void nedToGeodetic(const Eigen::Vector3d& ned_pos,
                     double& lat_deg,
                     double& lon_deg) const;

  // Get terrain elevation (bilinear, fallback to nearest sample). Returns std::nullopt on full void.
  std::optional<double> queryTerrainElevation(double lat_deg, double lon_deg) const;

private:
  std::shared_ptr<terrain::TerrainService> terrain_;
  TRNConfig cfg_;
  const double min_slope_tangent_;
};

} // namespace measurements
