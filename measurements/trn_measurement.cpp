#include "trn_measurement.h"
#include "core/utils/logging.h"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>

namespace measurements {

using std::unique_ptr;
using std::optional;

void TRNProcessor::nedToGeodetic(const Eigen::Vector3d& ned_pos,
                                 double& lat_deg,
                                 double& lon_deg) const {
  const double dN = ned_pos.x(); // north
  const double dE = ned_pos.y(); // east
  lat_deg = cfg_.origin_lat_deg + dN / metersPerDegLat();
  lon_deg = cfg_.origin_lon_deg + dE / metersPerDegLon(cfg_.origin_lat_deg);
}

optional<double> TRNProcessor::queryTerrainElevation(double lat_deg, double lon_deg) const {
  if (!terrain_) return 0.0; // treat as sea level if service is absent

  double elev = terrain_->getElevationBilinear(lat_deg, lon_deg);
  if (elev == -32768.0) {
    elev = terrain_->getElevation(lat_deg, lon_deg);
    if (elev == -32768.0) return std::nullopt;
  }
  return elev;
}

std::unique_ptr<aion::MeasurementBase>
TRNProcessor::makeMeasurement(const RadarAltimeterData& radar,
                              const State& state) {
  // basic checks
  if (!radar.valid) return nullptr;
  if (radar.measured_altitude < cfg_.min_agl_m ||
      radar.measured_altitude > cfg_.max_agl_m) {
    return nullptr;
  }

  // lat/lon from NED
  double lat_deg, lon_deg;
  nedToGeodetic(state.position, lat_deg, lon_deg);

  // DEM query
  auto elev_opt = queryTerrainElevation(lat_deg, lon_deg);
  if (!elev_opt.has_value()) {
    spdlog::info("TRN: DEM void at ({:.6f},{:.6f}) — skip", lat_deg, lon_deg);
    return nullptr;
  }
  const double terrain_elev_m = *elev_opt;

  // Compute terrain slope using finite differences
  double slope_tangent = 0.0;
  if (terrain_) {
    const double dlat = 0.0001;  // ~11m
    const double dlon = 0.0001;  // ~9m at this latitude

    auto elev_north = queryTerrainElevation(lat_deg + dlat, lon_deg);
    auto elev_south = queryTerrainElevation(lat_deg - dlat, lon_deg);
    auto elev_east = queryTerrainElevation(lat_deg, lon_deg + dlon);
    auto elev_west = queryTerrainElevation(lat_deg, lon_deg - dlon);

    if (elev_north && elev_south && elev_east && elev_west) {
      const double dx_meters = 2.0 * dlon * metersPerDegLon(lat_deg);
      const double dy_meters = 2.0 * dlat * metersPerDegLat();

      double dz_dx = (*elev_east - *elev_west) / dx_meters;
      double dz_dy = (*elev_north - *elev_south) / dy_meters;
      slope_tangent = std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy);
    }
  }

  // Check slope threshold
  if (slope_tangent < min_slope_tangent_) {
    double slope_deg = std::atan(slope_tangent) * 180.0 / M_PI;
    spdlog::info("TRN: slope too flat: {:.2f}° < {:.2f}° at ({:.5f},{:.5f})",
                slope_deg, cfg_.min_slope_deg, lat_deg, lon_deg);
    return nullptr;
  }

  const double slope_deg = std::atan(slope_tangent) * 180.0 / M_PI;

  // Compose measurement noise (radar ⊕ DEM)
  const double radar_sigma = std::max(1e-3, radar.noise_std);
  const double dem_sigma   = cfg_.dem_sigma_base_m + cfg_.dem_sigma_slope_coeff_mpd * slope_deg;

  // Build a temporary measurement to do a quick chi^2 gate (1-dof)
  TRNAltMeasurement tmp(radar.timestamp,
                        radar.measured_altitude,
                        terrain_elev_m,
                        cfg_.origin_alt_m,
                        radar_sigma,
                        dem_sigma);

  const Eigen::VectorXd z = tmp.getValue();         // [AGL_measured]
  const Eigen::VectorXd h = tmp.predict(state);     // [AGL_predicted]
  const Eigen::MatrixXd R = tmp.getCovariance();    // [1x1]

  const double innov   = z(0) - h(0);
  const double S       = R(0,0); // pre-gate with R only (UKF will add state part)
  const double chi2    = (innov*innov) / std::max(1e-12, S);

  spdlog::info("TRN: lat={:.5f} lon={:.5f} terr={:.1f}m slope={:.2f}° AGL_meas={:.2f} AGL_pred={:.2f} innov={:.2f} chi2={:.2f}",
              lat_deg, lon_deg, terrain_elev_m, slope_deg, z(0), h(0), innov, chi2);

  if (chi2 > cfg_.chi2_gate_1d) { // chi2 threshold already in chi-square units
    spdlog::info("TRN: chi2 gate fail: chi2={:.2f} > {:.2f}, innov={:.2f} m", chi2, cfg_.chi2_gate_1d, innov);
    return nullptr;
  }

  // Accept: hand an owning MeasurementBase to the UKF
  return std::make_unique<TRNAltMeasurement>(radar.timestamp,
                                             radar.measured_altitude,
                                             terrain_elev_m,
                                             cfg_.origin_alt_m,
                                             radar_sigma,
                                             dem_sigma);
}

} // namespace measurements
