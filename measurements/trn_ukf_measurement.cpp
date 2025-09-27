#include "trn_ukf_measurement.h"
#include <cmath>

namespace measurements {

using State = ::aion::State;

TRNUKFMeasurement::TRNUKFMeasurement(const RadarAltimeterData& radar_data,
                                     std::shared_ptr<terrain::TerrainService> terrain_service,
                                     const Params& params)
: radar_data_(radar_data),
  terrain_service_(std::move(terrain_service)),
  params_(params) {
  // Initialize R with radar noise only; DEM term will be added on first predict()
  R_.setIdentity();
  R_(0,0) = radar_data_.noise_std * radar_data_.noise_std;
}

void TRNUKFMeasurement::nedToGeodetic(const Eigen::Vector3d& ned,
                                      double& lat_deg, double& lon_deg) const {
  lat_deg = params_.origin_lat_deg + ned.x() / mPerDegLat();
  lon_deg = params_.origin_lon_deg + ned.y() / mPerDegLon(params_.origin_lat_deg);
}

double TRNUKFMeasurement::queryElevation(double lat_deg, double lon_deg) const {
  if (!terrain_service_) return 0.0;
  double e = terrain_service_->getElevationBilinear(lat_deg, lon_deg);
  if (e == -32768.0) e = terrain_service_->getElevation(lat_deg, lon_deg);
  if (e == -32768.0) e = 0.0; // last-resort: sea level
  return e;
}

double TRNUKFMeasurement::estimateDemSigma(double lat_deg, double lon_deg) const {
  if (!terrain_service_) return params_.dem_sigma_base_m;
  const double slope_deg = terrain_service_->getTerrainSlope(lat_deg, lon_deg);
  return params_.dem_sigma_base_m + params_.dem_sigma_slope_k * std::max(0.0, slope_deg);
}

Eigen::VectorXd TRNUKFMeasurement::predict(const State& state) const {
  // Convert current NED position to geodetic
  double lat_deg, lon_deg;
  nedToGeodetic(state.position, lat_deg, lon_deg);

  // DEM elevation (MSL)
  const double dem_elev_msl = queryElevation(lat_deg, lon_deg);

  // MSL altitude of platform from NED: alt_msl = origin_alt - z_down
  const double alt_msl = params_.origin_alt_m - state.position.z();

  // Predicted AGL
  const double agl_pred = alt_msl - dem_elev_msl;

  // Adaptive variance based on terrain slope
  const double slope_deg = terrain_service_ ? terrain_service_->getTerrainSlope(lat_deg, lon_deg) : 0.0;
  const double slope_factor = std::max(0.001, slope_deg * slope_deg); // Quadratic improvement with slope
  const double sigma_dem = params_.dem_sigma_base_m / std::sqrt(0.001 + slope_factor);

  // Adaptive R: radar noise + slope-dependent DEM uncertainty
  const double r_adaptive = radar_data_.noise_std * radar_data_.noise_std + sigma_dem * sigma_dem;
  // Clamp between reasonable bounds [0.25, 64.0] (0.5m to 8m sigma)
  const_cast<TRNUKFMeasurement*>(this)->R_(0,0) = std::min(64.0, std::max(0.25, r_adaptive));

  Eigen::VectorXd h(1);
  h(0) = agl_pred;
  return h;
}

Eigen::VectorXd TRNUKFMeasurement::getValue() const {
  Eigen::VectorXd z(1);
  z(0) = radar_data_.measured_altitude; // AGL measurement
  return z;
}

Eigen::MatrixXd TRNUKFMeasurement::getCovariance() const {
  Eigen::MatrixXd R(1,1);
  R(0,0) = R_(0,0);
  return R;
}

} // namespace measurements
