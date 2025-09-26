#pragma once

#include <memory>
#include <Eigen/Dense>

#include "../core/fusion/sr_ukf.h"
#include "../core/fusion/state.h"
#include "../terrain/terrain_service.h"

namespace measurements {

using State = ::aion::State;

// Raw radar altimeter data (AGL)
struct RadarAltimeterData {
  double timestamp{0.0};
  double measured_altitude{0.0}; // AGL [m]
  double noise_std{1.0};         // radar sigma [m]
  bool   valid{true};
};

// TRN Measurement adapter for UKF (1D AGL)
class TRNUKFMeasurement : public aion::MeasurementBase {
public:
  struct Params {
    double origin_lat_deg;
    double origin_lon_deg;
    double origin_alt_m;       // MSL altitude of NED origin
    double dem_sigma_base_m;   // DEM base uncertainty
    double dem_sigma_slope_k;  // +k * slope_deg

    Params() : origin_lat_deg(32.0853),
               origin_lon_deg(34.7818),
               origin_alt_m(20.0),
               dem_sigma_base_m(5.0),
               dem_sigma_slope_k(0.1) {}
  };

  TRNUKFMeasurement(const RadarAltimeterData& radar_data,
                    std::shared_ptr<terrain::TerrainService> terrain_service,
                    const Params& params = Params());

  ~TRNUKFMeasurement() override = default;

  // MeasurementBase API
  Eigen::VectorXd predict(const State& state) const override;   // h(x) = AGL_pred
  Eigen::VectorXd getValue() const override;                    // z = AGL_meas
  Eigen::MatrixXd getCovariance() const override;               // R (1x1)
  int    getDimension() const override { return 1; }
  double getTime() const override { return radar_data_.timestamp; }

private:
  static inline double mPerDegLat() { return 111111.0; }
  static inline double mPerDegLon(double lat_deg) {
    return 111111.0 * std::cos(lat_deg * M_PI / 180.0);
  }

  // NED → geodetic (deg) using fixed origin
  void nedToGeodetic(const Eigen::Vector3d& ned, double& lat_deg, double& lon_deg) const;

  // DEM helpers
  double queryElevation(double lat_deg, double lon_deg) const;     // meters MSL
  double estimateDemSigma(double lat_deg, double lon_deg) const;   // meters (std)

private:
  RadarAltimeterData radar_data_;
  std::shared_ptr<terrain::TerrainService> terrain_service_;
  Params params_;

  // cached covariance (1x1)
  Eigen::Matrix<double,1,1> R_;
};

} // namespace measurements
