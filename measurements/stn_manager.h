#pragma once

#include <memory>
#include <vector>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "stn_observation.h"
#include "../core/fusion/state.h"

namespace measurements {

class STNManager {
public:
  struct Params {
    std::string beacons_file;
    STNConfig config;

    // Local origin for NED conversion
    double origin_lat_deg;
    double origin_lon_deg;
    double origin_alt_msl;

    Params()
      : beacons_file("configs/stn/beacons.yaml"),
        origin_lat_deg(31.7683),
        origin_lon_deg(35.2137),
        origin_alt_msl(750.0) {}
  };

  explicit STNManager(const Params& params = Params());

  // Load beacons from YAML file
  bool loadBeacons();

  // Convert geodetic to local NED
  Eigen::Vector3d geodeticToNED(double lat_deg, double lon_deg, double alt_msl) const;

  // Compute GDOP for TOA (includes clock column)
  GDOPResult computeGDOP_TOA(const Eigen::Vector3d& position_ned,
                            const std::vector<std::string>& beacon_ids,
                            const std::vector<double>* sigmas = nullptr) const;

  // Compute GDOP for TDOA (no clock column)
  GDOPResult computeGDOP_TDOA(const Eigen::Vector3d& position_ned,
                              const std::vector<std::string>& beacon_ids,
                              const std::string& ref_beacon_id,
                              const std::vector<double>* sigmas = nullptr) const;

  // Legacy wrapper (defaults to TOA)
  GDOPResult computeGDOP(const Eigen::Vector3d& position_ned,
                        const std::vector<std::string>& beacon_ids) const {
    return computeGDOP_TOA(position_ned, beacon_ids);
  }

  // Select optimal beacon subset based on geometry
  std::vector<std::string> selectBeacons(const Eigen::Vector3d& position_ned,
                                        const std::vector<STNObservation>& observations) const;

  // Process new observations
  void processObservations(const std::vector<STNObservation>& observations,
                          double current_time);

  // RAIM: Detect and remove outliers
  std::vector<std::string> performRAIM(const Eigen::Vector3d& position_ned,
                                       const std::vector<STNObservation>& observations,
                                       const Eigen::Vector2d& clock_state) const;

  // Get beacon by ID
  const STNBeacon* getBeacon(const std::string& id) const;

  // Get all active beacons
  std::vector<STNBeacon> getActiveBeacons() const;

  // Compute TOA prediction (with clock bias)
  double predictTOA(const Eigen::Vector3d& position_ned,
                   const std::string& beacon_id,
                   double clock_bias_m) const;

  // Compute TDOA prediction (clock cancels)
  double predictTDOA(const Eigen::Vector3d& position_ned,
                    const std::string& beacon_id,
                    const std::string& ref_beacon_id) const;

  // Compute Doppler prediction (with clock drift)
  double predictDoppler(const Eigen::Vector3d& position_ned,
                       const Eigen::Vector3d& velocity_ned,
                       const std::string& beacon_id,
                       double clock_drift_frac) const;

  // Compute FDOA prediction (drift cancels)
  double predictFDOA(const Eigen::Vector3d& position_ned,
                    const Eigen::Vector3d& velocity_ned,
                    const std::string& beacon_id,
                    const std::string& ref_beacon_id) const;

  // Accessor for params
  const Params& params() const { return params_; }

  // Helper functions (needed by STNUKFMeasurement)
  double computeRange(const Eigen::Vector3d& position_ned,
                     const STNBeacon& beacon) const;

  Eigen::Vector3d computeUnitVector(const Eigen::Vector3d& position_ned,
                                   const STNBeacon& beacon) const;

private:
  Params params_;
  std::unordered_map<std::string, STNBeacon> beacons_;
  std::vector<STNObservation> observation_buffer_;

  // Constants for geodetic conversion
  static constexpr double WGS84_A = 6378137.0;  // Semi-major axis
  static constexpr double WGS84_E2 = 0.00669437999014; // First eccentricity squared

  // ECEF conversion helpers
  static void geodeticToECEF(double lat_rad, double lon_rad, double h,
                            Eigen::Vector3d& ecef);
  static Eigen::Matrix3d ecefToEnuRot(double lat_rad, double lon_rad);

  // Earth radius at latitude
  double earthRadius(double lat_deg) const;

  // Meters per degree at latitude
  double metersPerDegreeLat() const;
  double metersPerDegreeLon(double lat_deg) const;
};

} // namespace measurements