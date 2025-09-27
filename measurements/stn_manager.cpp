#include "stn_manager.h"
#include "../core/utils/logging.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <fstream>

namespace measurements {

STNManager::STNManager(const Params& params) : params_(params) {
  loadBeacons();
}

bool STNManager::loadBeacons() {
  try {
    std::ifstream file(params_.beacons_file);
    if (!file.is_open()) {
      return false;
    }

    YAML::Node root = YAML::LoadFile(params_.beacons_file);

    // Load configuration
    if (root["config"]) {
      auto cfg = root["config"];
      params_.config.min_beacons = cfg["min_beacons"].as<int>(4);
      params_.config.max_beacons = cfg["max_beacons"].as<int>(8);
      params_.config.gdop_threshold = cfg["gdop_threshold"].as<double>(6.0);
      params_.config.snr_threshold_db = cfg["snr_threshold_db"].as<double>(10.0);
      params_.config.raim_enabled = cfg["raim_enabled"].as<bool>(true);
      params_.config.raim_max_outliers = cfg["raim_max_outliers"].as<int>(2);
      params_.config.update_rate_hz = cfg["update_rate_hz"].as<double>(2.0);
    }

    // Load clock parameters
    if (root["clock"]) {
      auto clk = root["clock"];
      params_.config.use_ocxo = clk["use_ocxo"].as<bool>(true);
      params_.config.clock_drift_sigma_ppm = clk["drift_sigma_ppm"].as<double>(0.01);
      params_.config.clock_bias_sigma_m = clk["bias_sigma_m"].as<double>(30.0);
      params_.config.max_age_s = clk["max_age_s"].as<double>(10.0);
    }

    // Load beacons
    beacons_.clear();
    for (const auto& node : root["beacons"]) {
      STNBeacon beacon;
      beacon.id = node["id"].as<std::string>();
      beacon.name = node["name"].as<std::string>();
      beacon.type = node["type"].as<std::string>();
      beacon.latitude = node["latitude"].as<double>();
      beacon.longitude = node["longitude"].as<double>();
      beacon.altitude_msl = node["altitude_msl"].as<double>();
      beacon.frequency_mhz = node["frequency_mhz"].as<double>();
      beacon.power_dbm = node["power_dbm"].as<double>();
      beacon.clock_bias_sigma_m = node["clock_bias_sigma_m"].as<double>();
      beacon.tdoa_sigma_m = node["tdoa_sigma_m"].as<double>();
      beacon.enabled = node["enabled"].as<bool>(true);

      // Convert to NED using robust ECEF conversion
      beacon.position_ned = geodeticToNED(beacon.latitude, beacon.longitude, beacon.altitude_msl);

      if (beacon.enabled) {
        beacons_[beacon.id] = beacon;
      }
    }

    return !beacons_.empty();
  } catch (const std::exception& e) {
    return false;
  }
}

// ECEF conversion helpers
void STNManager::geodeticToECEF(double lat_rad, double lon_rad, double h, Eigen::Vector3d& ecef) {
  const double a = 6378137.0;
  const double e2 = 0.00669437999014;
  const double s = std::sin(lat_rad);
  const double c = std::cos(lat_rad);
  const double N = a / std::sqrt(1.0 - e2 * s * s);
  ecef << (N + h) * c * std::cos(lon_rad),
          (N + h) * c * std::sin(lon_rad),
          (N * (1 - e2) + h) * s;
}

Eigen::Matrix3d STNManager::ecefToEnuRot(double lat_rad, double lon_rad) {
  const double sL = std::sin(lat_rad);
  const double cL = std::cos(lat_rad);
  const double sl = std::sin(lon_rad);
  const double cl = std::cos(lon_rad);
  Eigen::Matrix3d R;
  R << -sl,       cl,      0,
       -sL * cl, -sL * sl, cL,
        cL * cl,  cL * sl, sL;
  return R;
}

Eigen::Vector3d STNManager::geodeticToNED(double lat_deg, double lon_deg, double alt_msl) const {
  const double deg2rad = M_PI / 180.0;
  const double lat0 = params_.origin_lat_deg * deg2rad;
  const double lon0 = params_.origin_lon_deg * deg2rad;
  const double lat = lat_deg * deg2rad;
  const double lon = lon_deg * deg2rad;

  Eigen::Vector3d ecef0, ecef;
  geodeticToECEF(lat0, lon0, params_.origin_alt_msl, ecef0);
  geodeticToECEF(lat, lon, alt_msl, ecef);

  Eigen::Matrix3d R = ecefToEnuRot(lat0, lon0);
  Eigen::Vector3d enu = R * (ecef - ecef0);

  // ENU to NED
  return Eigen::Vector3d(enu.y(), enu.x(), -enu.z());
}

// TOA prediction (includes clock bias)
double STNManager::predictTOA(const Eigen::Vector3d& position_ned,
                              const std::string& beacon_id,
                              double clock_bias_m) const {
  auto* beacon = getBeacon(beacon_id);
  if (!beacon) return 0.0;

  double range = computeRange(position_ned, *beacon);
  return range + clock_bias_m;
}

// TDOA prediction (clock cancels)
double STNManager::predictTDOA(const Eigen::Vector3d& position_ned,
                               const std::string& beacon_id,
                               const std::string& ref_beacon_id) const {
  auto* beacon_i = getBeacon(beacon_id);
  auto* beacon_ref = getBeacon(ref_beacon_id);
  if (!beacon_i || !beacon_ref) return 0.0;

  double range_i = computeRange(position_ned, *beacon_i);
  double range_ref = computeRange(position_ned, *beacon_ref);
  return range_i - range_ref;  // Clock bias cancels
}

// Doppler prediction (includes clock drift as fractional frequency)
double STNManager::predictDoppler(const Eigen::Vector3d& position_ned,
                                  const Eigen::Vector3d& velocity_ned,
                                  const std::string& beacon_id,
                                  double clock_drift_frac) const {
  auto* beacon = getBeacon(beacon_id);
  if (!beacon) return 0.0;

  Eigen::Vector3d unit = computeUnitVector(position_ned, *beacon);
  double radial_velocity = velocity_ned.dot(unit);

  const double c = 299792458.0;
  double freq_hz = beacon->frequency_mhz * 1e6;

  // Doppler = -(fc/c) * v_radial + fc * clock_drift_frac
  return -(freq_hz / c) * radial_velocity + freq_hz * clock_drift_frac;
}

// FDOA prediction (drift cancels)
double STNManager::predictFDOA(const Eigen::Vector3d& position_ned,
                               const Eigen::Vector3d& velocity_ned,
                               const std::string& beacon_id,
                               const std::string& ref_beacon_id) const {
  auto* beacon_i = getBeacon(beacon_id);
  auto* beacon_ref = getBeacon(ref_beacon_id);
  if (!beacon_i || !beacon_ref) return 0.0;

  // Assume same frequency for simplicity
  double freq_hz = beacon_i->frequency_mhz * 1e6;
  const double c = 299792458.0;

  Eigen::Vector3d unit_i = computeUnitVector(position_ned, *beacon_i);
  Eigen::Vector3d unit_ref = computeUnitVector(position_ned, *beacon_ref);

  // FDOA = -(fc/c) * (u_i - u_ref) · v
  return -(freq_hz / c) * ((unit_i - unit_ref).dot(velocity_ned));
}

// GDOP for TOA (with clock column)
GDOPResult STNManager::computeGDOP_TOA(const Eigen::Vector3d& position_ned,
                                       const std::vector<std::string>& beacon_ids,
                                       const std::vector<double>* sigmas) const {
  GDOPResult result;
  result.gdop = 999.9;  // Bad value

  if (beacon_ids.size() < 4) {
    return result;
  }

  // Build geometry matrix H
  Eigen::MatrixXd H(beacon_ids.size(), 4);
  int row = 0;

  for (const auto& id : beacon_ids) {
    auto it = beacons_.find(id);
    if (it == beacons_.end()) continue;

    const auto& beacon = it->second;
    Eigen::Vector3d unit = computeUnitVector(position_ned, beacon);

    H(row, 0) = unit.x();  // North
    H(row, 1) = unit.y();  // East
    H(row, 2) = unit.z();  // Down
    H(row, 3) = 1.0;        // Clock bias
    row++;
  }

  if (row < 4) {
    return result;
  }

  H.conservativeResize(row, 4);

  // Optional whitening by measurement uncertainty
  if (sigmas && sigmas->size() == beacon_ids.size()) {
    Eigen::VectorXd W(row);
    for (int i = 0; i < row; ++i) {
      W(i) = 1.0 / std::max(1e-3, (*sigmas)[i]);
    }
    H = W.asDiagonal() * H;
  }

  // Compute DOP matrix: (H^T H)^-1
  Eigen::Matrix4d HTH = H.transpose() * H;
  Eigen::Matrix4d Q = HTH.ldlt().solve(Eigen::Matrix4d::Identity());

  if (!Q.allFinite()) {
    return result;
  }

  // Extract DOP values
  result.pdop = std::sqrt(Q(0,0) + Q(1,1) + Q(2,2));
  result.hdop = std::sqrt(Q(0,0) + Q(1,1));
  result.vdop = std::sqrt(Q(2,2));
  result.tdop = std::sqrt(Q(3,3));
  result.gdop = std::sqrt(Q.trace());

  return result;
}

// GDOP for TDOA (no clock column)
GDOPResult STNManager::computeGDOP_TDOA(const Eigen::Vector3d& position_ned,
                                        const std::vector<std::string>& beacon_ids,
                                        const std::string& ref_beacon_id,
                                        const std::vector<double>* sigmas) const {
  GDOPResult result;
  result.gdop = 999.9;

  auto* beacon_ref = getBeacon(ref_beacon_id);
  if (!beacon_ref || beacon_ids.empty()) {
    return result;
  }

  // Build differenced geometry matrix
  Eigen::MatrixXd H(beacon_ids.size(), 3);  // No clock column
  Eigen::Vector3d unit_ref = computeUnitVector(position_ned, *beacon_ref);
  int row = 0;

  for (const auto& id : beacon_ids) {
    if (id == ref_beacon_id) continue;  // Skip reference beacon

    auto it = beacons_.find(id);
    if (it == beacons_.end()) continue;

    const auto& beacon = it->second;
    Eigen::Vector3d unit_i = computeUnitVector(position_ned, beacon);
    Eigen::Vector3d diff = unit_i - unit_ref;

    H(row, 0) = diff.x();
    H(row, 1) = diff.y();
    H(row, 2) = diff.z();
    row++;
  }

  if (row < 3) {
    return result;
  }

  H.conservativeResize(row, 3);

  // Optional whitening
  if (sigmas && sigmas->size() >= row) {
    Eigen::VectorXd W(row);
    for (int i = 0; i < row; ++i) {
      W(i) = 1.0 / std::max(1e-3, (*sigmas)[i]);
    }
    H = W.asDiagonal() * H;
  }

  Eigen::Matrix3d HTH = H.transpose() * H;
  Eigen::Matrix3d Q = HTH.ldlt().solve(Eigen::Matrix3d::Identity());

  if (!Q.allFinite()) {
    return result;
  }

  result.pdop = std::sqrt(Q(0,0) + Q(1,1) + Q(2,2));
  result.hdop = std::sqrt(Q(0,0) + Q(1,1));
  result.vdop = std::sqrt(Q(2,2));
  result.tdop = 0.0;  // No time state in TDOA
  result.gdop = std::sqrt(Q.trace());

  return result;
}

// Select beacons with fallback (always return best available)
std::vector<std::string> STNManager::selectBeacons(const Eigen::Vector3d& position_ned,
                                                  const std::vector<STNObservation>& observations) const {
  // Filter valid observations above SNR threshold
  std::vector<STNObservation> valid_obs;
  for (const auto& obs : observations) {
    if (!obs.is_valid || obs.snr_db < params_.config.snr_threshold_db) continue;
    if (!getBeacon(obs.beacon_id)) continue;
    valid_obs.push_back(obs);
  }

  if (valid_obs.size() < static_cast<size_t>(params_.config.min_beacons)) {
    return {};
  }

  // Sort by SNR (best first)
  std::sort(valid_obs.begin(), valid_obs.end(),
           [](const auto& a, const auto& b) { return a.snr_db > b.snr_db; });

  // Try different combinations to find best GDOP
  double best_gdop = 999.9;
  std::vector<std::string> best_set;

  int max_size = std::min(static_cast<int>(valid_obs.size()), params_.config.max_beacons);

  for (int size = params_.config.min_beacons; size <= max_size; size++) {
    std::vector<std::string> candidate;
    for (int i = 0; i < size; i++) {
      candidate.push_back(valid_obs[i].beacon_id);
    }

    GDOPResult gdop = computeGDOP_TOA(position_ned, candidate);
    if (gdop.gdop < best_gdop) {
      best_gdop = gdop.gdop;
      best_set = candidate;
    }
  }

  // Return best set even if GDOP exceeds threshold
  return best_set;
}

void STNManager::processObservations(const std::vector<STNObservation>& observations,
                                    double current_time) {
  // Remove old observations
  observation_buffer_.erase(
    std::remove_if(observation_buffer_.begin(), observation_buffer_.end(),
                  [current_time, this](const auto& obs) {
                    return (current_time - obs.timestamp) > params_.config.max_age_s;
                  }),
    observation_buffer_.end()
  );

  // Add new observations
  observation_buffer_.insert(observation_buffer_.end(), observations.begin(), observations.end());
}

// RAIM with whitened residuals
std::vector<std::string> STNManager::performRAIM(const Eigen::Vector3d& position_ned,
                                                const std::vector<STNObservation>& observations,
                                                const Eigen::Vector2d& clock_state) const {
  if (!params_.config.raim_enabled || observations.size() < 5) {
    std::vector<std::string> all_ids;
    for (const auto& obs : observations) {
      all_ids.push_back(obs.beacon_id);
    }
    return all_ids;
  }

  std::vector<std::string> good_beacons;
  std::vector<double> whitened_residuals;

  // Compute whitened residuals for each observation
  for (const auto& obs : observations) {
    double predicted = 0.0;
    double residual = 0.0;

    switch (obs.type) {
      case STNMeasurementType::TOA:
        predicted = predictTOA(position_ned, obs.beacon_id, clock_state(0));
        residual = std::abs(obs.value - predicted) / obs.sigma;
        break;

      case STNMeasurementType::TDOA:
        if (!obs.ref_beacon_id.empty()) {
          predicted = predictTDOA(position_ned, obs.beacon_id, obs.ref_beacon_id);
          residual = std::abs(obs.value - predicted) / obs.sigma;
        }
        break;

      default:
        continue;
    }

    whitened_residuals.push_back(residual);
    good_beacons.push_back(obs.beacon_id);
  }

  // Iteratively remove outliers
  int outliers_removed = 0;
  while (outliers_removed < params_.config.raim_max_outliers && good_beacons.size() > 4) {
    // Find worst residual
    auto max_it = std::max_element(whitened_residuals.begin(), whitened_residuals.end());
    if (*max_it < 3.0) {
      break;  // No significant outliers (3-sigma threshold)
    }

    // Remove worst beacon
    size_t idx = std::distance(whitened_residuals.begin(), max_it);
    good_beacons.erase(good_beacons.begin() + idx);
    whitened_residuals.erase(whitened_residuals.begin() + idx);
    outliers_removed++;
  }

  return good_beacons;
}

const STNBeacon* STNManager::getBeacon(const std::string& id) const {
  auto it = beacons_.find(id);
  return (it != beacons_.end()) ? &it->second : nullptr;
}

std::vector<STNBeacon> STNManager::getActiveBeacons() const {
  std::vector<STNBeacon> active;
  for (const auto& [id, beacon] : beacons_) {
    if (beacon.enabled) {
      active.push_back(beacon);
    }
  }
  return active;
}

double STNManager::computeRange(const Eigen::Vector3d& position_ned,
                               const STNBeacon& beacon) const {
  Eigen::Vector3d delta = beacon.position_ned - position_ned;
  return delta.norm();
}

Eigen::Vector3d STNManager::computeUnitVector(const Eigen::Vector3d& position_ned,
                                             const STNBeacon& beacon) const {
  Eigen::Vector3d delta = beacon.position_ned - position_ned;
  double range = delta.norm();
  if (range < 0.001) {
    // Log and skip beacon - something is wrong if range is that small
    LOG_WARN("STN: Beacon {} too close (range={:.6f}m), skipping", beacon.id, range);
    return Eigen::Vector3d::Zero();  // Signal invalid - caller should check
  }
  return delta / range;
}

double STNManager::earthRadius(double lat_deg) const {
  double lat_rad = lat_deg * M_PI / 180.0;
  double sin_lat = std::sin(lat_rad);
  double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
  return N;
}

double STNManager::metersPerDegreeLat() const {
  return 111132.92 - 559.82 * std::cos(2 * params_.origin_lat_deg * M_PI / 180.0);
}

double STNManager::metersPerDegreeLon(double lat_deg) const {
  double lat_rad = lat_deg * M_PI / 180.0;
  return 111412.84 * std::cos(lat_rad) - 93.5 * std::cos(3 * lat_rad);
}

} // namespace measurements