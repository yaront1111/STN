#include "stn_synthetic_generator.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace measurements {

STNSyntheticGenerator::STNSyntheticGenerator(std::shared_ptr<STNManager> stn_manager,
                                           const Params& params)
  : stn_manager_(std::move(stn_manager)),
    params_(params),
    rng_(std::random_device{}()),
    noise_dist_(0.0, 1.0),
    uniform_dist_(0.0, 1.0),
    current_clock_bias_m_(params.true_clock_bias_m),
    current_clock_drift_frac_(params.true_clock_drift_frac),
    last_update_time_(0.0) {}

std::vector<STNObservation> STNSyntheticGenerator::generateObservations(
    const aion::ExtendedState& true_state, double current_time) {
  return generateObservationsWithClock(true_state, current_time,
                                      current_clock_bias_m_,
                                      current_clock_drift_frac_);
}

std::vector<STNObservation> STNSyntheticGenerator::generateObservationsWithClock(
    const aion::ExtendedState& true_state,
    double current_time,
    double clock_bias_m,
    double clock_drift_frac) {

  std::vector<STNObservation> observations;

  // Update internal clock state
  if (current_time > last_update_time_) {
    double dt = current_time - last_update_time_;
    updateClockState(dt);
    last_update_time_ = current_time;
  }

  // Get all active beacons
  auto beacons = stn_manager_->getActiveBeacons();

  // Debug output
  std::cout << "Debug: Generating observations at time " << current_time << "\n";
  std::cout << "  Position NED: " << true_state.position.transpose() << "\n";
  std::cout << "  Beacons available: " << beacons.size() << "\n";

  // Find best beacon for reference (highest SNR)
  std::string ref_beacon_id;
  double max_snr = -100.0;

  for (const auto& beacon : beacons) {
    // Debug visibility
    Eigen::Vector3d delta = beacon.position_ned - true_state.position;
    double range = delta.norm();
    double elevation_rad = std::asin(-delta.z() / range);
    double elevation_deg = elevation_rad * 180.0 / M_PI;

    std::cout << "  Beacon " << beacon.id << ":\n";
    std::cout << "    Position NED: " << beacon.position_ned.transpose() << "\n";
    std::cout << "    Range: " << range << " m\n";
    std::cout << "    Elevation: " << elevation_deg << " deg\n";

    // Check visibility
    if (!isBeaconVisible(true_state.position, beacon)) {
      std::cout << "    Visibility: FAILED (max_range=" << params_.max_range_m
                << ", min_elev=" << params_.min_elevation_deg << ")\n";
      continue;
    }

    double snr_db = computeSNR(range, beacon.power_dbm);
    std::cout << "    SNR: " << snr_db << " dB\n";

    if (snr_db > max_snr) {
      max_snr = snr_db;
      ref_beacon_id = beacon.id;
    }
  }

  if (ref_beacon_id.empty()) {
    std::cout << "  No reference beacon found!\n";
    return observations;
  }

  std::cout << "  Reference beacon: " << ref_beacon_id << " (SNR: " << max_snr << " dB)\n";

  // Decide measurement type for this epoch (alternating TOA and TDOA)
  bool use_toa = (static_cast<int>(current_time) % 4 < 2); // TOA for 2s, then TDOA for 2s

  for (const auto& beacon : beacons) {
    // Check visibility
    if (!isBeaconVisible(true_state.position, beacon)) {
      continue;
    }

    // Simulate signal loss
    if (uniform_dist_(rng_) < params_.prob_loss) {
      continue;
    }

    // Compute true range
    Eigen::Vector3d delta = beacon.position_ned - true_state.position;
    double true_range = delta.norm();

    // Compute elevation angle for error modeling
    double elevation_rad = std::asin(-delta.z() / true_range);
    double elevation_deg = elevation_rad * 180.0 / M_PI;

    // Compute SNR
    double snr_db = computeSNR(true_range, beacon.power_dbm);
    if (snr_db < 10.0) continue;  // Too weak

    if (use_toa) {
      // Generate TOA measurement (includes clock bias)
      STNObservation toa_obs;
      toa_obs.timestamp = current_time;
      toa_obs.beacon_id = beacon.id;
      toa_obs.type = STNMeasurementType::TOA;

      // True TOA = range + clock bias
      double true_toa = true_range + clock_bias_m;

      // Add errors
      double multipath = simulateMultipath(elevation_deg);
      double ionosphere = simulateIonosphericDelay(beacon.frequency_mhz, elevation_deg);
      double noise = addNoise(0.0, params_.tdoa_noise_m);

      toa_obs.value = true_toa + multipath + ionosphere + noise;
      toa_obs.sigma = beacon.tdoa_sigma_m;
      toa_obs.snr_db = snr_db;
      toa_obs.carrier_phase = 0.0;
      toa_obs.is_valid = true;

      observations.push_back(toa_obs);
    } else {
      // Generate TDOA measurement (clock bias cancels)
      if (beacon.id == ref_beacon_id) continue; // Skip reference beacon

      STNObservation tdoa_obs;
      tdoa_obs.timestamp = current_time;
      tdoa_obs.beacon_id = beacon.id;
      tdoa_obs.ref_beacon_id = ref_beacon_id;
      tdoa_obs.type = STNMeasurementType::TDOA;

      // Find reference beacon
      const STNBeacon* ref_beacon = stn_manager_->getBeacon(ref_beacon_id);
      if (!ref_beacon) continue;

      // True TDOA = range_i - range_ref (clock cancels!)
      double ref_range = (ref_beacon->position_ned - true_state.position).norm();
      double true_tdoa = true_range - ref_range;

      // Add errors
      double multipath = simulateMultipath(elevation_deg);
      double ionosphere = simulateIonosphericDelay(beacon.frequency_mhz, elevation_deg);
      double noise = addNoise(0.0, params_.tdoa_noise_m);

      tdoa_obs.value = true_tdoa + multipath + ionosphere + noise;
      tdoa_obs.sigma = beacon.tdoa_sigma_m * std::sqrt(2.0); // TDOA has higher noise
      tdoa_obs.snr_db = snr_db;
      tdoa_obs.carrier_phase = 0.0;
      tdoa_obs.is_valid = true;

      observations.push_back(tdoa_obs);
    }

    // Generate Doppler measurement (less frequently)
    if (uniform_dist_(rng_) < params_.doppler_rate_hz * 0.05) {
      STNObservation doppler_obs;
      doppler_obs.timestamp = current_time;
      doppler_obs.beacon_id = beacon.id;
      doppler_obs.type = STNMeasurementType::DOPPLER;

      // Compute radial velocity
      Eigen::Vector3d unit = delta / true_range;
      double radial_velocity = true_state.velocity.dot(unit);

      // Doppler shift in Hz: -(fc/c) * radial_vel + fc * clock_drift_frac
      const double c = 299792458.0;
      double freq_hz = beacon.frequency_mhz * 1e6;
      double true_doppler = -(freq_hz / c) * radial_velocity + freq_hz * clock_drift_frac;

      // Add noise
      double noise = addNoise(0.0, params_.doppler_noise_hz);

      doppler_obs.value = true_doppler + noise;
      doppler_obs.sigma = params_.doppler_noise_hz;
      doppler_obs.snr_db = snr_db;
      doppler_obs.carrier_phase = 0.0;
      doppler_obs.is_valid = true;

      observations.push_back(doppler_obs);
    }

    // Could also generate FDOA measurements (differential Doppler)
    // but skipping for simplicity
  }

  std::cout << "  Generated " << observations.size() << " observations\n\n";
  return observations;
}

double STNSyntheticGenerator::simulateMultipath(double elevation_deg) const {
  // Simple multipath model: higher at low elevations
  if (elevation_deg < 5.0) {
    return addNoise(0.0, 10.0);  // High multipath below 5 degrees
  } else if (elevation_deg < 15.0) {
    return addNoise(0.0, 5.0);   // Medium multipath
  } else if (elevation_deg < 30.0) {
    return addNoise(0.0, 2.0);   // Low multipath
  }
  return addNoise(0.0, 0.5);     // Minimal multipath at high elevations
}

double STNSyntheticGenerator::simulateIonosphericDelay(double frequency_mhz,
                                                      double elevation_deg) const {
  // Simple ionospheric model (frequency and elevation dependent)
  const double f_squared = frequency_mhz * frequency_mhz;
  const double iono_constant = 40.3e16;  // Approximate TEC value

  // Obliquity factor
  double sin_el = std::sin(elevation_deg * M_PI / 180.0);
  double obliquity = 1.0 / std::sqrt(1.0 - 0.8 * 0.8 * (1.0 - sin_el * sin_el));

  // Delay in meters
  double delay_m = iono_constant / (f_squared * 1e12) * obliquity;

  // Add some variability
  return delay_m * (1.0 + addNoise(0.0, 0.1));
}

bool STNSyntheticGenerator::isBeaconVisible(const Eigen::Vector3d& position_ned,
                                           const STNBeacon& beacon) const {
  Eigen::Vector3d delta = beacon.position_ned - position_ned;
  double range = delta.norm();

  // Check range
  if (range > params_.max_range_m) {
    return false;
  }

  // Check elevation angle (beacon above receiver)
  double elevation_rad = std::asin(-delta.z() / range);
  double elevation_deg = elevation_rad * 180.0 / M_PI;

  return elevation_deg >= params_.min_elevation_deg;
}

double STNSyntheticGenerator::computeSNR(double range_m, double power_dbm) const {
  // Free space path loss model
  double range_km = range_m / 1000.0;
  double path_loss_db = params_.snr_range_factor * std::log10(range_km);

  return params_.snr_base_db + power_dbm - 43.0 - path_loss_db;
}

double STNSyntheticGenerator::addNoise(double value, double sigma) const {
  return value + sigma * noise_dist_(rng_);
}

void STNSyntheticGenerator::updateClockState(double dt) {
  // Random walk for clock bias (drift integrated over time)
  const double c = 299792458.0;
  current_clock_bias_m_ += current_clock_drift_frac_ * c * dt;  // drift_frac * c gives m/s
  current_clock_bias_m_ += addNoise(0.0, params_.clock_noise_m * std::sqrt(dt));

  // Random walk for clock drift (in fractional frequency units)
  current_clock_drift_frac_ += addNoise(0.0, 1e-12 * std::sqrt(dt)); // 1 ppt/sqrt(s) drift noise
}

} // namespace measurements