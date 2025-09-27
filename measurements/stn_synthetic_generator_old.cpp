#include "stn_synthetic_generator.h"
#include <cmath>
#include <algorithm>

namespace measurements {

STNSyntheticGenerator::STNSyntheticGenerator(std::shared_ptr<STNManager> stn_manager,
                                           const Params& params)
  : stn_manager_(std::move(stn_manager)),
    params_(params),
    rng_(std::random_device{}()),
    noise_dist_(0.0, 1.0),
    uniform_dist_(0.0, 1.0),
    current_clock_bias_m_(params.true_clock_bias_m),
    current_clock_drift_mps_(params.true_clock_drift_mps),
    last_update_time_(0.0) {}

std::vector<STNObservation> STNSyntheticGenerator::generateObservations(
    const aion::ExtendedState& true_state, double current_time) {
  return generateObservationsWithClock(true_state, current_time,
                                      current_clock_bias_m_,
                                      current_clock_drift_mps_);
}

std::vector<STNObservation> STNSyntheticGenerator::generateObservationsWithClock(
    const aion::ExtendedState& true_state,
    double current_time,
    double clock_bias_m,
    double clock_drift_mps) {

  std::vector<STNObservation> observations;

  // Update internal clock state
  if (current_time > last_update_time_) {
    double dt = current_time - last_update_time_;
    updateClockState(dt);
    last_update_time_ = current_time;
  }

  // Get all active beacons
  auto beacons = stn_manager_->getActiveBeacons();

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

    // Generate TDOA measurement
    if (uniform_dist_(rng_) < params_.tdoa_rate_hz * 0.1) {  // 10Hz base rate, decimated
      STNObservation tdoa_obs;
      tdoa_obs.timestamp = current_time;
      tdoa_obs.beacon_id = beacon.id;
      tdoa_obs.type = STNMeasurementType::TDOA;

      // True TDOA = range + clock bias
      double true_tdoa = true_range + clock_bias_m;

      // Add errors
      double multipath = simulateMultipath(elevation_deg);
      double ionosphere = simulateIonosphericDelay(beacon.frequency_mhz, elevation_deg);
      double noise = addNoise(0.0, params_.tdoa_noise_m);

      tdoa_obs.value = true_tdoa + multipath + ionosphere + noise;
      tdoa_obs.sigma = beacon.tdoa_sigma_m;
      tdoa_obs.snr_db = snr_db;
      tdoa_obs.carrier_phase = 0.0;  // Not used for TDOA
      tdoa_obs.is_valid = true;

      observations.push_back(tdoa_obs);
    }

    // Generate Doppler measurement
    if (uniform_dist_(rng_) < params_.doppler_rate_hz * 0.1) {
      STNObservation doppler_obs;
      doppler_obs.timestamp = current_time;
      doppler_obs.beacon_id = beacon.id;
      doppler_obs.type = STNMeasurementType::DOPPLER;

      // Compute radial velocity
      Eigen::Vector3d unit = delta / true_range;
      double radial_velocity = true_state.velocity.dot(unit);

      // Doppler shift in Hz
      const double c = 299792458.0;
      double freq_hz = beacon.frequency_mhz * 1e6;
      double true_doppler = -radial_velocity * freq_hz / c;

      // Add clock drift contribution
      double clock_doppler = clock_drift_mps * freq_hz / c;

      // Add noise
      double noise = addNoise(0.0, params_.doppler_noise_hz);

      doppler_obs.value = true_doppler + clock_doppler + noise;
      doppler_obs.sigma = params_.doppler_noise_hz;
      doppler_obs.snr_db = snr_db;
      doppler_obs.carrier_phase = 0.0;
      doppler_obs.is_valid = true;

      observations.push_back(doppler_obs);
    }
  }

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

  // Check elevation angle
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
  // Random walk for clock bias
  current_clock_bias_m_ += current_clock_drift_mps_ * dt;
  current_clock_bias_m_ += addNoise(0.0, params_.clock_noise_m * std::sqrt(dt));

  // Random walk for clock drift
  current_clock_drift_mps_ += addNoise(0.0, 0.01 * std::sqrt(dt));
}

} // namespace measurements