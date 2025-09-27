#pragma once

#include "stn_observation.h"
#include "stn_manager.h"
#include "../core/fusion/state_extended.h"
#include <random>
#include <vector>

namespace measurements {

// Generates synthetic STN observations for testing
class STNSyntheticGenerator {
public:
  struct Params {
    // Clock simulation
    double true_clock_bias_m;      // True clock bias
    double true_clock_drift_frac;   // True clock drift (Hz/Hz or ppm)
    double clock_noise_m;           // Clock noise sigma

    // Measurement noise
    double tdoa_noise_m;           // TDOA measurement noise
    double doppler_noise_hz;        // Doppler measurement noise
    double rss_noise_db;            // RSS measurement noise

    // Beacon visibility
    double max_range_m;         // Maximum beacon range (50km)
    double min_elevation_deg;      // Minimum elevation angle
    double prob_loss;              // Probability of signal loss

    // SNR simulation
    double snr_base_db;            // Base SNR at 1km
    double snr_range_factor;       // SNR loss per decade of range

    // Update rates
    double tdoa_rate_hz;            // TDOA measurement rate
    double doppler_rate_hz;         // Doppler measurement rate

    Params()
      : true_clock_bias_m(50.0),
        true_clock_drift_frac(1e-9),
        clock_noise_m(5.0),
        tdoa_noise_m(20.0),
        doppler_noise_hz(1.0),
        rss_noise_db(3.0),
        max_range_m(50000.0),
        min_elevation_deg(10.0),
        prob_loss(0.05),
        snr_base_db(30.0),
        snr_range_factor(20.0),
        tdoa_rate_hz(2.0),
        doppler_rate_hz(1.0) {}
  };

  STNSyntheticGenerator(std::shared_ptr<STNManager> stn_manager,
                       const Params& params = Params());

  // Generate observations for current state
  std::vector<STNObservation> generateObservations(const aion::ExtendedState& true_state,
                                                  double current_time);

  // Generate observations with custom clock error
  std::vector<STNObservation> generateObservationsWithClock(
    const aion::ExtendedState& true_state,
    double current_time,
    double clock_bias_m,
    double clock_drift_frac);

  // Simulate multipath error
  double simulateMultipath(double elevation_deg) const;

  // Simulate ionospheric delay
  double simulateIonosphericDelay(double frequency_mhz, double elevation_deg) const;

  // Get current true clock states
  double getTrueClockBias() const { return current_clock_bias_m_; }
  double getTrueClockDrift() const { return current_clock_drift_frac_; }

  // Reset clock simulation
  void resetClock() {
    current_clock_bias_m_ = params_.true_clock_bias_m;
    current_clock_drift_frac_ = params_.true_clock_drift_frac;
  }

private:
  std::shared_ptr<STNManager> stn_manager_;
  Params params_;

  // Random number generators
  mutable std::mt19937 rng_;
  mutable std::normal_distribution<double> noise_dist_;
  mutable std::uniform_real_distribution<double> uniform_dist_;

  // Clock state simulation
  double current_clock_bias_m_;
  double current_clock_drift_frac_;
  double last_update_time_;

  // Helper functions
  bool isBeaconVisible(const Eigen::Vector3d& position_ned,
                       const STNBeacon& beacon) const;

  double computeSNR(double range_m, double power_dbm) const;

  double addNoise(double value, double sigma) const;

  void updateClockState(double dt);
};

} // namespace measurements