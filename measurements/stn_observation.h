#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace measurements {

// Types of STN measurements
enum class STNMeasurementType {
  TOA,       // Time of Arrival (pseudorange with clock bias)
  TDOA,      // Time Difference of Arrival (clock cancels)
  DOPPLER,   // Doppler shift (with clock drift)
  FDOA,      // Frequency Difference of Arrival (drift cancels)
  RSS        // Received Signal Strength (for ranging)
};

// Single beacon definition
struct STNBeacon {
  std::string id;
  std::string name;
  std::string type;  // LTE, 5G, UWB, WiFi, etc.
  
  // Position in geodetic coordinates
  double latitude;   // degrees
  double longitude;  // degrees
  double altitude_msl; // meters
  
  // Position in local NED (computed from geodetic)
  Eigen::Vector3d position_ned;
  
  // RF characteristics
  double frequency_mhz;
  double power_dbm;
  
  // Uncertainty parameters
  double clock_bias_sigma_m;
  double tdoa_sigma_m;
  
  bool enabled;
};

// STN observation from RF frontend
struct STNObservation {
  double timestamp;           // Reception time (s)
  std::string beacon_id;      // Beacon identifier
  STNMeasurementType type;    // TOA, TDOA, Doppler, FDOA, RSS
  double value;               // TOA/TDOA: meters; Doppler/FDOA: Hz
  double sigma;               // Measurement uncertainty (same units as value)
  double snr_db;              // Signal-to-noise ratio
  std::string ref_beacon_id;  // Reference beacon for TDOA/FDOA (empty if N/A)
  double carrier_phase;       // Carrier phase (if available)
  bool is_valid;              // Validity flag
};

// STN configuration
struct STNConfig {
  bool enabled = false;
  std::string beacons_file = "configs/stn/beacons.yaml";
  
  // Beacon selection
  int min_beacons = 4;
  int max_beacons = 8;
  double gdop_threshold = 6.0;
  double snr_threshold_db = 10.0;
  double chi2_gate_threshold = 25.0;
  
  // RAIM (Receiver Autonomous Integrity Monitoring)
  bool raim_enabled = true;
  int raim_max_outliers = 2;
  
  // Update rate
  double update_rate_hz = 2.0;
  
  // Clock parameters
  bool use_ocxo = true;
  double clock_drift_sigma_ppm = 0.01;
  double clock_bias_sigma_m = 30.0;
  double max_age_s = 10.0;
  
  // Chi-squared gating
  double chi2_gate_1d = 9.0;  // 1-DOF chi-squared threshold
  double max_innovation_m = 100.0;
};

// GDOP (Geometric Dilution of Precision) result
struct GDOPResult {
  double gdop;      // Overall geometric DOP
  double pdop;      // Position DOP (3D)
  double hdop;      // Horizontal DOP
  double vdop;      // Vertical DOP
  double tdop;      // Time DOP
  
  bool is_acceptable() const {
    return gdop < 6.0;  // Common threshold for good geometry
  }
};

} // namespace measurements
