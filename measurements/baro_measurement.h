#pragma once
#include <Eigen/Dense>
#include <memory>

namespace measurements {

// Barometer measurement data
struct BaroData {
  double timestamp;           // s (monotonic)
  double pressure_pa;         // Pa (static)
  double temperature_c;       // °C (ambient)
  double altitude_m;          // Optional: baro-computed MSL (if sensor gives it)
  double sigma_m;             // 1-sigma alt uncertainty (m)

  // Per-sample adjustments (from calibration)
  double temp_bias = 0.0;     // m/°C residual temp bias (optional)
  double offset_m = 0.0;      // m static offset (optional)
};

// Barometer configuration & atmosphere
struct BaroConfig {
  bool   enabled = true;
  double update_rate_hz = 50.0;

  // Measurement model
  double sigma_m = 1.5;             // Base 1-sigma (m)
  double chi2_gate_1d = 9.0;        // ≈3σ for 1 dof
  double max_innovation_m = 10.0;

  // Reference: MSL altitude of NED origin (e.g., from config.common.origin.alt_m)
  double reference_alt_msl_m = 0.0; // m MSL

  // Atmosphere (ISA)
  double sea_level_pressure_pa = 101325.0;  // Pa (QNH)
  double sea_level_temp_K      = 288.15;    // K (15°C)
  double lapse_rate_Kpm        = 0.0065;    // K/m (positive magnitude)
  double gas_constant_R        = 287.05;    // J/(kg·K)
  double gravity               = 9.80665;   // m/s²

  // Filtering
  bool   use_low_pass = true;
  double filter_time_constant = 2.0; // s

  // Simple temperature compensation
  double temp_comp_factor_m_per_C = 0.1;  // m/°C
};

// Barometer measurement for UKF update
class BaroMeasurement {
public:
  explicit BaroMeasurement(const BaroConfig& config);
  ~BaroMeasurement() = default;

  // Process raw barometer data and compute innovation against current state
  bool processMeasurement(const BaroData& data, const Eigen::Vector3d& position_ned);

  // Predicted baro-measured MSL altitude from current NED position
  double predictMSL(const Eigen::Vector3d& position_ned) const;

  double getValue() const { return measurement_msl_; }
  double getSigma() const { return sigma_; }
  double getInnovation() const { return innovation_; }
  double getChi2() const { return chi2_; }
  bool   isValid() const { return is_valid_; }

  // Atmosphere conversions (ISA, troposphere, lapse-rate layer)
  static double pressureToAltitudeMSL(double pressure_pa,
                                      double sea_level_pressure_pa = 101325.0,
                                      double sea_level_temp_K = 288.15,
                                      double lapse_rate_Kpm = 0.0065,
                                      double gas_constant_R = 287.05,
                                      double gravity = 9.80665);

  static double altitudeMSLToPressure(double altitude_m,
                                      double sea_level_pressure_pa = 101325.0,
                                      double sea_level_temp_K = 288.15,
                                      double lapse_rate_Kpm = 0.0065,
                                      double gas_constant_R = 287.05,
                                      double gravity = 9.80665);

  // Optional: estimate QNH (sea-level pressure) from a known MSL altitude
  static double calibrateSeaLevelPressure(double known_alt_msl_m,
                                          double measured_pressure_pa,
                                          double sea_level_temp_K = 288.15,
                                          double lapse_rate_Kpm = 0.0065,
                                          double gas_constant_R = 287.05,
                                          double gravity = 9.80665);

private:
  BaroConfig cfg_;

  // Current measurement (MSL altitude, m)
  double measurement_msl_ = 0.0;
  double sigma_ = 0.0;
  double innovation_ = 0.0;
  double chi2_ = 0.0;
  bool   is_valid_ = false;

  // Low-pass filter state
  double filtered_altitude_msl_ = 0.0;
  double last_update_time_ = -1.0;
  bool   filter_initialized_ = false;

  // Helpers
  double applyTemperatureCompensation(double altitude_m, double temperature_c, double temp_bias) const;
  double applyLowPassFilter(double altitude_m, double timestamp);
};

} // namespace measurements
