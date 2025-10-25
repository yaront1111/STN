// Thermal Compensation for CHIMERA Navigation System
// Corrects IMU bias and barometer altitude for temperature variation
//
// Features:
// - Polynomial-based IMU bias correction (gyro & accel vs temperature)
// - Barometer altitude temperature correction
// - LUT loading from versioned JSON files
// - Graceful fallback if thermal calibration unavailable

#pragma once

#include <Eigen/Dense>
#include <string>
#include <optional>

namespace chimera {
namespace core {

/// Thermal calibration data structure
/// Polynomial coefficients for temperature-dependent bias correction
struct ThermalCalibration {
  // IMU gyro bias vs temperature (rad/s per °C)
  // bias(T) = bias_T0 + bias_T1 * (T - T_ref) + bias_T2 * (T - T_ref)^2
  Eigen::Vector3d gyro_bias_T0;    // Bias at reference temperature (rad/s)
  Eigen::Vector3d gyro_bias_T1;    // Linear coefficient (rad/s/°C)
  Eigen::Vector3d gyro_bias_T2;    // Quadratic coefficient (rad/s/°C²)

  // IMU accel bias vs temperature (m/s² per °C)
  Eigen::Vector3d accel_bias_T0;   // Bias at reference temperature (m/s²)
  Eigen::Vector3d accel_bias_T1;   // Linear coefficient (m/s²/°C)
  Eigen::Vector3d accel_bias_T2;   // Quadratic coefficient (m/s²/°C²)

  // Barometer altitude correction (meters per °C)
  double baro_temp_coeff = 0.0;    // Linear correction coefficient

  // Reference temperature for polynomial expansion
  double reference_temp_c = 25.0;   // Default: 25°C

  // Calibration metadata
  std::string version = "1.0.0";
  std::string imu_model;
  std::string baro_model;
  uint64_t calibration_timestamp_unix = 0;

  bool valid = false;               // Whether calibration is valid

  /// Default constructor: zero compensation (fallback mode)
  ThermalCalibration() {
    gyro_bias_T0.setZero();
    gyro_bias_T1.setZero();
    gyro_bias_T2.setZero();
    accel_bias_T0.setZero();
    accel_bias_T1.setZero();
    accel_bias_T2.setZero();
  }
};

/// Thermal compensation engine
/// Applies temperature-dependent corrections to sensor measurements
class ThermalCompensation {
public:
  /// Constructor: Initialize with optional calibration file path
  /// @param lut_path Path to thermal LUT JSON file (optional)
  explicit ThermalCompensation(const std::string& lut_path = "");

  /// Load thermal calibration from JSON file
  /// @param lut_path Path to thermal LUT JSON file
  /// @return true if loaded successfully, false otherwise
  bool loadCalibration(const std::string& lut_path);

  /// Check if thermal compensation is active
  /// @return true if valid calibration loaded
  bool isActive() const { return calib_.valid; }

  /// Get reference temperature
  /// @return Reference temperature in °C
  double getReferenceTemp() const { return calib_.reference_temp_c; }

  /// Compute temperature-compensated gyro bias
  /// @param temp_c Current temperature in °C
  /// @return Predicted gyro bias at given temperature (rad/s)
  Eigen::Vector3d compensateGyroBias(double temp_c) const;

  /// Compute temperature-compensated accel bias
  /// @param temp_c Current temperature in °C
  /// @return Predicted accel bias at given temperature (m/s²)
  Eigen::Vector3d compensateAccelBias(double temp_c) const;

  /// Compute temperature-compensated barometer altitude
  /// @param altitude_raw Raw barometer altitude in meters
  /// @param temp_c Current temperature in °C
  /// @return Temperature-corrected altitude in meters
  double compensateBaroAltitude(double altitude_raw, double temp_c) const;

  /// Get calibration metadata for logging/diagnostics
  /// @return Calibration data structure (read-only)
  const ThermalCalibration& getCalibration() const { return calib_; }

  /// Export calibration to JSON string (for saving/debugging)
  /// @return JSON string representation of calibration
  std::string exportToJson() const;

  /// Validate calibration consistency
  /// @return true if calibration passes sanity checks
  bool validateCalibration() const;

private:
  ThermalCalibration calib_;

  /// Parse JSON calibration file
  /// @param lut_path Path to JSON file
  /// @return Parsed calibration structure, or std::nullopt on error
  std::optional<ThermalCalibration> parseCalibrationFile(
    const std::string& lut_path) const;

  /// Evaluate polynomial: P(dT) = c0 + c1*dT + c2*dT^2
  /// @param c0 Constant coefficient
  /// @param c1 Linear coefficient
  /// @param c2 Quadratic coefficient
  /// @param dT Temperature delta from reference (T - T_ref)
  /// @return Polynomial evaluation result
  template <typename T>
  T evaluatePolynomial(const T& c0, const T& c1, const T& c2, double dT) const {
    return c0 + c1 * dT + c2 * dT * dT;
  }
};

}  // namespace core
}  // namespace chimera
