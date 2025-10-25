// CHIMERA Calibration Persistence Framework
// Factory and field calibration with versioned storage
// Implements Phase 2 calibration requirements from ROADMAP.md

#pragma once

#include <string>
#include <map>
#include <optional>
#include <Eigen/Dense>
#include <ctime>
#include <fstream>
#include <sstream>

namespace chimera {
namespace utils {

// Calibration version for compatibility checking
constexpr const char* CALIB_VERSION = "1.0.0";

// Calibration types
enum class CalibrationType {
  FACTORY,      // Factory calibration (permanent)
  FIELD,        // Field calibration (user-adjustable)
  AUTO          // Auto-detected (runtime)
};

// IMU calibration
struct ImuCalibration {
  // Scale and misalignment (3x3 matrix)
  Eigen::Matrix3d accel_scale = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d gyro_scale = Eigen::Matrix3d::Identity();

  // Bias offsets
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();

  // Temperature compensation polynomials (3rd order)
  // bias(T) = c0 + c1*T + c2*T^2 + c3*T^3
  std::array<Eigen::Vector3d, 4> accel_temp_poly;
  std::array<Eigen::Vector3d, 4> gyro_temp_poly;

  // Calibration metadata
  std::time_t calibration_time = 0;
  std::string calibration_location;
  double temperature_ref = 25.0;  // Reference temperature (°C)
};

// Magnetometer calibration
struct MagCalibration {
  // Hard-iron offset (3D)
  Eigen::Vector3d hard_iron = Eigen::Vector3d::Zero();

  // Soft-iron matrix (3x3)
  Eigen::Matrix3d soft_iron = Eigen::Matrix3d::Identity();

  // Expected norm (calibrated at specific location)
  double reference_norm = 1.0;
  double reference_latitude = 0.0;
  double reference_longitude = 0.0;

  std::time_t calibration_time = 0;
};

// Barometer calibration
struct BaroCalibration {
  // Pressure offset (Pa)
  double pressure_offset = 0.0;

  // Temperature correction polynomial
  // altitude_correction(T) = c0 + c1*T + c2*T^2
  std::array<double, 3> temp_poly = {0.0, 0.0, 0.0};

  // "Tap to zero" field calibration
  double field_offset_agl = 0.0;  // Ground level offset
  std::time_t field_calib_time = 0;

  std::time_t calibration_time = 0;
};

// Camera/Optical Flow calibration
struct CameraCalibration {
  // Intrinsics (focal length, principal point)
  double fx = 500.0;
  double fy = 500.0;
  double cx = 320.0;
  double cy = 240.0;

  // Distortion coefficients (radial + tangential)
  std::array<double, 5> distortion = {0.0, 0.0, 0.0, 0.0, 0.0};

  // Extrinsics (camera to body frame)
  Eigen::Matrix4d T_body_camera = Eigen::Matrix4d::Identity();

  std::time_t calibration_time = 0;
};

// LiDAR/Range sensor calibration
struct LidarCalibration {
  // Range offset (meters)
  double range_offset = 0.0;

  // Scale factor (for unit conversion)
  double range_scale = 1.0;

  // Mounting angle (for angled LiDAR)
  double pitch_angle_deg = 0.0;  // Positive = pointing down

  std::time_t calibration_time = 0;
};

// Master calibration container
struct CalibrationData {
  std::string version = CALIB_VERSION;
  CalibrationType type = CalibrationType::FACTORY;

  // Sensor calibrations
  ImuCalibration imu;
  MagCalibration mag;
  BaroCalibration baro;
  CameraCalibration camera;
  LidarCalibration lidar;

  // Metadata
  std::string vehicle_id;
  std::string notes;
  std::time_t created_time = 0;
  std::time_t modified_time = 0;

  // Checksums for validation
  std::string checksum_imu;
  std::string checksum_mag;
  std::string checksum_baro;
};

// Calibration Manager
class CalibrationManager {
 public:
  CalibrationManager() = default;

  // Load calibration from file (JSON format)
  bool load(const std::string& filepath);

  // Save calibration to file
  bool save(const std::string& filepath) const;

  // Get/Set calibration data
  const CalibrationData& data() const { return data_; }
  CalibrationData& data() { return data_; }

  // Validation
  bool isValid() const;
  bool isStale(double max_age_days) const;
  bool checkVersion() const;

  // Field calibration helpers
  void updateBaroFieldOffset(double current_agl);
  void updateMagFieldCalibration(const std::vector<Eigen::Vector3d>& samples);

  // Apply calibrations to raw sensor data
  Eigen::Vector3d applyImuAccel(const Eigen::Vector3d& raw_accel, double temp = 25.0) const;
  Eigen::Vector3d applyImuGyro(const Eigen::Vector3d& raw_gyro, double temp = 25.0) const;
  Eigen::Vector3d applyMag(const Eigen::Vector3d& raw_mag) const;
  double applyBaro(double raw_altitude, double temp = 25.0) const;
  double applyLidar(double raw_range) const;

  // Generate checksum for data integrity
  static std::string computeChecksum(const std::string& data);

 private:
  CalibrationData data_;
  bool loaded_ = false;

  // JSON serialization helpers (simplified - use real JSON lib in production)
  std::string serializeToJSON() const;
  bool deserializeFromJSON(const std::string& json_str);

  // Temperature polynomial evaluation
  Eigen::Vector3d evalTempPoly(const std::array<Eigen::Vector3d, 4>& poly, double temp) const;
  double evalTempPolyScalar(const std::array<double, 3>& poly, double temp) const;
};

// Factory calibration defaults
inline CalibrationData createFactoryDefaults() {
  CalibrationData calib;
  calib.type = CalibrationType::FACTORY;
  calib.version = CALIB_VERSION;
  calib.created_time = std::time(nullptr);

  // IMU defaults (identity matrices, zero bias)
  calib.imu.accel_scale = Eigen::Matrix3d::Identity();
  calib.imu.gyro_scale = Eigen::Matrix3d::Identity();
  calib.imu.accel_bias = Eigen::Vector3d::Zero();
  calib.imu.gyro_bias = Eigen::Vector3d::Zero();

  // Zero temperature compensation
  for (int i = 0; i < 4; ++i) {
    calib.imu.accel_temp_poly[i] = Eigen::Vector3d::Zero();
    calib.imu.gyro_temp_poly[i] = Eigen::Vector3d::Zero();
  }

  // Mag defaults
  calib.mag.hard_iron = Eigen::Vector3d::Zero();
  calib.mag.soft_iron = Eigen::Matrix3d::Identity();
  calib.mag.reference_norm = 1.0;

  // Baro defaults
  calib.baro.pressure_offset = 0.0;
  calib.baro.temp_poly = {0.0, 0.0, 0.0};

  // Camera defaults (typical values)
  calib.camera.fx = 500.0;
  calib.camera.fy = 500.0;
  calib.camera.cx = 320.0;
  calib.camera.cy = 240.0;
  calib.camera.T_body_camera = Eigen::Matrix4d::Identity();

  // LiDAR defaults
  calib.lidar.range_offset = 0.0;
  calib.lidar.range_scale = 1.0;
  calib.lidar.pitch_angle_deg = 0.0;

  return calib;
}

}  // namespace utils
}  // namespace chimera
