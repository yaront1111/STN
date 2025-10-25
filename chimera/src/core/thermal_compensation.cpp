// Thermal Compensation Implementation
// Temperature-dependent sensor correction for CHIMERA

#include "thermal_compensation.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>

namespace chimera {
namespace core {

ThermalCompensation::ThermalCompensation(const std::string& lut_path) {
  if (!lut_path.empty()) {
    if (!loadCalibration(lut_path)) {
      std::cerr << "[ThermalComp] Failed to load calibration from: "
                << lut_path << "\n";
      std::cerr << "[ThermalComp] Falling back to NO thermal compensation\n";
    }
  } else {
    std::cout << "[ThermalComp] No LUT path provided, thermal compensation DISABLED\n";
  }
}

bool ThermalCompensation::loadCalibration(const std::string& lut_path) {
  auto calib_opt = parseCalibrationFile(lut_path);
  if (!calib_opt) {
    return false;
  }

  calib_ = *calib_opt;

  // Validate calibration
  if (!validateCalibration()) {
    std::cerr << "[ThermalComp] Calibration failed validation checks\n";
    calib_.valid = false;
    return false;
  }

  calib_.valid = true;
  std::cout << "[ThermalComp] Loaded thermal calibration v" << calib_.version
            << " (T_ref=" << calib_.reference_temp_c << "°C)\n";
  std::cout << "[ThermalComp]   IMU: " << calib_.imu_model << "\n";
  std::cout << "[ThermalComp]   Baro: " << calib_.baro_model << "\n";

  return true;
}

std::optional<ThermalCalibration> ThermalCompensation::parseCalibrationFile(
    const std::string& lut_path) const {
  try {
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(lut_path, pt);

    ThermalCalibration calib;

    // Metadata
    calib.version = pt.get<std::string>("version", "1.0.0");
    calib.imu_model = pt.get<std::string>("imu_model", "unknown");
    calib.baro_model = pt.get<std::string>("baro_model", "unknown");
    calib.reference_temp_c = pt.get<double>("reference_temp_c", 25.0);
    calib.calibration_timestamp_unix = pt.get<uint64_t>(
      "calibration_timestamp_unix", 0);

    // Gyro bias polynomials (rad/s and derivatives)
    if (auto gyro_node = pt.get_child_optional("gyro_bias")) {
      auto& g = *gyro_node;
      calib.gyro_bias_T0 = Eigen::Vector3d(
        g.get<double>("T0.x", 0.0),
        g.get<double>("T0.y", 0.0),
        g.get<double>("T0.z", 0.0));
      calib.gyro_bias_T1 = Eigen::Vector3d(
        g.get<double>("T1.x", 0.0),
        g.get<double>("T1.y", 0.0),
        g.get<double>("T1.z", 0.0));
      calib.gyro_bias_T2 = Eigen::Vector3d(
        g.get<double>("T2.x", 0.0),
        g.get<double>("T2.y", 0.0),
        g.get<double>("T2.z", 0.0));
    }

    // Accel bias polynomials (m/s² and derivatives)
    if (auto accel_node = pt.get_child_optional("accel_bias")) {
      auto& a = *accel_node;
      calib.accel_bias_T0 = Eigen::Vector3d(
        a.get<double>("T0.x", 0.0),
        a.get<double>("T0.y", 0.0),
        a.get<double>("T0.z", 0.0));
      calib.accel_bias_T1 = Eigen::Vector3d(
        a.get<double>("T1.x", 0.0),
        a.get<double>("T1.y", 0.0),
        a.get<double>("T1.z", 0.0));
      calib.accel_bias_T2 = Eigen::Vector3d(
        a.get<double>("T2.x", 0.0),
        a.get<double>("T2.y", 0.0),
        a.get<double>("T2.z", 0.0));
    }

    // Baro temperature coefficient (meters/°C)
    calib.baro_temp_coeff = pt.get<double>("baro_temp_coeff", 0.0);

    return calib;

  } catch (const std::exception& e) {
    std::cerr << "[ThermalComp] Error parsing calibration file: "
              << e.what() << "\n";
    return std::nullopt;
  }
}

Eigen::Vector3d ThermalCompensation::compensateGyroBias(double temp_c) const {
  if (!calib_.valid) {
    return Eigen::Vector3d::Zero();
  }

  double dT = temp_c - calib_.reference_temp_c;
  return evaluatePolynomial(
    calib_.gyro_bias_T0,
    calib_.gyro_bias_T1,
    calib_.gyro_bias_T2,
    dT);
}

Eigen::Vector3d ThermalCompensation::compensateAccelBias(double temp_c) const {
  if (!calib_.valid) {
    return Eigen::Vector3d::Zero();
  }

  double dT = temp_c - calib_.reference_temp_c;
  return evaluatePolynomial(
    calib_.accel_bias_T0,
    calib_.accel_bias_T1,
    calib_.accel_bias_T2,
    dT);
}

double ThermalCompensation::compensateBaroAltitude(
    double altitude_raw, double temp_c) const {
  if (!calib_.valid) {
    return altitude_raw;
  }

  double dT = temp_c - calib_.reference_temp_c;
  return altitude_raw + calib_.baro_temp_coeff * dT;
}

bool ThermalCompensation::validateCalibration() const {
  // Sanity checks on calibration values

  // 1. Reference temperature should be reasonable (-40°C to +80°C)
  if (calib_.reference_temp_c < -40.0 || calib_.reference_temp_c > 80.0) {
    std::cerr << "[ThermalComp] Invalid reference temperature: "
              << calib_.reference_temp_c << "°C\n";
    return false;
  }

  // 2. Gyro bias at T0 should be reasonable (< 0.1 rad/s = ~6 deg/s)
  if (calib_.gyro_bias_T0.norm() > 0.1) {
    std::cerr << "[ThermalComp] Suspiciously large gyro bias T0: "
              << calib_.gyro_bias_T0.norm() << " rad/s\n";
    return false;
  }

  // 3. Gyro bias linear coeff should be reasonable (< 0.01 rad/s/°C)
  if (calib_.gyro_bias_T1.norm() > 0.01) {
    std::cerr << "[ThermalComp] Suspiciously large gyro bias T1: "
              << calib_.gyro_bias_T1.norm() << " rad/s/°C\n";
    return false;
  }

  // 4. Accel bias at T0 should be reasonable (< 0.5 m/s²)
  if (calib_.accel_bias_T0.norm() > 0.5) {
    std::cerr << "[ThermalComp] Suspiciously large accel bias T0: "
              << calib_.accel_bias_T0.norm() << " m/s²\n";
    return false;
  }

  // 5. Accel bias linear coeff should be reasonable (< 0.05 m/s²/°C)
  if (calib_.accel_bias_T1.norm() > 0.05) {
    std::cerr << "[ThermalComp] Suspiciously large accel bias T1: "
              << calib_.accel_bias_T1.norm() << " m/s²/°C\n";
    return false;
  }

  // 6. Baro temp coeff should be reasonable (< 5 meters/°C)
  if (std::abs(calib_.baro_temp_coeff) > 5.0) {
    std::cerr << "[ThermalComp] Suspiciously large baro temp coeff: "
              << calib_.baro_temp_coeff << " m/°C\n";
    return false;
  }

  return true;
}

std::string ThermalCompensation::exportToJson() const {
  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"version\": \"" << calib_.version << "\",\n";
  oss << "  \"imu_model\": \"" << calib_.imu_model << "\",\n";
  oss << "  \"baro_model\": \"" << calib_.baro_model << "\",\n";
  oss << "  \"reference_temp_c\": " << calib_.reference_temp_c << ",\n";
  oss << "  \"calibration_timestamp_unix\": "
      << calib_.calibration_timestamp_unix << ",\n";

  oss << "  \"gyro_bias\": {\n";
  oss << "    \"T0\": {\"x\": " << calib_.gyro_bias_T0.x()
      << ", \"y\": " << calib_.gyro_bias_T0.y()
      << ", \"z\": " << calib_.gyro_bias_T0.z() << "},\n";
  oss << "    \"T1\": {\"x\": " << calib_.gyro_bias_T1.x()
      << ", \"y\": " << calib_.gyro_bias_T1.y()
      << ", \"z\": " << calib_.gyro_bias_T1.z() << "},\n";
  oss << "    \"T2\": {\"x\": " << calib_.gyro_bias_T2.x()
      << ", \"y\": " << calib_.gyro_bias_T2.y()
      << ", \"z\": " << calib_.gyro_bias_T2.z() << "}\n";
  oss << "  },\n";

  oss << "  \"accel_bias\": {\n";
  oss << "    \"T0\": {\"x\": " << calib_.accel_bias_T0.x()
      << ", \"y\": " << calib_.accel_bias_T0.y()
      << ", \"z\": " << calib_.accel_bias_T0.z() << "},\n";
  oss << "    \"T1\": {\"x\": " << calib_.accel_bias_T1.x()
      << ", \"y\": " << calib_.accel_bias_T1.y()
      << ", \"z\": " << calib_.accel_bias_T1.z() << "},\n";
  oss << "    \"T2\": {\"x\": " << calib_.accel_bias_T2.x()
      << ", \"y\": " << calib_.accel_bias_T2.y()
      << ", \"z\": " << calib_.accel_bias_T2.z() << "}\n";
  oss << "  },\n";

  oss << "  \"baro_temp_coeff\": " << calib_.baro_temp_coeff << ",\n";
  oss << "  \"valid\": " << (calib_.valid ? "true" : "false") << "\n";
  oss << "}\n";

  return oss.str();
}

}  // namespace core
}  // namespace chimera
