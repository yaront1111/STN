// CHIMERA Calibration Persistence Implementation

#include "utils/calibration.h"
#include <cmath>
#include <iostream>
#include <sstream>

namespace chimera {
namespace utils {

bool CalibrationManager::load(const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "[Calibration] Failed to open: " << filepath << "\n";
    return false;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string json_str = buffer.str();

  if (!deserializeFromJSON(json_str)) {
    std::cerr << "[Calibration] Failed to parse JSON\n";
    return false;
  }

  loaded_ = true;
  std::cout << "[Calibration] Loaded from " << filepath
            << " (version " << data_.version << ")\n";
  return true;
}

bool CalibrationManager::save(const std::string& filepath) const {
  std::string json_str = serializeToJSON();

  std::ofstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "[Calibration] Failed to write: " << filepath << "\n";
    return false;
  }

  file << json_str;
  file.close();

  std::cout << "[Calibration] Saved to " << filepath << "\n";
  return true;
}

bool CalibrationManager::isValid() const {
  // Check version compatibility
  if (!checkVersion()) return false;

  // Verify checksums (placeholder - implement with real hash)
  // TODO: Implement SHA256 checksums

  return loaded_;
}

bool CalibrationManager::isStale(double max_age_days) const {
  if (data_.modified_time == 0) return true;

  std::time_t now = std::time(nullptr);
  double age_days = difftime(now, data_.modified_time) / (24.0 * 3600.0);

  return age_days > max_age_days;
}

bool CalibrationManager::checkVersion() const {
  return data_.version == CALIB_VERSION;
}

void CalibrationManager::updateBaroFieldOffset(double current_agl) {
  data_.baro.field_offset_agl = current_agl;
  data_.baro.field_calib_time = std::time(nullptr);
  data_.modified_time = std::time(nullptr);

  std::cout << "[Calibration] Baro field offset updated: " << current_agl << " m AGL\n";
}

void CalibrationManager::updateMagFieldCalibration(
    const std::vector<Eigen::Vector3d>& samples) {
  if (samples.size() < 100) {
    std::cerr << "[Calibration] Insufficient mag samples for calibration\n";
    return;
  }

  // Simple hard-iron calibration: average of min and max
  Eigen::Vector3d min_val = samples[0];
  Eigen::Vector3d max_val = samples[0];

  for (const auto& s : samples) {
    min_val = min_val.cwiseMin(s);
    max_val = max_val.cwiseMax(s);
  }

  data_.mag.hard_iron = (min_val + max_val) / 2.0;
  data_.mag.calibration_time = std::time(nullptr);
  data_.modified_time = std::time(nullptr);

  std::cout << "[Calibration] Mag hard-iron updated: ["
            << data_.mag.hard_iron.transpose() << "]\n";
}

Eigen::Vector3d CalibrationManager::applyImuAccel(
    const Eigen::Vector3d& raw_accel, double temp) const {
  // Apply scale and bias
  Eigen::Vector3d corrected = data_.imu.accel_scale * raw_accel - data_.imu.accel_bias;

  // Apply temperature compensation
  Eigen::Vector3d temp_correction = evalTempPoly(data_.imu.accel_temp_poly, temp);
  corrected -= temp_correction;

  return corrected;
}

Eigen::Vector3d CalibrationManager::applyImuGyro(
    const Eigen::Vector3d& raw_gyro, double temp) const {
  Eigen::Vector3d corrected = data_.imu.gyro_scale * raw_gyro - data_.imu.gyro_bias;

  Eigen::Vector3d temp_correction = evalTempPoly(data_.imu.gyro_temp_poly, temp);
  corrected -= temp_correction;

  return corrected;
}

Eigen::Vector3d CalibrationManager::applyMag(const Eigen::Vector3d& raw_mag) const {
  // Apply hard-iron and soft-iron correction
  Eigen::Vector3d corrected = raw_mag - data_.mag.hard_iron;
  corrected = data_.mag.soft_iron * corrected;
  return corrected;
}

double CalibrationManager::applyBaro(double raw_altitude, double temp) const {
  double corrected = raw_altitude + data_.baro.pressure_offset;

  // Apply temperature correction
  double temp_correction = evalTempPolyScalar(data_.baro.temp_poly, temp);
  corrected += temp_correction;

  // Apply field offset
  corrected += data_.baro.field_offset_agl;

  return corrected;
}

double CalibrationManager::applyLidar(double raw_range) const {
  double corrected = raw_range * data_.lidar.range_scale + data_.lidar.range_offset;

  // Apply pitch angle correction if needed
  if (std::abs(data_.lidar.pitch_angle_deg) > 0.1) {
    double pitch_rad = data_.lidar.pitch_angle_deg * M_PI / 180.0;
    corrected *= std::cos(pitch_rad);
  }

  return corrected;
}

std::string CalibrationManager::computeChecksum(const std::string& data) {
  // Placeholder: Use SHA256 in production
  // For now, return simple hash
  std::size_t h = std::hash<std::string>{}(data);
  std::stringstream ss;
  ss << std::hex << h;
  return ss.str();
}

std::string CalibrationManager::serializeToJSON() const {
  // Simplified JSON serialization (use real JSON library in production)
  std::stringstream json;

  json << "{\n";
  json << "  \"version\": \"" << data_.version << "\",\n";
  json << "  \"type\": " << static_cast<int>(data_.type) << ",\n";
  json << "  \"vehicle_id\": \"" << data_.vehicle_id << "\",\n";

  // IMU
  json << "  \"imu\": {\n";
  json << "    \"accel_bias\": [" << data_.imu.accel_bias.transpose() << "],\n";
  json << "    \"gyro_bias\": [" << data_.imu.gyro_bias.transpose() << "]\n";
  json << "  },\n";

  // Mag
  json << "  \"mag\": {\n";
  json << "    \"hard_iron\": [" << data_.mag.hard_iron.transpose() << "],\n";
  json << "    \"reference_norm\": " << data_.mag.reference_norm << "\n";
  json << "  },\n";

  // Baro
  json << "  \"baro\": {\n";
  json << "    \"field_offset_agl\": " << data_.baro.field_offset_agl << "\n";
  json << "  }\n";

  json << "}\n";

  return json.str();
}

bool CalibrationManager::deserializeFromJSON(const std::string& json_str) {
  // Simplified parser (use real JSON library in production)
  // This is a placeholder that creates factory defaults
  data_ = createFactoryDefaults();
  return true;
}

Eigen::Vector3d CalibrationManager::evalTempPoly(
    const std::array<Eigen::Vector3d, 4>& poly, double temp) const {
  double dT = temp - data_.imu.temperature_ref;
  return poly[0] + poly[1] * dT + poly[2] * dT * dT + poly[3] * dT * dT * dT;
}

double CalibrationManager::evalTempPolyScalar(
    const std::array<double, 3>& poly, double temp) const {
  double dT = temp - 25.0;  // Reference temperature
  return poly[0] + poly[1] * dT + poly[2] * dT * dT;
}

}  // namespace utils
}  // namespace chimera
