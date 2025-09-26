#include "sensors/imu/imu_driver.h"
#include "core/utils/logging.h"
#include "core/utils/time_utils.h"
#include "core/utils/config.h"

#include <fstream>

namespace aion {

void IMUCalibration::calibrate(IMUData& data) const {
  // Temperature compensation
  double temp_delta = data.temperature - reference_temp;
  Eigen::Vector3d gyro_temp_correction = gyro_temp_coeff * temp_delta;
  Eigen::Vector3d accel_temp_correction = accel_temp_coeff * temp_delta;

  // Apply calibration chain:
  // 1. Remove static bias
  data.angular_velocity -= gyro_bias;
  data.acceleration -= accel_bias;

  // 2. Temperature compensation
  data.angular_velocity -= gyro_temp_correction;
  data.acceleration -= accel_temp_correction;

  // 3. Misalignment correction
  data.angular_velocity = gyro_misalignment * data.angular_velocity;
  data.acceleration = accel_misalignment * data.acceleration;

  // 4. Scale factor correction
  data.angular_velocity = data.angular_velocity.cwiseProduct(gyro_scale);
  data.acceleration = data.acceleration.cwiseProduct(accel_scale);
}

bool IMUDriver::loadCalibration(const std::string& filepath) {
  try {
    YAML::Node calib = YAML::LoadFile(filepath);

    if (calib["gyro_scale"]) {
      auto gs = calib["gyro_scale"].as<std::vector<double>>();
      if (gs.size() == 3) {
        calibration_.gyro_scale = Eigen::Vector3d(gs[0], gs[1], gs[2]);
      }
    }

    if (calib["accel_scale"]) {
      auto as = calib["accel_scale"].as<std::vector<double>>();
      if (as.size() == 3) {
        calibration_.accel_scale = Eigen::Vector3d(as[0], as[1], as[2]);
      }
    }

    if (calib["gyro_bias"]) {
      auto gb = calib["gyro_bias"].as<std::vector<double>>();
      if (gb.size() == 3) {
        // Convert deg/s to rad/s
        calibration_.gyro_bias = Eigen::Vector3d(
            gb[0] * M_PI / 180.0,
            gb[1] * M_PI / 180.0,
            gb[2] * M_PI / 180.0);
      }
    }

    if (calib["accel_bias"]) {
      auto ab = calib["accel_bias"].as<std::vector<double>>();
      if (ab.size() == 3) {
        calibration_.accel_bias = Eigen::Vector3d(ab[0], ab[1], ab[2]);
      }
    }

    if (calib["reference_temp"]) {
      calibration_.reference_temp = calib["reference_temp"].as<double>();
    }

    if (calib["gyro_noise_density"]) {
      // Convert deg/s/sqrt(Hz) to rad/s/sqrt(Hz)
      calibration_.gyro_noise_density =
          calib["gyro_noise_density"].as<double>() * M_PI / 180.0;
    }

    if (calib["accel_noise_density"]) {
      calibration_.accel_noise_density = calib["accel_noise_density"].as<double>();
    }

    if (calib["gyro_bias_stability"]) {
      // Convert deg/hr to rad/s
      calibration_.gyro_bias_stability =
          calib["gyro_bias_stability"].as<double>() / 3600.0 * M_PI / 180.0;
    }

    if (calib["accel_bias_stability"]) {
      calibration_.accel_bias_stability = calib["accel_bias_stability"].as<double>();
    }

    LOG_INFO("Loaded IMU calibration from: {}", filepath);
    return true;

  } catch (const std::exception& e) {
    LOG_ERROR("Failed to load IMU calibration from {}: {}", filepath, e.what());
    return false;
  }
}

IMUDriver::Stats IMUDriver::getStats() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  Stats current_stats = stats_;

  // Calculate actual rate
  if (stats_window_count_ > 0 && stats_.last_timestamp > stats_start_time_) {
    double duration = stats_.last_timestamp - stats_start_time_;
    current_stats.actual_rate_hz = stats_window_count_ / duration;
  }

  return current_stats;
}

void IMUDriver::publishIMU(const IMUData& raw_data) {
  // Apply calibration
  IMUData calibrated = raw_data;
  calibration_.calibrate(calibrated);

  // Update statistics
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.samples_received++;
    stats_.last_timestamp = calibrated.timestamp;
    stats_.temperature_avg =
        (stats_.temperature_avg * (stats_.samples_received - 1) +
         calibrated.temperature) / stats_.samples_received;

    // Track rate calculation window
    if (stats_window_count_ == 0) {
      stats_start_time_ = calibrated.timestamp;
    }
    stats_window_count_++;

    // Reset window every 1000 samples for rate calculation
    if (stats_window_count_ >= 1000) {
      double duration = calibrated.timestamp - stats_start_time_;
      if (duration > 0) {
        stats_.actual_rate_hz = stats_window_count_ / duration;
      }
      stats_window_count_ = 0;
    }
  }

  // Call user callback
  if (callback_) {
    callback_(calibrated);
  }
}

}  // namespace aion