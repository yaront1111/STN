#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>

#include "core/fusion/sr_ukf.h"  // For IMUData

namespace aion {

/**
 * IMU calibration parameters
 */
struct IMUCalibration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Scale factor errors (multiplicative)
  Eigen::Vector3d gyro_scale{1.0, 1.0, 1.0};
  Eigen::Vector3d accel_scale{1.0, 1.0, 1.0};

  // Misalignment (small angle approx, rad)
  Eigen::Matrix3d gyro_misalignment{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d accel_misalignment{Eigen::Matrix3d::Identity()};

  // Static biases (additive)
  Eigen::Vector3d gyro_bias{0.0, 0.0, 0.0};    // rad/s
  Eigen::Vector3d accel_bias{0.0, 0.0, 0.0};   // m/s^2

  // Temperature compensation
  double reference_temp{25.0};  // Celsius
  Eigen::Vector3d gyro_temp_coeff{0.0, 0.0, 0.0};   // rad/s/C
  Eigen::Vector3d accel_temp_coeff{0.0, 0.0, 0.0};  // m/s^2/C

  // Noise parameters (for simulation/filtering)
  double gyro_noise_density{0.02 * M_PI / 180.0};    // rad/s/sqrt(Hz)
  double accel_noise_density{0.03};                  // m/s^2/sqrt(Hz)
  double gyro_bias_stability{3.0 / 3600.0 * M_PI / 180.0};  // rad/s (3 deg/hr)
  double accel_bias_stability{0.05};                 // m/s^2

  // Apply calibration to raw measurements
  void calibrate(IMUData& data) const;
};

/**
 * Abstract base class for IMU drivers
 */
class IMUDriver {
 public:
  using Callback = std::function<void(const IMUData&)>;

  IMUDriver() = default;
  virtual ~IMUDriver() = default;

  // Start/stop data acquisition
  virtual bool start() = 0;
  virtual void stop() = 0;
  virtual bool isRunning() const { return is_running_.load(); }

  // Set callback for new IMU data
  void setCallback(Callback cb) { callback_ = cb; }

  // Get/set calibration
  const IMUCalibration& getCalibration() const { return calibration_; }
  void setCalibration(const IMUCalibration& calib) { calibration_ = calib; }

  // Load calibration from file
  bool loadCalibration(const std::string& filepath);

  // Driver info
  virtual std::string getName() const = 0;
  virtual double getRate() const = 0;  // Expected Hz

  // Statistics
  struct Stats {
    uint64_t samples_received{0};
    uint64_t samples_dropped{0};
    double last_timestamp{0.0};
    double actual_rate_hz{0.0};
    double temperature_avg{25.0};
  };
  Stats getStats() const;

 protected:
  // Derived classes call this to publish data
  void publishIMU(const IMUData& raw_data);

  std::atomic<bool> is_running_{false};
  Callback callback_;
  IMUCalibration calibration_;

  // Statistics
  mutable std::mutex stats_mutex_;
  Stats stats_;
  double stats_start_time_{0.0};
  uint64_t stats_window_count_{0};
};

}  // namespace aion