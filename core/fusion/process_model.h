#pragma once

#include <Eigen/Dense>

#include "core/fusion/state.h"

namespace aion {

// Forward-declare to avoid circular include with sr_ukf.h
struct IMUData;

/**
 * Process model for INS mechanization (body → NED).
 * Implements strapdown with simple coning/sculling compensation.
 */
class ProcessModel {
 public:
  explicit ProcessModel(double gravity = 9.80665);

  // Propagate state using one IMU sample over dt
  void propagate(State& state, const IMUData& imu, double dt);

  // Optional: continuous-time noise → discrete Q(d t) (error-state order)
  Eigen::MatrixXd getProcessNoise(double dt) const;

  // Noise density setters (per √Hz)
  void setGyroNoiseDensity(double noise);      // rad/s/√Hz
  void setAccelNoiseDensity(double noise);     // m/s^2/√Hz
  void setGyroBiasNoiseDensity(double noise);  // rad/s^2/√Hz
  void setAccelBiasNoiseDensity(double noise); // m/s^3/√Hz
  void setWindNoiseDensity(double noise);      // m/s^2/√Hz

  // Gravity (NED, positive down)
  Eigen::Vector3d getGravity() const;

 private:
  // 2nd-order coning compensation
  Eigen::Vector3d coningCompensation(const Eigen::Vector3d& omega,
                                     const Eigen::Vector3d& prev_omega,
                                     double dt) const;

  // 2nd-order sculling compensation
  Eigen::Vector3d scullingCompensation(const Eigen::Vector3d& accel,
                                       const Eigen::Vector3d& omega,
                                       const Eigen::Vector3d& prev_accel,
                                       const Eigen::Vector3d& prev_omega,
                                       double dt) const;

  // Gravity magnitude (replace with WGS84 later)
  double gravity_;  // m/s^2

  // Process noise densities (per √Hz)
  double gyro_noise_density_       = 0.02;   // rad/s/√Hz
  double accel_noise_density_      = 0.03;   // m/s^2/√Hz
  double gyro_bias_noise_density_  = 5e-4;   // rad/s^2/√Hz
  double accel_bias_noise_density_ = 5e-4;   // m/s^3/√Hz
  double wind_noise_density_       = 0.1;    // m/s^2/√Hz

  // Previous IMU for compensation
  Eigen::Vector3d prev_omega_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d prev_accel_{Eigen::Vector3d::Zero()};
  bool first_imu_ = true;
};

}  // namespace aion
