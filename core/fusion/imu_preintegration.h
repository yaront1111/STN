#pragma once

#include <Eigen/Dense>
#include <vector>

#include "core/fusion/sr_ukf.h"  // For IMUData

namespace aion {

/**
 * IMU Preintegration for factor graphs
 * Based on Forster et al. "On-Manifold Preintegration for Real-Time
 * Visual-Inertial Odometry"
 *
 * Accumulates IMU measurements between keyframes to create a single
 * relative motion constraint with uncertainty.
 */
class IMUPreintegration {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Params {
    double gyro_noise_density;    // rad/s/sqrt(Hz)
    double accel_noise_density;   // m/s^2/sqrt(Hz)
    double gyro_bias_rw;          // rad/s/sqrt(s)
    double accel_bias_rw;         // m/s^2/sqrt(s)
    Eigen::Vector3d gravity;      // NED gravity

    Params()
        : gyro_noise_density(0.02 * M_PI / 180.0),
          accel_noise_density(0.03),
          gyro_bias_rw(1e-5),
          accel_bias_rw(1e-4),
          gravity(0.0, 0.0, 9.80665) {}
  };

  explicit IMUPreintegration(const Params& params = Params());

  // Reset preintegration (start new interval)
  void reset();

  // Add an IMU measurement
  void addMeasurement(const IMUData& imu, double dt);

  // Get preintegrated values (delta from t_i to t_j)
  const Eigen::Vector3d& getDeltaPosition() const { return delta_position_; }
  const Eigen::Vector3d& getDeltaVelocity() const { return delta_velocity_; }
  const Eigen::Quaterniond& getDeltaRotation() const { return delta_rotation_; }
  double getDeltaTime() const { return delta_time_; }

  // Get covariance of preintegrated measurements (15x15)
  const Eigen::Matrix<double, 15, 15>& getCovariance() const { return covariance_; }

  // Get Jacobians for bias correction (avoid re-integration)
  const Eigen::Matrix3d& getPositionJacobianGyro() const { return J_p_bg_; }
  const Eigen::Matrix3d& getPositionJacobianAccel() const { return J_p_ba_; }
  const Eigen::Matrix3d& getVelocityJacobianGyro() const { return J_v_bg_; }
  const Eigen::Matrix3d& getVelocityJacobianAccel() const { return J_v_ba_; }
  const Eigen::Matrix3d& getRotationJacobianGyro() const { return J_R_bg_; }

  // Apply bias correction (first-order)
  void correctBias(const Eigen::Vector3d& delta_bg,
                   const Eigen::Vector3d& delta_ba);

  // Get corrected values after bias update
  Eigen::Vector3d getCorrectedDeltaPosition(const Eigen::Vector3d& delta_bg,
                                            const Eigen::Vector3d& delta_ba) const;
  Eigen::Vector3d getCorrectedDeltaVelocity(const Eigen::Vector3d& delta_bg,
                                            const Eigen::Vector3d& delta_ba) const;
  Eigen::Quaterniond getCorrectedDeltaRotation(const Eigen::Vector3d& delta_bg) const;

  // Get linearized bias values used during preintegration
  const Eigen::Vector3d& getLinearizedGyroBias() const { return linearized_bg_; }
  const Eigen::Vector3d& getLinearizedAccelBias() const { return linearized_ba_; }
  void setLinearizedBias(const Eigen::Vector3d& bg, const Eigen::Vector3d& ba);

  // Statistics
  size_t getMeasurementCount() const { return measurements_.size(); }
  const std::vector<IMUData>& getMeasurements() const { return measurements_; }

  // Utility: SO(3) exponential map
  static Eigen::Matrix3d expSO3(const Eigen::Vector3d& omega);
  static Eigen::Vector3d logSO3(const Eigen::Matrix3d& R);

  // Right Jacobian of SO(3)
  static Eigen::Matrix3d rightJacobianSO3(const Eigen::Vector3d& omega);

  // Helper: skew-symmetric matrix
  static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d M;
    M << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return M;
  }

 private:
  // Propagate state and covariance
  void propagate(const Eigen::Vector3d& omega, const Eigen::Vector3d& acc, double dt);

  // Update Jacobians
  void updateJacobians(const Eigen::Vector3d& omega, const Eigen::Vector3d& acc, double dt);

  Params params_;

  // Preintegrated values (relative to frame i)
  Eigen::Vector3d delta_position_;
  Eigen::Vector3d delta_velocity_;
  Eigen::Quaterniond delta_rotation_;
  double delta_time_;

  // Linearization point for bias
  Eigen::Vector3d linearized_bg_;
  Eigen::Vector3d linearized_ba_;

  // Jacobians w.r.t bias (for correction without re-integration)
  Eigen::Matrix3d J_p_bg_;  // ∂Δp/∂b_g
  Eigen::Matrix3d J_p_ba_;  // ∂Δp/∂b_a
  Eigen::Matrix3d J_v_bg_;  // ∂Δv/∂b_g
  Eigen::Matrix3d J_v_ba_;  // ∂Δv/∂b_a
  Eigen::Matrix3d J_R_bg_;  // ∂ΔR/∂b_g

  // Covariance matrix (15x15: position, velocity, rotation, bg, ba)
  Eigen::Matrix<double, 15, 15> covariance_;

  // Noise matrix (continuous time)
  Eigen::Matrix<double, 18, 18> noise_;  // 6 for IMU noise, 12 for bias random walk

  // Stored measurements (for potential re-integration)
  std::vector<IMUData> measurements_;
  std::vector<double> dt_buffer_;
};

}  // namespace aion