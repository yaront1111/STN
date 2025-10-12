#include "core/fusion/imu_preintegration.h"
#include "core/utils/logging.h"

#include <cmath>

namespace aion {

IMUPreintegration::IMUPreintegration(const Params& params) : params_(params) {
  reset();

  // Setup noise matrix
  noise_.setZero();

  // IMU noise (continuous time)
  double ng2 = params_.gyro_noise_density * params_.gyro_noise_density;
  double na2 = params_.accel_noise_density * params_.accel_noise_density;
  noise_.block<3, 3>(0, 0) = ng2 * Eigen::Matrix3d::Identity();  // gyro noise
  noise_.block<3, 3>(3, 3) = na2 * Eigen::Matrix3d::Identity();  // accel noise

  // Bias random walk
  double nbg2 = params_.gyro_bias_rw * params_.gyro_bias_rw;
  double nba2 = params_.accel_bias_rw * params_.accel_bias_rw;
  noise_.block<3, 3>(6, 6) = nbg2 * Eigen::Matrix3d::Identity();   // gyro bias rw
  noise_.block<3, 3>(9, 9) = nba2 * Eigen::Matrix3d::Identity();   // accel bias rw
}

void IMUPreintegration::reset() {
  delta_position_ = Eigen::Vector3d::Zero();
  delta_velocity_ = Eigen::Vector3d::Zero();
  delta_rotation_ = Eigen::Quaterniond::Identity();
  delta_time_ = 0.0;

  linearized_bg_ = Eigen::Vector3d::Zero();
  linearized_ba_ = Eigen::Vector3d::Zero();

  J_p_bg_ = Eigen::Matrix3d::Zero();
  J_p_ba_ = Eigen::Matrix3d::Zero();
  J_v_bg_ = Eigen::Matrix3d::Zero();
  J_v_ba_ = Eigen::Matrix3d::Zero();
  J_R_bg_ = Eigen::Matrix3d::Zero();

  covariance_.setZero();

  measurements_.clear();
  dt_buffer_.clear();
}

void IMUPreintegration::addMeasurement(const IMUData& imu, double dt) {
  if (dt <= 0.0 || dt > 1.0) {
    LOG_WARN("Invalid dt in preintegration: {}", dt);
    return;
  }

  // Remove bias from measurements
  Eigen::Vector3d omega = imu.angular_velocity - linearized_bg_;
  Eigen::Vector3d acc = imu.acceleration - linearized_ba_;

  // Store measurement for potential re-integration
  measurements_.push_back(imu);
  dt_buffer_.push_back(dt);

  // Propagate preintegration
  propagate(omega, acc, dt);

  // Update Jacobians
  updateJacobians(omega, acc, dt);

  // Update time
  delta_time_ += dt;
}

void IMUPreintegration::propagate(const Eigen::Vector3d& omega,
                                  const Eigen::Vector3d& acc,
                                  double dt) {
  // Current rotation matrix
  Eigen::Matrix3d R_ij = delta_rotation_.toRotationMatrix();

  // Propagate rotation (exponential map)
  Eigen::Vector3d theta = omega * dt;
  Eigen::Matrix3d dR = expSO3(theta);
  delta_rotation_ = Eigen::Quaterniond(R_ij * dR);
  delta_rotation_.normalize();

  // Propagate velocity
  delta_velocity_ += R_ij * acc * dt;

  // Propagate position (with second-order term)
  delta_position_ += delta_velocity_ * dt + 0.5 * R_ij * acc * dt * dt;

  // Propagate covariance (continuous time)
  // State transition matrix F (15x15)
  Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();

  // Position derivatives
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();  // ∂p/∂v

  // Velocity derivatives
  F.block<3, 3>(3, 6) = -R_ij * skewSymmetric(acc);   // ∂v/∂θ
  F.block<3, 3>(3, 12) = -R_ij;                       // ∂v/∂ba

  // Rotation derivatives
  F.block<3, 3>(6, 6) = -skewSymmetric(omega);        // ∂θ/∂θ
  F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity(); // ∂θ/∂bg

  // Noise input matrix G (15x12)
  Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();
  G.block<3, 3>(3, 3) = -R_ij;                        // accel noise → velocity
  G.block<3, 3>(6, 0) = -Eigen::Matrix3d::Identity(); // gyro noise → rotation
  G.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();  // gyro bias noise
  G.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity(); // accel bias noise

  // Discrete time covariance update (first-order approximation)
  covariance_ = covariance_ + (F * covariance_ + covariance_ * F.transpose() +
                               G * noise_.block<12, 12>(0, 0) * G.transpose()) * dt;

  // Ensure symmetry
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

void IMUPreintegration::updateJacobians(const Eigen::Vector3d& omega,
                                        const Eigen::Vector3d& acc,
                                        double dt) {
  // Current rotation matrix
  Eigen::Matrix3d R_ij = delta_rotation_.toRotationMatrix();

  // Right Jacobian for rotation update
  Eigen::Vector3d theta = omega * dt;
  Eigen::Matrix3d Jr = rightJacobianSO3(theta);

  // Update Jacobians (first-order approximation)
  // See Forster et al. equations

  // Position Jacobian updates
  J_p_ba_ = J_p_ba_ + J_v_ba_ * dt - 0.5 * R_ij * dt * dt;
  J_p_bg_ = J_p_bg_ + J_v_bg_ * dt - 0.5 * R_ij * skewSymmetric(acc) * J_R_bg_ * dt * dt;

  // Velocity Jacobian updates
  J_v_ba_ = J_v_ba_ - R_ij * dt;
  J_v_bg_ = J_v_bg_ - R_ij * skewSymmetric(acc) * J_R_bg_ * dt;

  // Rotation Jacobian update
  J_R_bg_ = J_R_bg_ - Jr * dt;
}

void IMUPreintegration::correctBias(const Eigen::Vector3d& delta_bg,
                                    const Eigen::Vector3d& delta_ba) {
  // Apply first-order correction
  delta_position_ = getCorrectedDeltaPosition(delta_bg, delta_ba);
  delta_velocity_ = getCorrectedDeltaVelocity(delta_bg, delta_ba);
  delta_rotation_ = getCorrectedDeltaRotation(delta_bg);

  // Update linearization point
  linearized_bg_ += delta_bg;
  linearized_ba_ += delta_ba;
}

Eigen::Vector3d IMUPreintegration::getCorrectedDeltaPosition(
    const Eigen::Vector3d& delta_bg,
    const Eigen::Vector3d& delta_ba) const {
  return delta_position_ + J_p_bg_ * delta_bg + J_p_ba_ * delta_ba;
}

Eigen::Vector3d IMUPreintegration::getCorrectedDeltaVelocity(
    const Eigen::Vector3d& delta_bg,
    const Eigen::Vector3d& delta_ba) const {
  return delta_velocity_ + J_v_bg_ * delta_bg + J_v_ba_ * delta_ba;
}

Eigen::Quaterniond IMUPreintegration::getCorrectedDeltaRotation(
    const Eigen::Vector3d& delta_bg) const {
  // First-order approximation: ΔR_corrected ≈ ΔR * Exp(J_R_bg * δbg)
  Eigen::Vector3d dtheta = J_R_bg_ * delta_bg;
  Eigen::Matrix3d dR = expSO3(dtheta);
  return Eigen::Quaterniond(delta_rotation_.toRotationMatrix() * dR);
}

void IMUPreintegration::setLinearizedBias(const Eigen::Vector3d& bg,
                                          const Eigen::Vector3d& ba) {
  linearized_bg_ = bg;
  linearized_ba_ = ba;
}

Eigen::Matrix3d IMUPreintegration::expSO3(const Eigen::Vector3d& omega) {
  double theta = omega.norm();
  if (theta < 1e-6) {
    // First-order approximation for small angles
    return Eigen::Matrix3d::Identity() + skewSymmetric(omega);
  }

  Eigen::Vector3d k = omega / theta;
  Eigen::Matrix3d K = skewSymmetric(k);

  // Rodrigues' formula
  return Eigen::Matrix3d::Identity() +
         std::sin(theta) * K +
         (1.0 - std::cos(theta)) * K * K;
}

Eigen::Vector3d IMUPreintegration::logSO3(const Eigen::Matrix3d& R) {
  double trace = R.trace();
  Eigen::Vector3d omega;

  if (trace >= 3.0 - 1e-6) {
    // Close to identity - use first-order approximation
    omega << 0.5 * (R(2, 1) - R(1, 2)),
             0.5 * (R(0, 2) - R(2, 0)),
             0.5 * (R(1, 0) - R(0, 1));
  } else {
    double theta = std::acos(0.5 * (trace - 1.0));
    omega = (theta / (2.0 * std::sin(theta))) *
            Eigen::Vector3d(R(2, 1) - R(1, 2),
                           R(0, 2) - R(2, 0),
                           R(1, 0) - R(0, 1));
  }

  return omega;
}

Eigen::Matrix3d IMUPreintegration::rightJacobianSO3(const Eigen::Vector3d& omega) {
  double theta = omega.norm();

  if (theta < 1e-6) {
    // First-order approximation
    return Eigen::Matrix3d::Identity() - 0.5 * skewSymmetric(omega);
  }

  Eigen::Vector3d k = omega / theta;
  Eigen::Matrix3d K = skewSymmetric(k);

  // Right Jacobian of SO(3)
  return Eigen::Matrix3d::Identity() -
         ((1.0 - std::cos(theta)) / theta) * K +
         ((theta - std::sin(theta)) / theta) * K * K;
}

}  // namespace aion