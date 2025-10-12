#include "core/fusion/process_model.h"

#include <cmath>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include "core/fusion/state.h"
#include "core/fusion/sr_ukf.h"  // for IMUData definition

namespace aion {

ProcessModel::ProcessModel(double gravity) : gravity_(gravity) {}

void ProcessModel::propagate(State& state, const IMUData& imu, double dt) {
  if (dt <= 0.0 || dt > 1.0) {
    return;  // guard invalid dt
  }

  // Bias-compensated body-frame signals
  Eigen::Vector3d omega_b = imu.angular_velocity - state.gyro_bias;   // rad/s
  Eigen::Vector3d acc_b   = imu.acceleration     - state.accel_bias;  // m/s^2

  // Apply accel scale factors (diag)
  acc_b = acc_b.cwiseProduct(state.accel_scale);

  // Integrals
  Eigen::Vector3d dtheta = omega_b * dt;
  Eigen::Vector3d dv_b   = acc_b   * dt;

  if (!first_imu_) {
    dtheta += coningCompensation(omega_b, prev_omega_, dt);
    dv_b   += scullingCompensation(acc_b, omega_b, prev_accel_, prev_omega_, dt);
  }

  // Attitude update (q_nb, NED->body): q_nb_new = q_nb_old ⊗ Exp(dtheta)  [RIGHT multiply]
  const double n = dtheta.norm();
  if (n > 0.0) {
    Eigen::Quaterniond dq(Eigen::AngleAxisd(n, dtheta / n));
    state.quaternion = state.quaternion * dq;   // RIGHT-multiply (q_nb)
    state.quaternion.normalize();
  }

  // Specific force to NED: a_n = R_bn * f_b + g_n   (R_bn = body->nav)
  const Eigen::Vector3d a_ned = state.R_bn() * acc_b + getGravity();

  // Monitor residual acceleration for attitude estimation health
  static double last_warn_time = 0.0;
  const double current_time = state.time;
  const double throttle_period = 1.0; // WARN at most once per second

  const double a_norm = a_ned.norm();
  const double a_z_abs = std::abs(a_ned.z());

  // Only warn for genuinely concerning accelerations with throttling
  if ((a_norm > 2.0 || a_z_abs > 0.5) && (current_time - last_warn_time > throttle_period)) {
    // Calculate approximate tilt error from residual acceleration
    const double tilt_err_deg = std::asin(std::min(a_norm / 9.81, 1.0)) * 180.0 / M_PI;
    spdlog::warn("UKF: Residual accel detected - a_ned=[{:.3f},{:.3f},{:.3f}] m/s², "
                 "‖a‖={:.3f} m/s², est. tilt error ≈{:.1f}°",
                 a_ned.x(), a_ned.y(), a_ned.z(), a_norm, tilt_err_deg);
    last_warn_time = current_time;
  }

  // Velocity and position (simple constant-accel integration)
  state.velocity += a_ned * dt;
  state.position += state.velocity * dt + 0.5 * a_ned * dt * dt;

  // Save for next step
  prev_omega_ = omega_b;
  prev_accel_ = acc_b;
  first_imu_  = false;
}

Eigen::MatrixXd ProcessModel::getProcessNoise(double dt) const {
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(ErrorState::DIM, ErrorState::DIM);

  // Very simple CT → DT mapping to keep P from becoming too “sharp”
  const double q_p  = 0.25 * accel_noise_density_ * accel_noise_density_ * dt * dt; // pos
  const double q_v  =        accel_noise_density_ * accel_noise_density_ * dt;      // vel
  const double q_th =        gyro_noise_density_  * gyro_noise_density_  * dt;      // att
  const double q_bg =        gyro_bias_noise_density_  * gyro_bias_noise_density_  * dt;
  const double q_ba =        accel_bias_noise_density_ * accel_bias_noise_density_ * dt;
  const double q_w  =        wind_noise_density_       * wind_noise_density_       * dt;

  Q.block<3,3>(0,0).diagonal().setConstant(q_p);
  Q.block<3,3>(3,3).diagonal().setConstant(q_v);
  Q.block<3,3>(6,6).diagonal().setConstant(q_th);
  Q.block<3,3>(9,9).diagonal().setConstant(q_bg);
  Q.block<3,3>(12,12).diagonal().setConstant(q_ba);
  Q.block<2,2>(15,15).diagonal().setConstant(q_w);

  return Q;
}

void ProcessModel::setGyroNoiseDensity(double noise)      { gyro_noise_density_ = noise; }
void ProcessModel::setAccelNoiseDensity(double noise)     { accel_noise_density_ = noise; }
void ProcessModel::setGyroBiasNoiseDensity(double noise)  { gyro_bias_noise_density_ = noise; }
void ProcessModel::setAccelBiasNoiseDensity(double noise) { accel_bias_noise_density_ = noise; }
void ProcessModel::setWindNoiseDensity(double noise)      { wind_noise_density_ = noise; }

Eigen::Vector3d ProcessModel::getGravity() const {
  // NED: positive down along +Z
  return Eigen::Vector3d(0.0, 0.0, gravity_);
}

Eigen::Vector3d ProcessModel::coningCompensation(const Eigen::Vector3d& omega,
                                                 const Eigen::Vector3d& prev_omega,
                                                 double dt) const {
  // δθ_coning ≈ (1/12) * (ω_{k-1} × ω_k) * dt^2
  return (1.0 / 12.0) * prev_omega.cross(omega) * dt * dt;
}

Eigen::Vector3d ProcessModel::scullingCompensation(const Eigen::Vector3d& accel,
                                                   const Eigen::Vector3d& omega,
                                                   const Eigen::Vector3d& prev_accel,
                                                   const Eigen::Vector3d& prev_omega,
                                                   double dt) const {
  // δv_sculling ≈ (1/12) * (ω_{k-1} × a_k + a_{k-1} × ω_k) * dt^2
  return (1.0 / 12.0) * (prev_omega.cross(accel) + prev_accel.cross(omega)) * dt * dt;
}

}  // namespace aion
