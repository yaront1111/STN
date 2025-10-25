#include "core/ukf.h"
#include "core/thermal_compensation.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SO3.h>
#include <cmath>

namespace chimera {
namespace core {

UKF::UKF(const NoiseParams& noise, double gravity_magnitude,
         const std::string& thermal_lut_path)
    : velocity_(Eigen::Vector3d::Zero()),
      gyro_bias_(Eigen::Vector3d::Zero()),
      accel_bias_(Eigen::Vector3d::Zero()),
      timestamp_(0.0),
      initialized_(false),
      noise_(noise),
      gravity_magnitude_(gravity_magnitude),
      gravity_vector_(0.0, 0.0, gravity_magnitude),  // NED: Down-positive
      alpha_(1e-3),
      beta_(2.0),
      kappa_(0.0) {
  // Initialize thermal compensation
  thermal_comp_ = std::make_unique<ThermalCompensation>(thermal_lut_path);
  // Error-state dimension
  constexpr int n = 15;
  lambda_ = alpha_ * alpha_ * (n + kappa_) - n;

  // Unscented weights
  weights_mean_.setZero();
  weights_cov_.setZero();
  weights_mean_(0) = lambda_ / (n + lambda_);
  weights_cov_(0)  = lambda_ / (n + lambda_) + (1 - alpha_ * alpha_ + beta_);
  for (int i = 1; i < 2 * n + 1; ++i) {
    const double w = 1.0 / (2.0 * (n + lambda_));
    weights_mean_(i) = w;
    weights_cov_(i)  = w;
  }

  // Covariance sqrt init (overwritten in initialize)
  cov_sqrt_.setIdentity();
}

UKF::~UKF() = default;

void UKF::initialize(const gtsam::Pose3& pose, const Eigen::Vector3d& velocity,
                     const Eigen::Matrix<double, 15, 15>& initial_cov) {
  pose_ = pose;
  velocity_ = velocity;
  gyro_bias_.setZero();
  accel_bias_.setZero();

  // Square-root of initial covariance (Cholesky), with gentle regularization fallback
  Eigen::LLT<Eigen::Matrix<double, 15, 15>> llt(initial_cov);
  if (llt.info() == Eigen::Success) {
    cov_sqrt_ = llt.matrixL();
  } else {
    Eigen::Matrix<double, 15, 15> reg = initial_cov;
    reg += 1e-6 * Eigen::Matrix<double, 15, 15>::Identity();
    cov_sqrt_ = reg.llt().matrixL();
  }

  timestamp_ = 0.0;
  initialized_ = true;
}

void UKF::propagate(const ImuMeasurement& imu) {
  if (!initialized_) return;

  const double dt = imu.timestamp - timestamp_;
  if (dt <= 0.0 || dt > 0.1) {  // guard against time glitches
    timestamp_ = imu.timestamp;
    return;
  }

  // ---- (0) Temperature compensation on nominal biases (hook preserved) ----
  auto [bg_nom, ba_nom] = applyThermalCorrection(gyro_bias_, accel_bias_, imu.temperature);

  // ---- (1) Form discrete process noise Q(dt) and add BEFORE UT ----
  Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
  const double an2 = noise_.accel_noise_density * noise_.accel_noise_density;
  const double gn2 = noise_.gyro_noise_density  * noise_.gyro_noise_density;
  const double ab2 = noise_.accel_random_walk   * noise_.accel_random_walk;
  const double gb2 = noise_.gyro_random_walk    * noise_.gyro_random_walk;

  // Pragmatic discrete scalings for UAV-Lite; tune as needed
  Q.block<3,3>(0,0)   = (an2 * dt * dt) * Eigen::Matrix3d::Identity(); // position ~ 1/2 a dt^2
  Q.block<3,3>(3,3)   = (an2 * dt)      * Eigen::Matrix3d::Identity(); // velocity ~ a dt
  Q.block<3,3>(6,6)   = (gn2 * dt)      * Eigen::Matrix3d::Identity(); // rotation ~ gyro noise
  Q.block<3,3>(9,9)   = (gb2 * dt)      * Eigen::Matrix3d::Identity(); // gyro bias RW
  Q.block<3,3>(12,12) = (ab2 * dt)      * Eigen::Matrix3d::Identity(); // accel bias RW

  Eigen::Matrix<double, 15, 15> P_prior = cov_sqrt_ * cov_sqrt_.transpose() + Q;
  Eigen::LLT<Eigen::Matrix<double, 15, 15>> llt(P_prior);
  if (llt.info() != Eigen::Success) {
    P_prior += 1e-6 * Eigen::Matrix<double, 15, 15>::Identity();
  }
  const Eigen::Matrix<double, 15, 15> P_prior_sqrt = P_prior.llt().matrixL();

  // ---- (2) Generate ERROR-STATE sigma points from P_prior ----
  Eigen::Matrix<double, 15, 31> sigma_points = generateSigmaPoints(P_prior_sqrt);

  // ---- (3) Propagate error-state sigmas with LINEARIZED dynamics around the nominal ----
  const gtsam::Rot3 R_nom = pose_.rotation();
  const Eigen::Vector3d omega_b = imu.gyro  - bg_nom; // bias-corrected for linearization
  const Eigen::Vector3d f_b     = imu.accel - ba_nom; // specific force (bias-corrected)

  Eigen::Matrix<double, 15, 31> propagated_points;
  const Eigen::Matrix3d R = R_nom.matrix();
  const Eigen::Matrix3d f_hat = gtsam::SO3::Hat(f_b);
  const Eigen::Matrix3d w_hat = gtsam::SO3::Hat(omega_b);

  for (int i = 0; i < 31; ++i) {
    const Eigen::Vector3d dp  = sigma_points.col(i).segment<3>(0);
    const Eigen::Vector3d dv  = sigma_points.col(i).segment<3>(3);
    const Eigen::Vector3d dth = sigma_points.col(i).segment<3>(6);
    const Eigen::Vector3d dbg = sigma_points.col(i).segment<3>(9);
    const Eigen::Vector3d dba = sigma_points.col(i).segment<3>(12);

    Eigen::Matrix<double, 15, 1> dxn;
    // d(dp)/dt   = dv
    dxn.segment<3>(0) = dp + dv * dt;
    // d(dv)/dt   = -R * [f_b]_x * dth - R * dba
    dxn.segment<3>(3) = dv + (-R * f_hat * dth - R * dba) * dt;
    // d(dth)/dt  = -[omega_b]_x * dth - dbg
    dxn.segment<3>(6) = dth + (-w_hat * dth - dbg) * dt;
    // Bias deltas (mean) remain (random-walk mean = 0)
    dxn.segment<3>(9)  = dbg;
    dxn.segment<3>(12) = dba;

    propagated_points.col(i) = dxn;
  }

  // ---- (4) Recover mean error and new covariance sqrt from propagated error sigmas ----
  Eigen::Matrix<double, 15, 1> mean_delta;
  computeMeanAndCovariance(propagated_points, mean_delta, cov_sqrt_);

  // ---- (5) Integrate the NOMINAL state with full strapdown physics ----
  {
    // Use *current* (pre-correction) nominal biases for propagation this step
    const Eigen::Vector3d omega_b_nom = imu.gyro  - gyro_bias_;
    const Eigen::Vector3d f_b_nom     = imu.accel - accel_bias_;

    // Attitude update (SO(3))
    const gtsam::Rot3 R_k  = pose_.rotation();
    const gtsam::Rot3 dR   = gtsam::Rot3::Expmap(omega_b_nom * dt);
    const gtsam::Rot3 R_k1 = R_k * dR;

    // World acceleration and kinematics
    const Eigen::Vector3d a_w  = R_k.matrix() * f_b_nom + gravity_vector_;
    const Eigen::Vector3d v_k1 = velocity_ + a_w * dt;
    const gtsam::Point3   p_k1 = pose_.translation() + velocity_ * dt + 0.5 * a_w * dt * dt;

    pose_     = gtsam::Pose3(R_k1, p_k1);
    velocity_ = v_k1;
  }

  // ---- (6) Apply small error correction to NEW nominal ----
  // Position
  pose_ = gtsam::Pose3(pose_.rotation(),
                       pose_.translation() + mean_delta.segment<3>(0));
  // Velocity
  velocity_ += mean_delta.segment<3>(3);
  // Rotation (SO(3) small-angle)
  const Eigen::Vector3d dtheta = mean_delta.segment<3>(6);
  if (dtheta.norm() > 1e-10) {
    const gtsam::Rot3 dR = gtsam::Rot3::Expmap(dtheta);
    pose_ = gtsam::Pose3(pose_.rotation() * dR, pose_.translation());
  }
  // Biases
  gyro_bias_  += mean_delta.segment<3>(9);
  accel_bias_ += mean_delta.segment<3>(12);

  timestamp_ = imu.timestamp;
}

void UKF::resetPose(const gtsam::Pose3& new_pose) {
  if (!initialized_) return;
  // Snap pose only (keep velocity & biases)
  pose_ = new_pose;
  // Optionally: shrink pose/orientation covariance here after anchors
}

void UKF::resetPoseVelocity(const gtsam::Pose3& new_pose, const Eigen::Vector3d& new_velocity) {
  if (!initialized_) return;
  // Snap both pose and velocity (prevents accel bias drift accumulation)
  pose_ = new_pose;
  velocity_ = new_velocity;
  // Optionally: shrink pose/velocity covariance here after anchors
}

void UKF::resetWithCovariance(const gtsam::Pose3& new_pose,
                               const Eigen::Vector3d& new_velocity,
                               const Eigen::Vector3d& new_gyro_bias,
                               const Eigen::Vector3d& new_accel_bias,
                               const Eigen::Matrix<double, 15, 15>& new_covariance) {
  if (!initialized_) return;

  // Reset the mean state from smoother's optimized estimates
  pose_ = new_pose;
  velocity_ = new_velocity;
  gyro_bias_ = new_gyro_bias;
  accel_bias_ = new_accel_bias;

  // Reset the covariance from smoother's marginal covariance
  // The input covariance MUST be in UKF error-state order: [pos, vel, rot, gyro_bias, accel_bias]
  // Apply Cholesky decomposition to get square-root form
  Eigen::LLT<Eigen::Matrix<double, 15, 15>> llt(new_covariance);
  if (llt.info() == Eigen::Success) {
    cov_sqrt_ = llt.matrixL();
  } else {
    // If decomposition fails (e.g., not positive definite), apply regularization
    Eigen::Matrix<double, 15, 15> reg_cov = new_covariance;
    reg_cov += 1e-6 * Eigen::Matrix<double, 15, 15>::Identity();
    cov_sqrt_ = reg_cov.llt().matrixL();
  }

  // Note: This is the preferred re-anchoring method because it resets BOTH the state mean
  // AND the uncertainty estimate. This prevents filter overconfidence after smoother updates.
}

gtsam::Pose3 UKF::getPose() const { return pose_; }
Eigen::Vector3d UKF::getVelocity() const { return velocity_; }
Eigen::Vector3d UKF::getGyroBias() const { return gyro_bias_; }
Eigen::Vector3d UKF::getAccelBias() const { return accel_bias_; }

Eigen::Matrix<double, 15, 15> UKF::getCovariance() const {
  return cov_sqrt_ * cov_sqrt_.transpose();
}

// ----------------- Sigma machinery (unchanged interface) -----------------

Eigen::Matrix<double, 15, 31> UKF::generateSigmaPoints(
    const Eigen::Matrix<double, 15, 15>& cov_sqrt) const {
  constexpr int n = 15;
  Eigen::Matrix<double, 15, 31> sigma;

  // Center (zero error)
  sigma.col(0).setZero();

  // Scaled sqrt(P)
  const double scale = std::sqrt(n + lambda_);
  Eigen::Matrix<double, 15, 15> A = scale * cov_sqrt;

  for (int i = 0; i < n; ++i) {
    sigma.col(1 + i)      = A.col(i);
    sigma.col(1 + n + i)  = -A.col(i);
  }
  return sigma;
}

/**
 * NOTE: Kept to satisfy the original header. We now implement it as the
 * same linearized ERROR-STATE propagation used in propagate(), so callers (if any)
 * still get correct error dynamics. It uses the current nominal pose/bias.
 */
Eigen::Matrix<double, 15, 1> UKF::propagateSigmaPoint(
    const Eigen::Matrix<double, 15, 1>& sigma_point,
    const Eigen::Vector3d& accel,  // RAW specific force
    const Eigen::Vector3d& gyro,   // RAW body rates
    double dt) const {

  const gtsam::Rot3 R_nom = pose_.rotation();
  const Eigen::Vector3d omega_b = gyro  - gyro_bias_;
  const Eigen::Vector3d f_b     = accel - accel_bias_;

  const Eigen::Matrix3d R     = R_nom.matrix();
  const Eigen::Matrix3d f_hat = gtsam::SO3::Hat(f_b);
  const Eigen::Matrix3d w_hat = gtsam::SO3::Hat(omega_b);

  const Eigen::Vector3d dp  = sigma_point.segment<3>(0);
  const Eigen::Vector3d dv  = sigma_point.segment<3>(3);
  const Eigen::Vector3d dth = sigma_point.segment<3>(6);
  const Eigen::Vector3d dbg = sigma_point.segment<3>(9);
  const Eigen::Vector3d dba = sigma_point.segment<3>(12);

  Eigen::Matrix<double, 15, 1> dxn;
  dxn.segment<3>(0)  = dp  + dv * dt;
  dxn.segment<3>(3)  = dv  + (-R * f_hat * dth - R * dba) * dt;
  dxn.segment<3>(6)  = dth + (-w_hat * dth - dbg) * dt;
  dxn.segment<3>(9)  = dbg;
  dxn.segment<3>(12) = dba;

  return dxn;
}

void UKF::computeMeanAndCovariance(const Eigen::Matrix<double, 15, 31>& sigma_points,
                                   Eigen::Matrix<double, 15, 1>& mean,
                                   Eigen::Matrix<double, 15, 15>& cov_sqrt) const {
  // Mean of error deltas
  mean.setZero();
  for (int i = 0; i < 31; ++i) {
    mean += weights_mean_(i) * sigma_points.col(i);
  }

  // Covariance from centered sigmas
  Eigen::Matrix<double, 15, 15> P = Eigen::Matrix<double, 15, 15>::Zero();
  for (int i = 0; i < 31; ++i) {
    const Eigen::Matrix<double, 15, 1> d = sigma_points.col(i) - mean;
    P.noalias() += weights_cov_(i) * (d * d.transpose());
  }

  // Square-root via Cholesky (regularize if needed)
  Eigen::LLT<Eigen::Matrix<double, 15, 15>> llt(P);
  if (llt.info() == Eigen::Success) {
    cov_sqrt = llt.matrixL();
  } else {
    P += 1e-6 * Eigen::Matrix<double, 15, 15>::Identity();
    cov_sqrt = P.llt().matrixL();
  }
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> UKF::applyThermalCorrection(
    const Eigen::Vector3d& gyro_bias, const Eigen::Vector3d& accel_bias,
    double temperature) const {
  if (!thermal_comp_ || !thermal_comp_->isActive()) {
    // No thermal compensation available, return biases unchanged
    return {gyro_bias, accel_bias};
  }

  // Compute temperature-dependent bias prediction
  Eigen::Vector3d thermal_gyro_bias = thermal_comp_->compensateGyroBias(temperature);
  Eigen::Vector3d thermal_accel_bias = thermal_comp_->compensateAccelBias(temperature);

  // Combine: Add thermal model prediction to estimated bias
  // This compensates for temperature-induced drift
  Eigen::Vector3d corrected_gyro_bias = gyro_bias + thermal_gyro_bias;
  Eigen::Vector3d corrected_accel_bias = accel_bias + thermal_accel_bias;

  return {corrected_gyro_bias, corrected_accel_bias};
}

}  // namespace core
}  // namespace chimera
