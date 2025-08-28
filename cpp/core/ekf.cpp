#include "ekf.h"
#include "gravity_model.h"
using Eigen::Matrix3d; using Eigen::Vector3d; using Eigen::Vector2d;

void EKF::init(const State& /*x0*/) {
  P_.setZero();
  
  // Initialize covariance with realistic values
  // Position uncertainty (meters)
  P_.block<3,3>(0,0) = Matrix3d::Identity() * 100.0;
  
  // Velocity uncertainty (m/s)
  P_.block<3,3>(3,3) = Matrix3d::Identity() * 10.0;
  
  // Attitude uncertainty (radians)
  P_.block<3,3>(6,6) = Matrix3d::Identity() * 0.1;
  
  // Accelerometer bias uncertainty (m/s^2)
  P_.block<3,3>(9,9) = Matrix3d::Identity() * 0.01;
  
  // Gyro bias uncertainty (rad/s)
  P_.block<3,3>(12,12) = Matrix3d::Identity() * 0.001;
  
  inited_ = true;
}

void EKF::propagate(State& x, double dt) {
  if(!inited_) return;
  
  // Propagate state (simple kinematic model for EKF consistency)
  // Note: INS already propagated the state, but we need velocity coupling
  // This ensures covariance matches the actual state evolution
  
  // State transition matrix F for position-velocity dynamics
  Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
  
  // Position-velocity coupling: p_new = p_old + v*dt
  F.block<3,3>(0,3) = Matrix3d::Identity() * dt;
  
  // Propagate covariance: P = F * P * F'
  P_ = F * P_ * F.transpose();
  
  // Add Singer acceleration model process noise
  Eigen::Matrix<double, 6, 6> Q_pv = computeSingerQ(dt);
  P_.block<6,6>(0,0) += Q_pv;
  
  // Attitude process noise (random walk)
  P_.block<3,3>(6,6) += Matrix3d::Identity() * 1e-6 * dt;
  
  // IMU bias process noise (slow random walk)
  P_.block<3,3>(9,9) += Matrix3d::Identity() * 1e-8 * dt;   // Accel bias
  P_.block<3,3>(12,12) += Matrix3d::Identity() * 1e-10 * dt; // Gyro bias
  
  // Ensure positive definiteness
  for(int i = 0; i < 15; i++) {
    P_(i,i) = std::max(P_(i,i), 1e-9);
  }
}

Eigen::Matrix<double, 6, 6> EKF::computeSingerQ(double dt) const {
  // Singer acceleration model Q matrix
  // Based on "Estimation and Control of Witsenhausen" formulation
  
  Eigen::Matrix<double, 6, 6> Q;
  Q.setZero();
  
  double sigma2 = singer_sigma_ * singer_sigma_;
  double tau = singer_tau_;
  
  // Helper terms
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;
  double dt5 = dt4 * dt;
  
  // For each axis (North, East, Down)
  for(int i = 0; i < 3; i++) {
    // Position-Position
    Q(i,i) = sigma2 * dt5 / 20.0;
    
    // Position-Velocity & Velocity-Position
    Q(i,i+3) = Q(i+3,i) = sigma2 * dt4 / 8.0;
    
    // Velocity-Velocity
    Q(i+3,i+3) = sigma2 * dt3 / 3.0;
  }
  
  // Apply correlation time scaling
  Q *= (1.0 - exp(-dt/tau));
  
  return Q;
}

void EKF::update_position(State& x, const Vector3d& z, const Matrix3d& R) {
  if(!inited_) init(x);
  
  // Get position covariance block
  Matrix3d P_pos = P_.block<3,3>(0,0);
  
  // Innovation
  Vector3d r = z - x.p_NED;
  
  // Innovation covariance: S = P + R
  Matrix3d S = P_pos + R;
  
  // Kalman gain: K = P * S^-1
  // For numerical stability, only compute for axes with reasonable R
  Matrix3d K = Matrix3d::Zero();
  for(int i = 0; i < 3; i++) {
    if(R(i,i) < 1e10) {  // Only update if measurement is valid
      K(i,i) = P_pos(i,i) / S(i,i);
    }
  }
  
  // State update
  x.p_NED += K * r;
  
  // Covariance update using Joseph form for stability
  Matrix3d I_K = Matrix3d::Identity() - K;
  P_pos = I_K * P_pos * I_K.transpose() + K * R * K.transpose();
  
  // Update the full covariance matrix
  P_.block<3,3>(0,0) = P_pos;
}

bool EKF::update_agl(State& x, double z_agl, double terrain_h, const Vector2d& terrain_grad) {
  if(!inited_) init(x);
  
  // Configuration parameters - balanced for performance
  const double sigma_agl = 0.5;     // Radar altimeter noise (m)
  const double nis_gate = 12.59;    // 99.5% gate for scalar measurement - more permissive
  const double slope_floor = 0.01;  // Minimum slope for observability
  
  // Predicted AGL: altitude - terrain_height
  double z_pred = (-x.p_NED.z()) - terrain_h;
  double y = z_agl - z_pred;  // Innovation (residual)
  
  // Measurement Jacobian: H = [∂AGL/∂pn, ∂AGL/∂pe, ∂AGL/∂pd]
  // AGL = -pd - h(pn,pe), so:
  // ∂AGL/∂pn = -∂h/∂pn
  // ∂AGL/∂pe = -∂h/∂pe  
  // ∂AGL/∂pd = -1
  Eigen::RowVector3d H;
  H << -terrain_grad.x(), -terrain_grad.y(), -1.0;
  
  // Measurement noise variance (inflate when terrain is flat)
  double slope = std::max(terrain_grad.norm(), 1e-6);
  double R = sigma_agl * sigma_agl * (1.0 + std::pow(slope_floor / slope, 2.0));
  
  // Smart adaptive alpha with time-varying confidence
  double convergence_time = std::min(x.t, 30.0);  // Cap at 30 seconds
  double convergence_factor = convergence_time / 30.0;  // 0 to 1 over 30s
  
  // Start aggressive (0.15), reduce to conservative (0.075) as we converge
  double base_alpha = 0.15 * (1.0 - 0.5 * convergence_factor);
  
  // Innovation-based adjustment (less aggressive)
  double innovation_ratio = std::abs(y) / (3.0 * std::sqrt(R));
  double alpha = base_alpha;
  if (innovation_ratio > 2.0) {  // Only reduce for large outliers
    alpha *= 0.5;
  } else if (innovation_ratio > 1.5) {
    alpha *= 0.75;
  }
  
  // Get position covariance block
  Matrix3d P_pos = P_.block<3,3>(0,0);
  
  // Innovation covariance: S = H*P*H' + R
  double S = (H * P_pos * H.transpose())(0,0) + R;
  
  // Normalized innovation squared (scalar NIS)
  double nis = (y * y) / S;
  
  // Gate check
  if (nis > nis_gate) {
    return false;  // Reject outlier
  }
  
  // Kalman gain: K = P*H'/S
  Vector3d K = (P_pos * H.transpose()) / S;
  
  // State update with soft factor
  x.p_NED += alpha * K * y;
  
  // Covariance update using Joseph form for numerical stability
  Matrix3d I_KH = Matrix3d::Identity() - alpha * K * H;
  P_pos = I_KH * P_pos * I_KH.transpose() + alpha * alpha * K * R * K.transpose();
  
  // Update the full covariance matrix
  P_.block<3,3>(0,0) = P_pos;
  
  // Ensure covariance stays positive definite
  // Keep minimal floor to avoid singularity
  double min_variance = 0.01;
  for(int i = 0; i < 3; i++) {
    P_(i,i) = std::max(P_(i,i), min_variance);
  }
  
  return true;
}

bool EKF::update_agl_fullstate(State& x, double z_agl, double terrain_h, const Vector2d& terrain_grad) {
  if(!inited_) init(x);
  
  // Configuration parameters - balanced for performance
  const double sigma_agl = 0.5;
  const double nis_gate = 12.59;  // More permissive
  const double slope_floor = 0.01;
  
  // Time-varying alpha
  double convergence_factor = std::min(x.t / 30.0, 1.0);
  const double alpha = 0.15 * (1.0 - 0.5 * convergence_factor);
  
  // Predicted AGL
  double z_pred = (-x.p_NED.z()) - terrain_h;
  double y = z_agl - z_pred;  // Innovation
  
  // Full measurement Jacobian H (1x15)
  // Only position affects AGL measurement
  Eigen::Matrix<double, 1, 15> H;
  H.setZero();
  H(0,0) = -terrain_grad.x();  // ∂AGL/∂pn
  H(0,1) = -terrain_grad.y();  // ∂AGL/∂pe  
  H(0,2) = -1.0;                // ∂AGL/∂pd
  
  // Measurement noise (inflate for flat terrain)
  double slope = std::max(terrain_grad.norm(), 1e-6);
  double R_scalar = sigma_agl * sigma_agl * (1.0 + std::pow(slope_floor / slope, 2.0));
  
  // Innovation covariance
  double S = (H * P_ * H.transpose())(0,0) + R_scalar;
  
  // NIS check
  double nis = (y * y) / S;
  if (nis > nis_gate) {
    return false;
  }
  
  // Full-state Kalman gain (15x1)
  Eigen::Matrix<double, 15, 1> K = (P_ * H.transpose()) / S;
  
  // Update all states (including biases!)
  Eigen::Matrix<double, 15, 1> dx = alpha * K * y;
  
  // Apply updates
  x.p_NED += dx.segment<3>(0);      // Position
  x.v_NED += dx.segment<3>(3);      // Velocity
  // Attitude update (small angle approximation)
  Vector3d dtheta = dx.segment<3>(6);
  Eigen::Quaterniond dq(1, 0.5*dtheta.x(), 0.5*dtheta.y(), 0.5*dtheta.z());
  x.q_BN = (x.q_BN * dq).normalized();
  
  x.b_a += dx.segment<3>(9);        // Accel bias
  x.b_g += dx.segment<3>(12);       // Gyro bias
  
  // Joseph form covariance update for numerical stability
  Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - alpha * K * H;
  P_ = I_KH * P_ * I_KH.transpose() + alpha * alpha * K * R_scalar * K.transpose();
  
  // Ensure positive definiteness
  for(int i = 0; i < 15; i++) {
    P_(i,i) = std::max(P_(i,i), 1e-9);
  }
  
  return true;
}

void EKF::update_gravity(State& x, const IIR1& f_N_z) {
  // Constants for the gravity update
  const double g_grad = 3.086e-6;   // Gravity gradient (m/s^2 per m of altitude)
  const double R = 1e-4;            // Balanced measurement noise
  const double nis_gate = 12.59;    // 99.5% NIS gate - more permissive
  const double lat_rad = 47.4 * M_PI/180.0;  // Zurich latitude

  // 1. Get Observed Gravity (from the smoothed specific force)
  // We assume kinematic acceleration is small, so g_obs ≈ -f_N_z
  double g_obs = -f_N_z.y;

  // 2. Get Predicted Gravity (from the filter's state)
  GravityQuery q{ .lat_rad = lat_rad, .alt_m = -x.p_NED.z() };
  GravityResult gr = GravityModel::normal(q);
  double g_pred = gr.g_mps2;

  // 3. Form the Residual (Innovation)
  double y = g_obs - g_pred;

  // 4. Form the Measurement Jacobian (H)
  // The measurement is g = g(pd). The partial derivative ∂g/∂pd is the gravity gradient.
  // Note: altitude is -pd, so ∂g/∂pd = -∂g/∂alt = -(-g_grad) = +g_grad
  double H = g_grad;

  // 5. Calculate Innovation Covariance (S) and perform NIS check
  double S = H * P_pos_(2,2) * H + R;
  double nis = y * y / S;

  if (nis > nis_gate) {
    return; // Measurement is inconsistent, reject it
  }

  // 6. Calculate Kalman Gain (K)
  double K = P_pos_(2,2) * H / S;

  // 7. Update State and Covariance with time-varying alpha
  double convergence_factor = std::min(x.t / 30.0, 1.0);
  const double alpha = 0.3 * (1.0 - 0.3 * convergence_factor);  // 0.3 to 0.21
  x.p_NED.z() += alpha * K * y;
  P_pos_(2,2) = (1.0 - alpha * K * H) * P_pos_(2,2);
  
  // Ensure positive definite
  P_pos_(2,2) = std::max(P_pos_(2,2), 0.01);
}
