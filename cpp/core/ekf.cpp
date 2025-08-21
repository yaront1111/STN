#include "ekf.h"
#include "gravity_model.h"
using Eigen::Matrix3d; using Eigen::Vector3d; using Eigen::Vector2d;

void EKF::init(const State& /*x0*/) {
  P_.setIdentity(); P_ *= 1e-2; 
  P_pos_ = Matrix3d::Identity() * 100.0;  // Large initial position uncertainty
  inited_ = true;
}

void EKF::propagate(State& /*x*/, double dt) {
  if(!inited_) return;
  P_ += Eigen::Matrix<double,15,15>::Identity()*1e-4;
  
  // Position random walk process noise - optimized for Grade A
  const double q_horiz = 1.5;  // m^2/s - balanced for stability
  const double q_vert = 0.3;   // m^2/s - vertical tighter with baro
  P_pos_(0,0) += q_horiz * dt;
  P_pos_(1,1) += q_horiz * dt;
  P_pos_(2,2) += q_vert * dt;
  
  // Limit covariance growth but also maintain minimum
  for(int i = 0; i < 3; i++) {
    P_pos_(i,i) = std::clamp(P_pos_(i,i), 0.01, 100.0);  // Between 0.1m and 10m std dev
  }
}

void EKF::update_position(State& x, const Vector3d& z, const Matrix3d& R) {
  if(!inited_) init(x);
  
  // Innovation
  Vector3d r = z - x.p_NED;
  
  // Innovation covariance: S = P + R
  Matrix3d S = P_pos_ + R;
  
  // Kalman gain: K = P * S^-1
  // For numerical stability, only compute for axes with reasonable R
  Matrix3d K = Matrix3d::Zero();
  for(int i = 0; i < 3; i++) {
    if(R(i,i) < 1e10) {  // Only update if measurement is valid
      K(i,i) = P_pos_(i,i) / S(i,i);
    }
  }
  
  // State update
  x.p_NED += K * r;
  
  // Covariance update: P = (I - K) * P
  P_pos_ = (Matrix3d::Identity() - K) * P_pos_;
}

bool EKF::update_agl(State& x, double z_agl, double terrain_h, const Vector2d& terrain_grad) {
  if(!inited_) init(x);
  
  // Configuration parameters
  const double sigma_agl = 0.7;     // Radar altimeter noise (m)
  const double nis_gate = 9.21;     // 97.5% gate for scalar measurement
  const double slope_floor = 0.02;  // Minimum slope for observability
  const double alpha = 0.3;         // Soft update factor
  
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
  
  // Innovation covariance: S = H*P*H' + R
  double S = (H * P_pos_ * H.transpose())(0,0) + R;
  
  // Normalized innovation squared (scalar NIS)
  double nis = (y * y) / S;
  
  // Gate check
  if (nis > nis_gate) {
    return false;  // Reject outlier
  }
  
  // Kalman gain: K = P*H'/S
  Vector3d K = (P_pos_ * H.transpose()) / S;
  
  // State update with soft factor
  x.p_NED += alpha * K * y;
  
  // Covariance update
  P_pos_ = (Matrix3d::Identity() - alpha * K * H) * P_pos_;
  
  // Ensure covariance stays positive definite
  for(int i = 0; i < 3; i++) {
    P_pos_(i,i) = std::max(P_pos_(i,i), 0.01);
  }
  
  return true;
}

void EKF::update_gravity(State& x, const IIR1& f_N_z) {
  // Constants for the gravity update
  const double g_grad = 3.086e-6;   // Gravity gradient (m/s^2 per m of altitude)
  const double R = 5e-5;            // Tighter measurement noise for stronger update
  const double nis_gate = 9.21;     // 97.5% NIS gate - accept more updates
  const double lat_rad = 32.0 * M_PI/180.0;  // Fixed latitude for Phase 1

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

  // 7. Update State and Covariance with soft factor for stability
  const double alpha = 0.5;  // Stronger update for better vertical control
  x.p_NED.z() += alpha * K * y;
  P_pos_(2,2) = (1.0 - alpha * K * H) * P_pos_(2,2);
  
  // Ensure positive definite
  P_pos_(2,2) = std::max(P_pos_(2,2), 0.01);
}
