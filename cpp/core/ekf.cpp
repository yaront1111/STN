#include "ekf.h"
using Eigen::Matrix3d; using Eigen::Vector3d;

void EKF::init(const State& /*x0*/) {
  P_.setIdentity(); P_ *= 1e-2; 
  P_pos_ = Matrix3d::Identity() * 100.0;  // Large initial position uncertainty
  inited_ = true;
}

void EKF::propagate(State& /*x*/, double dt) {
  if(!inited_) return;
  P_ += Eigen::Matrix<double,15,15>::Identity()*1e-4;
  
  // Position random walk process noise
  const double q = 5.0;  // m^2/s - increased to allow faster convergence
  P_pos_ += Matrix3d::Identity() * q * dt;
  
  // Limit covariance growth
  for(int i = 0; i < 3; i++) {
    P_pos_(i,i) = std::min(P_pos_(i,i), 10000.0);  // Cap at 100m std dev
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
