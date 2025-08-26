#pragma once
#include "types.h"
#include "filters.h"

class EKF {
public:
  void init(const State& x0);
  void propagate(State& x, double dt);
  void update_position(State& x, const Eigen::Vector3d& z_pos_NED, const Eigen::Matrix3d& R);
  bool update_agl(State& x, double z_agl, double terrain_h, const Eigen::Vector2d& terrain_grad);
  bool update_agl_fullstate(State& x, double z_agl, double terrain_h, const Eigen::Vector2d& terrain_grad);
  void update_gravity(State& x, const IIR1& f_N_z);  // New gravity update method
  
  // Accessors for adaptive TRN
  Eigen::Matrix3d get_P_pos() const { return P_.block<3,3>(0,0); }
  void set_P_pos(const Eigen::Matrix3d& P) { P_.block<3,3>(0,0) = P; }
  
  // Singer model parameters
  void setSingerParams(double accel_sigma, double correlation_time) {
    singer_sigma_ = accel_sigma;
    singer_tau_ = correlation_time;
  }
  
private:
  bool inited_ = false;
  // Full 15-state covariance: [pos(3), vel(3), att(3), b_a(3), b_g(3)]
  Eigen::Matrix<double, 15, 15> P_;
  
  // Singer acceleration model parameters
  double singer_sigma_ = 0.5;  // Acceleration noise std dev (m/s^2)
  double singer_tau_ = 60.0;   // Correlation time constant (seconds)
  
  // Helper function to compute Singer Q matrix
  Eigen::Matrix<double, 6, 6> computeSingerQ(double dt) const;
  
  IIR1 g_obs_filt_;  // Low-pass filter for gravity observation
  
  // Deprecated - now using full P_ matrix
  Eigen::Matrix3d P_pos_ = Eigen::Matrix3d::Identity() * 100.0;
};
