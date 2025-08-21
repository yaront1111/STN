#pragma once
#include "types.h"

class EKF {
public:
  void init(const State& x0);
  void propagate(State& x, double dt);
  void update_position(State& x, const Eigen::Vector3d& z_pos_NED, const Eigen::Matrix3d& R);
  bool update_agl(State& x, double z_agl, double terrain_h, const Eigen::Vector2d& terrain_grad);
  
  // Accessors for adaptive TRN
  Eigen::Matrix3d get_P_pos() const { return P_pos_; }
  void set_P_pos(const Eigen::Matrix3d& P) { P_pos_ = P; }
private:
  bool inited_ = false;
  Eigen::Matrix<double, 15, 15> P_;
  Eigen::Matrix3d P_pos_ = Eigen::Matrix3d::Identity() * 100.0;  // Large initial uncertainty
};
