#pragma once
#include "types.h"

class EKF {
public:
  void init(const State& x0);
  void propagate(State& x, double dt);
  void update_position(State& x, const Eigen::Vector3d& z_pos_NED, const Eigen::Matrix3d& R);
private:
  bool inited_ = false;
  Eigen::Matrix<double, 15, 15> P_;
  Eigen::Matrix3d P_pos_ = Eigen::Matrix3d::Identity() * 100.0;  // Large initial uncertainty
};
