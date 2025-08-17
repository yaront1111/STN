#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct State {
  Eigen::Vector3d p_NED{0,0,0};     // m
  Eigen::Vector3d v_NED{0,0,0};     // m/s
  Eigen::Quaterniond q_BN{1,0,0,0}; // body->NED
  Eigen::Vector3d b_a{0,0,0};       // accel bias (m/s^2)
  Eigen::Vector3d b_g{0,0,0};       // gyro bias (rad/s)
  double t{0.0};                    // seconds since start
};

struct ImuSample {
  double t{0.0};
  Eigen::Vector3d acc_mps2{0,0,0};  // body specific force
  Eigen::Vector3d gyro_rps{0,0,0};  // body angular rate
};
