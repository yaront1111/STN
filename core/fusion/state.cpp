#include "core/fusion/state.h"
#include <cmath>

namespace aion {

//---------------- State ----------------

void State::reset() {
  position.setZero();
  velocity.setZero();
  quaternion.setIdentity();
  gyro_bias.setZero();
  accel_bias.setZero();
  accel_scale.setOnes();
  wind.setZero();
  time = 0.0;
}

Eigen::Vector3d State::eulerRPY() const {
  const Eigen::Matrix3d R = R_bn(); // body→NED

  // Robust pitch from R(2,0); clamp for numerical safety
  const double s = std::clamp(-R(2,0), -1.0, 1.0);
  const double roll  = std::atan2(R(2,1), R(2,2));
  const double pitch = std::asin(s);
  const double yaw   = std::atan2(R(1,0), R(0,0));
  return {roll, pitch, yaw};
}

//---------------- Error-State ----------------

Eigen::VectorXd ErrorState::toVector() const {
  Eigen::VectorXd v(DIM);
  v.segment<3>(0)  = delta_position;
  v.segment<3>(3)  = delta_velocity;
  v.segment<3>(6)  = delta_theta;
  v.segment<3>(9)  = delta_gyro_bias;
  v.segment<3>(12) = delta_accel_bias;
  v.segment<2>(15) = delta_wind;
  return v;
}

void ErrorState::fromVector(const Eigen::VectorXd& vec) {
  if (vec.size() != DIM) throw std::runtime_error("Invalid error-state dimension");
  delta_position   = vec.segment<3>(0);
  delta_velocity   = vec.segment<3>(3);
  delta_theta      = vec.segment<3>(6);
  delta_gyro_bias  = vec.segment<3>(9);
  delta_accel_bias = vec.segment<3>(12);
  delta_wind       = vec.segment<2>(15);
}

void ErrorState::reset() {
  delta_position.setZero();
  delta_velocity.setZero();
  delta_theta.setZero();
  delta_gyro_bias.setZero();
  delta_accel_bias.setZero();
  delta_wind.setZero();
}

//---------------- Covariance ----------------

StateCovariance::StateCovariance() : P_(ErrorState::DIM, ErrorState::DIM) {
  P_.setIdentity();
}

void StateCovariance::initialize(double pos_std, double vel_std, double att_std,
                                 double gb_std, double ba_std, double wind_std) {
  P_.setZero();
  positionBlock().setIdentity(); positionBlock() *= pos_std * pos_std;
  velocityBlock().setIdentity(); velocityBlock() *= vel_std * vel_std;
  attitudeBlock().setIdentity(); attitudeBlock() *= att_std * att_std;
  gyroBiasBlock().setIdentity(); gyroBiasBlock() *= gb_std * gb_std;
  accelBiasBlock().setIdentity(); accelBiasBlock() *= ba_std * ba_std;
  windBlock().setIdentity();     windBlock()     *= wind_std * wind_std;
}

bool StateCovariance::isPositiveDefinite() const {
  Eigen::LLT<Eigen::MatrixXd> llt(P_);
  return (llt.info() == Eigen::Success);
}

void StateCovariance::enforceSymmetry() {
  P_ = 0.5 * (P_ + P_.transpose());
  if (!isPositiveDefinite()) {
    // Adaptive jitter based on scale (trace) to avoid unit issues
    const double tr = std::max(P_.trace(), 1e-12);
    const double eps = std::max(1e-12, 1e-9 * tr / static_cast<double>(P_.rows()));
    P_.diagonal().array() += eps;
  }
}

} // namespace aion
