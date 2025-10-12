#pragma once
#include "state.h"
#include <Eigen/Dense>

namespace aion {

// Extended state with STN clock parameters
struct ExtendedState : public State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Clock parameters for STN
  double clock_bias_m{0.0};      // Clock bias in meters (bias * c)
  double clock_drift_frac{0.0};  // Clock drift as fractional frequency (Hz/Hz or ppm)

  // Extended dimension
  static constexpr int DIM_EXT = State::DIM + 2; // Add clock bias & drift

  // Convert to/from standard State
  State toStandardState() const {
    State s;
    s.position = position;
    s.velocity = velocity;
    s.quaternion = quaternion;
    s.gyro_bias = gyro_bias;
    s.accel_bias = accel_bias;
    s.accel_scale = accel_scale;
    s.wind = wind;
    s.time = time;
    return s;
  }

  void fromStandardState(const State& s) {
    position = s.position;
    velocity = s.velocity;
    quaternion = s.quaternion;
    gyro_bias = s.gyro_bias;
    accel_bias = s.accel_bias;
    accel_scale = s.accel_scale;
    wind = s.wind;
    time = s.time;
    // Clock parameters stay as is
  }

  void resetClock() {
    clock_bias_m = 0.0;
    clock_drift_frac = 0.0;
  }
};

// Extended error state
struct ExtendedErrorState : public ErrorState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int DIM_EXT = ErrorState::DIM + 2;

  double delta_clock_bias_m{0.0};
  double delta_clock_drift_frac{0.0};

  Eigen::VectorXd toExtendedVector() const {
    Eigen::VectorXd vec(DIM_EXT);
    vec.head<ErrorState::DIM>() = toVector();
    vec(ErrorState::DIM) = delta_clock_bias_m;
    vec(ErrorState::DIM + 1) = delta_clock_drift_frac;
    return vec;
  }

  void fromExtendedVector(const Eigen::VectorXd& vec) {
    if (vec.size() != DIM_EXT) {
      throw std::runtime_error("Invalid extended vector size");
    }
    fromVector(vec.head<ErrorState::DIM>());
    delta_clock_bias_m = vec(ErrorState::DIM);
    delta_clock_drift_frac = vec(ErrorState::DIM + 1);
  }

  void resetExtended() {
    reset();
    delta_clock_bias_m = 0.0;
    delta_clock_drift_frac = 0.0;
  }
};

// Extended covariance
class ExtendedStateCovariance : public StateCovariance {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ExtendedStateCovariance() {
    // Resize to include clock states
    P_ext_.setZero(ExtendedErrorState::DIM_EXT, ExtendedErrorState::DIM_EXT);

    // Initialize standard states
    P_ext_.topLeftCorner<ErrorState::DIM, ErrorState::DIM>() = matrix();

    // Initialize clock covariance
    P_ext_(ErrorState::DIM, ErrorState::DIM) = 900.0;  // 30m clock bias sigma
    P_ext_(ErrorState::DIM + 1, ErrorState::DIM + 1) = 1e-12; // 1 ppb drift sigma
  }

  const Eigen::MatrixXd& extendedMatrix() const { return P_ext_; }
  Eigen::MatrixXd& extendedMatrix() { return P_ext_; }

  // Clock covariance blocks
  double clockBiasVariance() const {
    return P_ext_(ErrorState::DIM, ErrorState::DIM);
  }

  double clockDriftVariance() const {
    return P_ext_(ErrorState::DIM + 1, ErrorState::DIM + 1);
  }

  void setClockBiasVariance(double var) {
    P_ext_(ErrorState::DIM, ErrorState::DIM) = var;
  }

  void setClockDriftVariance(double var) {
    P_ext_(ErrorState::DIM + 1, ErrorState::DIM + 1) = var;
  }

  // Sync standard covariance with extended
  void syncFromStandard() {
    P_ext_.topLeftCorner<ErrorState::DIM, ErrorState::DIM>() = matrix();
  }

  void syncToStandard() {
    matrix() = P_ext_.topLeftCorner<ErrorState::DIM, ErrorState::DIM>();
  }

private:
  Eigen::MatrixXd P_ext_; // 19x19 extended error-state covariance
};

} // namespace aion