#pragma once

#include "sr_ukf.h"
#include "state_extended.h"
#include <Eigen/Dense>

namespace aion {

// Extended SR-UKF for STN with clock states
class ExtendedSRUKF : public SRUKF {
public:
  struct ExtendedParams : public Params {
    // Clock process noise
    double clock_bias_process_noise{10.0};     // m/sqrt(s)
    double clock_drift_process_noise{1e-10};   // (Hz/Hz)/sqrt(s)

    // Clock measurement gates
    double clock_innovation_gate{100.0};       // meters for clock bias

    ExtendedParams() : Params() {}
  };

  explicit ExtendedSRUKF(const ExtendedParams& params = ExtendedParams());

  // Extended state accessors
  ExtendedState getExtendedState() const;
  void setExtendedState(const ExtendedState& state);

  // Extended covariance
  ExtendedStateCovariance getExtendedCovariance() const;
  void setExtendedCovariance(const ExtendedStateCovariance& cov);

  // Process model for extended states
  void predictExtended(double dt, const Eigen::Vector3d& accel_body,
                      const Eigen::Vector3d& gyro_body);

  // Update with STN measurements that use clock states
  void updateExtended(const MeasurementBase& measurement);

  // Reset clock states
  void resetClock();

  // Get clock uncertainty
  double getClockBiasStd() const;
  double getClockDriftStd() const;

protected:
  // Override sigma point generation for extended state
  void generateSigmaPointsExtended();

  // Propagate sigma points through extended process model
  void propagateSigmaPointsExtended(double dt);

  // Add clock process noise
  void addClockProcessNoise(double dt);

private:
  ExtendedParams ext_params_;
  ExtendedState ext_state_;
  ExtendedStateCovariance ext_covariance_;

  // Extended sigma points (19-dimensional)
  Eigen::MatrixXd sigma_points_ext_;  // 19 x (2*19+1)
  Eigen::VectorXd weights_mean_;      // 2*19+1
  Eigen::VectorXd weights_cov_;       // 2*19+1

  // Extended dimensions
  static constexpr int DIM_EXT = ExtendedErrorState::DIM_EXT;  // 19
  static constexpr int NUM_SIGMA = 2 * DIM_EXT + 1;            // 39
};

} // namespace aion