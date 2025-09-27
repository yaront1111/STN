#pragma once

#include <memory>
#include <vector>
#include <limits>

#include <Eigen/Dense>

#include "core/fusion/state.h"
#include "core/fusion/process_model.h"

namespace aion {

/**
 * IMU data container (specific force in body, rad/s gyro).
 */
struct IMUData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d angular_velocity;  // [rad/s]
  Eigen::Vector3d acceleration;      // [m/s^2] (specific force)
  double timestamp;                  // [s]
  double temperature;                // [°C]

  IMUData()
      : angular_velocity(Eigen::Vector3d::Zero()),
        acceleration(Eigen::Vector3d::Zero()),
        timestamp(0.0),
        temperature(25.0) {}
};

/**
 * Abstract measurement used by the UKF update step.
 * Implementors provide prediction from state, value, covariance, dimension, and time.
 */
class MeasurementBase {
 public:
  virtual ~MeasurementBase() = default;

  virtual Eigen::VectorXd predict(const State& state) const = 0;
  virtual Eigen::VectorXd getValue() const = 0;
  virtual Eigen::MatrixXd getCovariance() const = 0;
  virtual int getDimension() const = 0;
  virtual double getTime() const = 0;
};

/**
 * UKF configuration (densities are per √Hz).
 */
struct SRUKFConfig {
  // Unscented transform scaling
  double alpha = 1e-3;
  double beta  = 2.0;
  double kappa = 0.0;

  // Process noise densities
  double gyro_noise_density        = 2.0e-2; // (rad/s)/√Hz
  double accel_noise_density       = 3.0e-2; // m/s^2/√Hz
  double gyro_bias_noise_density   = 5.0e-4; // (rad/s^2)/√Hz
  double accel_bias_noise_density  = 5.0e-4; // m/s^3/√Hz
  double wind_noise_density        = 1.0e-1; // m/s^2/√Hz

  // Gravity magnitude [m/s^2] (NED +down)
  double gravity = 9.80665;

  // NIS gating threshold (set to infinity to disable)
  double nis_gate_1d = std::numeric_limits<double>::infinity();
};

/**
 * Error-state UKF on SO(3) (non square-root variant).
 * - Sigma points live in the error/tangent space (ErrorState).
 * - We propagate nominal states through the process model for each sigma.
 * - Mean is computed on the manifold via small Gauss-Newton.
 */
class SRUKF {
 public:
  using Config = SRUKFConfig;

  explicit SRUKF(const Config& config = Config());
  ~SRUKF();

  // Init
  void initialize(const State& initial_state, const StateCovariance& initial_cov);

  // Predict with one IMU sample
  void predict(const IMUData& imu_data);

  // Update with a generic measurement
  void update(const MeasurementBase& measurement);

  // Schmidt-Kalman update with attitude protection (zeros attitude gain)
  void schmidtUpdate(const MeasurementBase& measurement);

  // Blend to graph posterior (soft reset)
  void softReset(const State& state, const StateCovariance& cov);

  // Accessors
  const State&           getState() const { return state_; }
  const StateCovariance& getCovariance() const { return covariance_; }

  // Consistency (best-effort)
  double getNEES() const { return nees_; }
  double getNIS()  const { return nis_;  }

  // Coarse health check
  bool isHealthy() const;

 private:
  // UKF internals
  void computeWeights();
  void generateSigmaPoints();
  void propagateSigmaPoints(double dt);
  void computePrediction();         // manifold mean + covariance recompute
  void addProcessNoise(double dt);  // diagonal Qd add (keeps filter consistent)

  // Apply ErrorState correction to nominal state
  void applyCorrection(const ErrorState& error);

  // State & covariance
  State           state_;
  StateCovariance covariance_;

  // Process model
  std::unique_ptr<ProcessModel> process_model_;

  // Config
  Config config_;
  IMUData last_imu_{};  // last IMU for sigma propagation

  // Sigma points (tangent) and propagated states (nominal)
  std::vector<ErrorState> sigma_points_;            // size = 2L+1
  std::vector<State>      propagated_sigma_points_; // size = 2L+1

  // Unscented weights
  Eigen::VectorXd weights_mean_; // 2L+1
  Eigen::VectorXd weights_cov_;  // 2L+1

  // Consistency stats
  double nees_{0.0};
  double nis_{0.0};
  std::vector<double> innovation_history_;

  // Dimensions
  static constexpr int L = ErrorState::DIM;               // 17
  static constexpr int NUM_SIGMA_POINTS = 2 * L + 1;      // 35
};

}  // namespace aion
