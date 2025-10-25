#pragma once

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/ImuBias.h>
#include <memory>

namespace chimera {
namespace core {

/**
 * @brief Unscented Kalman Filter with SO(3) mechanization and square-root covariance
 *
 * State vector (error-state formulation):
 * - position (3): [px, py, pz] in world frame
 * - velocity (3): [vx, vy, vz] in world frame
 * - rotation (SO(3)): represented as Rot3
 * - gyro bias (3): [bg_x, bg_y, bg_z]
 * - accel bias (3): [ba_x, ba_y, ba_z]
 *
 * Total: 15 DOF (position + velocity + rotation (3 for error state) + gyro_bias + accel_bias)
 */
class UKF {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief IMU measurement structure
   */
  struct ImuMeasurement {
    double timestamp;
    Eigen::Vector3d accel;   // Specific force (m/s^2) in body frame
    Eigen::Vector3d gyro;    // Angular velocity (rad/s) in body frame
    double temperature;      // Sensor temperature (Celsius)
  };

  /**
   * @brief UKF noise parameters
   */
  struct NoiseParams {
    double gyro_noise_density;     // rad/s/sqrt(Hz)
    double gyro_random_walk;       // rad/s^2/sqrt(Hz)
    double accel_noise_density;    // m/s^2/sqrt(Hz)
    double accel_random_walk;      // m/s^3/sqrt(Hz)
  };

  /**
   * @brief Constructor
   * @param noise UKF noise parameters
   * @param gravity_magnitude Gravity magnitude (default: 9.80665 m/s^2)
   */
  explicit UKF(const NoiseParams& noise, double gravity_magnitude = 9.80665);

  /**
   * @brief Initialize the filter with initial state
   * @param pose Initial pose (position + orientation)
   * @param velocity Initial velocity in world frame
   * @param initial_cov Initial state covariance (15x15)
   */
  void initialize(const gtsam::Pose3& pose, const Eigen::Vector3d& velocity,
                  const Eigen::Matrix<double, 15, 15>& initial_cov);

  /**
   * @brief Propagate state using IMU measurement (prediction step)
   * @param imu IMU measurement
   */
  void propagate(const ImuMeasurement& imu);

  /**
   * @brief Reset pose to a new value (re-anchoring from smoother)
   * @param new_pose New pose from smoother estimate
   */
  void resetPose(const gtsam::Pose3& new_pose);

  /**
   * @brief Reset pose and velocity to new values (re-anchoring from smoother)
   * @param new_pose New pose from smoother estimate
   * @param new_velocity New velocity in world frame from smoother
   */
  void resetPoseVelocity(const gtsam::Pose3& new_pose, const Eigen::Vector3d& new_velocity);

  /**
   * @brief Reset complete state (pose, velocity, biases) AND covariance from smoother
   * @param new_pose Optimized pose from smoother
   * @param new_velocity Optimized velocity from smoother
   * @param new_gyro_bias Optimized gyro bias from smoother
   * @param new_accel_bias Optimized accel bias from smoother
   * @param new_covariance Full 15x15 covariance matrix from smoother (must be in UKF state order)
   *
   * This is the preferred re-anchoring method as it resets both the state mean AND uncertainty.
   * The covariance must be in UKF error-state order: [rot, vel, pos, gyro_bias, accel_bias].
   * Use covariance permutation logic to convert from GTSAM's order if needed.
   */
  void resetWithCovariance(const gtsam::Pose3& new_pose,
                           const Eigen::Vector3d& new_velocity,
                           const Eigen::Vector3d& new_gyro_bias,
                           const Eigen::Vector3d& new_accel_bias,
                           const Eigen::Matrix<double, 15, 15>& new_covariance);

  /**
   * @brief Get current state
   * @return Current pose estimate
   */
  gtsam::Pose3 getPose() const;

  /**
   * @brief Get current velocity
   * @return Current velocity estimate in world frame
   */
  Eigen::Vector3d getVelocity() const;

  /**
   * @brief Get current gyro bias
   * @return Current gyro bias estimate
   */
  Eigen::Vector3d getGyroBias() const;

  /**
   * @brief Get current accel bias
   * @return Current accel bias estimate
   */
  Eigen::Vector3d getAccelBias() const;

  /**
   * @brief Get current state covariance
   * @return 15x15 covariance matrix
   */
  Eigen::Matrix<double, 15, 15> getCovariance() const;

  /**
   * @brief Get current timestamp
   * @return Latest propagation timestamp
   */
  double getTimestamp() const { return timestamp_; }

  /**
   * @brief Check if filter is initialized
   */
  bool isInitialized() const { return initialized_; }

 private:
  /**
   * @brief Generate sigma points using square-root (Cholesky) decomposition
   * @param cov_sqrt Square-root of covariance (15x15)
   * @return Matrix of sigma points (15 x 31), where 31 = 2*15 + 1
   */
  Eigen::Matrix<double, 15, 31> generateSigmaPoints(
      const Eigen::Matrix<double, 15, 15>& cov_sqrt) const;

  /**
   * @brief Propagate a single sigma point through dynamics
   * @param sigma_point State perturbation (15x1)
   * @param accel Measured specific force (body frame)
   * @param gyro Measured angular velocity (body frame)
   * @param dt Time delta
   * @return Propagated sigma point (15x1)
   */
  Eigen::Matrix<double, 15, 1> propagateSigmaPoint(
      const Eigen::Matrix<double, 15, 1>& sigma_point, const Eigen::Vector3d& accel,
      const Eigen::Vector3d& gyro, double dt) const;

  /**
   * @brief Compute mean and covariance from propagated sigma points
   * @param sigma_points Propagated sigma points (15 x 31)
   * @param mean[out] Mean state (15x1)
   * @param cov_sqrt[out] Square-root covariance (15x15)
   */
  void computeMeanAndCovariance(const Eigen::Matrix<double, 15, 31>& sigma_points,
                                Eigen::Matrix<double, 15, 1>& mean,
                                Eigen::Matrix<double, 15, 15>& cov_sqrt) const;

  /**
   * @brief Apply thermal bias correction (placeholder for future)
   * @param gyro_bias Current gyro bias
   * @param accel_bias Current accel bias
   * @param temperature Sensor temperature
   * @return Corrected biases
   */
  std::pair<Eigen::Vector3d, Eigen::Vector3d> applyThermalCorrection(
      const Eigen::Vector3d& gyro_bias, const Eigen::Vector3d& accel_bias,
      double temperature) const;

  // State
  gtsam::Pose3 pose_;                                  // Current pose (position + rotation)
  Eigen::Vector3d velocity_;                           // Velocity in world frame
  Eigen::Vector3d gyro_bias_;                          // Gyro bias
  Eigen::Vector3d accel_bias_;                         // Accel bias
  Eigen::Matrix<double, 15, 15> cov_sqrt_;             // Square-root covariance (Cholesky)
  double timestamp_;                                   // Last update timestamp
  bool initialized_;

  // Noise parameters
  NoiseParams noise_;
  double gravity_magnitude_;
  Eigen::Vector3d gravity_vector_;  // NED: [0, 0, +g] (Down-positive)

  // UKF parameters
  double alpha_;   // Spread of sigma points (typically 1e-3)
  double beta_;    // Prior knowledge of distribution (2.0 for Gaussian)
  double kappa_;   // Secondary scaling (typically 0)
  double lambda_;  // Composite scaling parameter

  // Weights for sigma points
  Eigen::Matrix<double, 31, 1> weights_mean_;
  Eigen::Matrix<double, 31, 1> weights_cov_;
};

}  // namespace core
}  // namespace chimera
