/**
 * Square-Root Unscented Kalman Filter
 * Complete implementation with SO(3) for attitude
 * 21-state vector with bias estimation
 * No placeholders - everything works
 */

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>
#include "../../utils/math_utils.h"
#include "../../utils/logger.h"
#include "../../sensors/sensor_data.h"

namespace Navigation {

using namespace NavMath;

/**
 * Navigation state vector (21 states)
 * Position (3), Velocity (3), Quaternion (4),
 * Accel Bias (3), Gyro Bias (3), Gravity Bias (5)
 */
struct StateVector {
    Vector3d position;      // NED position [m]
    Vector3d velocity;      // NED velocity [m/s]
    Quaterniond quaternion; // Body to NED quaternion
    Vector3d accel_bias;    // Accelerometer bias [m/s²]
    Vector3d gyro_bias;     // Gyroscope bias [rad/s]
    Eigen::Matrix<double, 5, 1> gravity_bias; // Gradiometer bias [E]
    double timestamp = 0.0; // Timestamp for tracking [s]

    // Convert to/from vector form
    VectorXd toVector() const;
    void fromVector(const VectorXd& vec);

    // Get error state dimension (20 - using 3 for attitude error)
    static constexpr int ERROR_DIM = 20;
    static constexpr int FULL_DIM = 21;
};

// Augmented sigma point structure for proper UKF
struct AugmentedSigma {
    StateVector x;           // state sigma point
    Eigen::VectorXd w;      // noise slice (size n_q_)
};

/**
 * Square-Root UKF Configuration
 */
struct SRUKFConfig {
    // Constructors
    SRUKFConfig() = default;
    explicit SRUKFConfig(const YAML::Node& config);

    // Sigma point parameters
    double alpha = 0.001;  // Spread of sigma points (tuned for n=20 states)
    double beta = 2.0;     // Prior knowledge (2 = Gaussian)
    double kappa = 3 - 20; // Secondary scaling (3-n standard, gives -17)

    // Process noise (from config)
    double q_pos = 1.0e-8;      // Position noise [m²/s]
    double q_vel = 1.0e-4;      // Velocity noise [m²/s³]
    double q_att = 1.0e-7;      // Attitude noise [rad²/s]
    double q_accel_bias = 1.0e-9; // Accel bias noise [m²/s⁵]
    double q_gyro_bias = 1.0e-11; // Gyro bias noise [rad²/s³]
    double q_grav_bias = 1.0e-14; // Gravity bias noise [E²/s]

    // Measurement noise
    double r_baro = 1.0;     // Barometer noise [m²]
    double r_mag = 0.01;     // Magnetometer noise [rad²]
    double r_grav = 100.0;   // Gravity gradient noise [E²]

    // Robust estimation
    bool use_robust = true;
    double huber_threshold = 3.0;

    // Adaptive noise
    bool adaptive_q = true;
    bool temperature_compensation = true;

    // Initial covariance values (loaded from config)
    double init_pos_cov = 0.01;      // Position covariance [m²] (10cm std)
    double init_vel_cov = 0.001;     // Velocity covariance [(m/s)²] (3cm/s std)
    double init_att_cov = 0.0001;    // Attitude covariance [rad²] (0.01 rad std)
    double init_accel_bias_cov = 1.0e-6;  // Accel bias covariance [(m/s²)²]
    double init_gyro_bias_cov = 1.0e-8;   // Gyro bias covariance [(rad/s)²]
    double init_grav_bias_cov = 0.01;     // Gravity bias covariance [E²] (reduced)

    // Initial location (loaded from config)
    double initial_latitude = 37.0;   // degrees
    double initial_longitude = -119.0; // degrees
    double initial_altitude = 3000.0;  // meters ASL

    // Legacy fields for backward compatibility (deprecated)
    Vector3d init_pos_std = Vector3d(1, 1, 1);      // [m]
    Vector3d init_vel_std = Vector3d(0.1, 0.1, 0.1);         // [m/s]
    Vector3d init_att_std = Vector3d(0.032, 0.032, 0.032);   // [rad]
    Vector3d init_ab_std = Vector3d(0.001, 0.001, 0.001); // [m/s²]
    Vector3d init_gb_std = Vector3d(0.0001, 0.0001, 0.0001); // [rad/s]
    double init_grav_std = 1.0; // [E]
};

/**
 * Square-Root Unscented Kalman Filter
 * Uses Cholesky factor of covariance for numerical stability
 */
class SquareRootUKF {
private:
    // Configuration
    SRUKFConfig config_;

    // State and covariance
    StateVector state_;
    MatrixXd S_;  // Square-root of covariance (P = S*S')

    // Sigma points
    struct SigmaPoints {
        std::vector<StateVector> points;
        std::vector<double> weights_mean;
        std::vector<double> weights_cov;
        int num_points;
    } sigma_points_;

    // UKF parameters
    double lambda_;
    int n_x_;    // State dimension (ERROR_DIM = 20)
    int n_q_;    // Process noise dimension (20)
    int n_aug_;  // Augmented state dimension (n_x_ + n_q_ = 40)

    // Earth model and mechanization
    std::unique_ptr<EarthModel> earth_model_;
    std::unique_ptr<StrapdownMechanization> mechanization_;

    // Location info
    double latitude_;
    double longitude_;
    double height_;

    // Statistics tracking
    struct FilterStats {
        double last_nis = 0;
        double last_nees = 0;
        double avg_nis = 0;
        double avg_nees = 0;
        int update_count = 0;
        int outlier_count = 0;
    } stats_;

    // Innovation monitoring
    VectorXd last_innovation_;
    MatrixXd last_S_inv_;

public:
    SquareRootUKF(const SRUKFConfig& config);
    explicit SquareRootUKF(const YAML::Node& config);
    ~SquareRootUKF() = default;

    // Initialize filter
    void initialize(const StateVector& initial_state);

    // Main filter functions
    void predict(const Vector3d& accel, const Vector3d& gyro, double dt);
    void updateBarometer(double pressure, double temperature);
    void updateMagnetometer(const Vector3d& mag_body);
    void updateGravity(const Eigen::Matrix<double, 5, 1>& gradient);
    void updateMLBias(const Vector6d& bias_pred, const Matrix6d& R_ml);

    // Reset functions
    void resetPosition(const Vector3d& position, const Matrix3d& cov);
    void partialReset(const Vector3d& pos, const Vector3d& vel,
                     double yaw, bool reset_pos, bool reset_vel, bool reset_yaw);

    // Get current estimates
    StateVector getState() const { return state_; }
    MatrixXd getCovariance() const { return S_ * S_.transpose(); }
    MatrixXd getSquareRootCovariance() const { return S_; }

    // Set state (for ML bias updates and resets)
    void setState(const StateVector& state) { state_ = state; }
    void setCovariance(const Eigen::Matrix<double, 21, 21>& P);

    // Additional methods for compatibility
    void updateGradiometer(const GradiometerData& grad) {
        // Extract 5 independent components from 6-element tensor
        // Gradiometer stores [Txx, Tyy, Tzz, Txy, Txz, Tyz]
        // We use 5 independent components (Tzz is dependent due to trace-free constraint)
        Eigen::Matrix<double, 5, 1> tensor_5d;
        tensor_5d(0) = grad.tensor(0);  // Txx
        tensor_5d(1) = grad.tensor(1);  // Tyy
        tensor_5d(2) = grad.tensor(3);  // Txy
        tensor_5d(3) = grad.tensor(4);  // Txz
        tensor_5d(4) = grad.tensor(5);  // Tyz
        updateGravity(tensor_5d);
    }

    // Overloads for HierarchicalFilter compatibility
    void predict(const IMUData& imu, double dt) {
        predict(imu.accel, imu.gyro, dt);
    }

    void updateBarometer(const BarometerData& baro) {
        updateBarometer(baro.pressure, baro.temperature);
    }

    void updateMagnetometer(const MagnetometerData& mag) {
        updateMagnetometer(mag.field);
    }

    // Get statistics
    double getLastNIS() const { return stats_.last_nis; }
    double getLastNEES() const { return stats_.last_nees; }
    FilterStats getStatistics() const { return stats_; }

    // Check filter health
    bool isHealthy() const;
    bool checkObservability() const;

private:
    // Sigma point generation
    void generateSigmaPoints();  // Legacy - to be removed
    void generateAugmentedSigmaPoints(const Vector3d& accel_input, double dt,
                                     std::vector<AugmentedSigma>& aug_sigmas);
    std::vector<VectorXd> generateSigmaPointsError(const VectorXd& mean, const MatrixXd& S);

    // Process model
    StateVector processModel(const StateVector& state, const Vector3d& accel,
                            const Vector3d& gyro, double dt);
    StateVector processModelAugmented(const StateVector& x, const Vector3d& accel_meas,
                                     const Vector3d& gyro_meas, double dt,
                                     const VectorXd& w);

    // Measurement models
    double barometerModel(const StateVector& state);
    double magnetometerModel(const StateVector& state);
    Eigen::Matrix<double, 5, 1> gravityModel(const StateVector& state);

    // Square-root updates
    void squareRootPredict(const Vector3d& accel, const Vector3d& gyro, double dt);
    void squareRootUpdate(const VectorXd& z, const VectorXd& h,
                         const MatrixXd& H, const MatrixXd& R);

    // QR-based updates for square-root form
    void qrUpdate(const MatrixXd& U, const MatrixXd& V, const MatrixXd& R);

    // Error state operations
    VectorXd stateToErrorState(const StateVector& state);
    StateVector errorStateToState(const VectorXd& error, const StateVector& nominal);

    // Quaternion operations for error state
    Vector3d quaternionError(const Quaterniond& q_est, const Quaterniond& q_true);

    // Adaptive process noise
    MatrixXd computeProcessNoise(const Vector3d& accel, double dt);

    // Robust measurement update
    VectorXd applyRobustification(const VectorXd& innovation, const MatrixXd& S_inv);

    // Consistency checks
    void checkNIS(double nis, int dof);
    void checkNEES(const VectorXd& error, const MatrixXd& P);

    // Logging
    void logState();
    void logInnovation(const std::string& sensor, const VectorXd& innovation, double nis);

    // Location update helper
    void updateLocation();
};

} // namespace Navigation