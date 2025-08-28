#pragma once
#include "types.h"
#include <Eigen/Dense>
#include <vector>

/**
 * Unscented Kalman Filter for Gravity-Primary Navigation
 * 
 * Superior to EKF for highly nonlinear gravity gradient measurements
 * Uses sigma points to propagate uncertainty without linearization
 */
class UKF {
public:
    // Extended state dimension: 18 states
    // [position(3), velocity(3), attitude(4), acc_bias(3), gyro_bias(3), clock_offset(1), clock_drift(1)]
    static constexpr int STATE_DIM = 18;
    static constexpr int SIGMA_POINTS = 2 * STATE_DIM + 1;
    
    struct Config {
        double alpha;  // Spread of sigma points
        double beta;    // Prior knowledge of distribution (2 = Gaussian)
        double kappa;   // Secondary scaling parameter
        
        // Process noise parameters
        double q_pos;
        double q_vel;
        double q_att;
        double q_acc_bias;
        double q_gyro_bias;
        double q_clock_offset;
        double q_clock_drift;
        
        // Constructor with default values
        Config() : 
            alpha(1e-3), beta(2.0), kappa(0.0),
            q_pos(0.01), q_vel(0.1), q_att(0.001),
            q_acc_bias(1e-6), q_gyro_bias(1e-7),
            q_clock_offset(1e-9), q_clock_drift(1e-12) {}
    };
    
    UKF(const Config& cfg = Config()) : cfg_(cfg) {
        computeWeights();
    }
    
    /**
     * Initialize filter with initial state and covariance
     */
    void init(const State& x0, const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P0);
    
    /**
     * Prediction step - propagate sigma points through dynamics
     */
    void predict(double dt);
    
    /**
     * Update with gravity gradient tensor measurement
     */
    void updateGravityGradient(const Eigen::Matrix3d& measured_gradient,
                               const Eigen::Matrix3d& gradient_noise);
    
    /**
     * Update with gravity anomaly scalar measurement
     */
    void updateGravityAnomaly(double measured_anomaly, double anomaly_noise);
    
    /**
     * Propagate with IMU measurements
     */
    void propagateWithIMU(const ImuSample& imu, double dt);
    
    /**
     * Update with clock measurements
     */
    void updateClock(double offset_s, double drift_ppm);
    
    /**
     * Get current state estimate
     */
    State getState() const { return x_; }
    
    /**
     * Get current covariance
     */
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> getCovariance() const { return P_; }
    
private:
    Config cfg_;
    State x_;  // Current state estimate
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_;  // State covariance
    
    // Sigma points and weights
    std::vector<State> sigma_points_;
    Eigen::VectorXd weights_mean_;
    Eigen::VectorXd weights_cov_;
    
    // Computed UKF parameters
    double lambda_;
    
    void computeWeights();
    void generateSigmaPoints();
    State propagateState(const State& x, double dt);
    Eigen::VectorXd stateToVector(const State& x);
    State vectorToState(const Eigen::VectorXd& v);
    
    /**
     * Compute gravity gradient at given position using EGM2020
     */
    Eigen::Matrix3d computeGradientFromPosition(const Eigen::Vector3d& pos_ECEF);
};