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
        double alpha = 1e-3;  // Spread of sigma points
        double beta = 2.0;    // Prior knowledge of distribution (2 = Gaussian)
        double kappa = 0.0;   // Secondary scaling parameter
        
        // Process noise parameters
        double q_pos = 0.01;
        double q_vel = 0.1;
        double q_att = 0.001;
        double q_acc_bias = 1e-6;
        double q_gyro_bias = 1e-7;
        double q_clock_offset = 1e-9;
        double q_clock_drift = 1e-12;
    };
    
    UKF(const Config& cfg = Config()) : cfg_(cfg) {
        computeWeights();
    }
    
    /**
     * Initialize filter with initial state and covariance
     */
    void init(const ExtendedState& x0, const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P0);
    
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
     * Get current state estimate
     */
    ExtendedState getState() const { return x_; }
    
    /**
     * Get current covariance
     */
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> getCovariance() const { return P_; }
    
private:
    Config cfg_;
    ExtendedState x_;  // Current state estimate
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_;  // State covariance
    
    // Sigma points and weights
    std::vector<ExtendedState> sigma_points_;
    Eigen::VectorXd weights_mean_;
    Eigen::VectorXd weights_cov_;
    
    // Computed UKF parameters
    double lambda_;
    
    void computeWeights();
    void generateSigmaPoints();
    ExtendedState propagateState(const ExtendedState& x, double dt);
    Eigen::VectorXd stateToVector(const ExtendedState& x);
    ExtendedState vectorToState(const Eigen::VectorXd& v);
    
    /**
     * Compute gravity gradient at given position using EGM2020
     */
    Eigen::Matrix3d computeGradientFromPosition(const Eigen::Vector3d& pos_ECEF);
};