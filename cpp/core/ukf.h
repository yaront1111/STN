#pragma once
#include "types.h"
#include "integrity.hpp"
#include <Eigen/Dense>
#include <vector>
#include <iostream>

/**
 * Stable UKF Implementation using Error-State Formulation
 * 
 * Key insight: Use 15-dimensional error state for covariance
 * while maintaining 16-dimensional full state
 * 
 * This avoids quaternion normalization issues in covariance
 */
class UKF {
public:
    // State dimensions
    static constexpr int FULL_STATE_DIM = 16;  // p(3) + v(3) + q(4) + ba(3) + bg(3)
    static constexpr int ERROR_STATE_DIM = 15; // p(3) + v(3) + θ(3) + ba(3) + bg(3)
    static constexpr int NUM_SIGMA_POINTS = 2 * ERROR_STATE_DIM + 1;
    
    // State indices
    static constexpr int POS_IDX = 0;
    static constexpr int VEL_IDX = 3;
    static constexpr int ATT_IDX = 6;  // In error state, this is 3D rotation vector
    static constexpr int BA_IDX = 9;
    static constexpr int BG_IDX = 12;
    
    struct Config {
        double alpha;  // Spread of sigma points
        double beta;    // Prior knowledge (2 = Gaussian)
        double kappa;   // Secondary scaling
        
        // Process noise (standard deviations)
        double sigma_pos;      // m
        double sigma_vel;      // m/s
        double sigma_att;     // rad
        double sigma_ba;      // m/s²
        double sigma_bg;      // rad/s
        
        Config() : 
            alpha(1e-3), beta(2.0), kappa(0.0),
            sigma_pos(0.1), sigma_vel(1.0), sigma_att(0.01),
            sigma_ba(1e-4), sigma_bg(1e-5) {}
    };
    
    UKF(const Config& cfg = Config());
    
    /**
     * Initialize filter with state and covariance
     */
    void init(const State& x0, const Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& P0);
    
    /**
     * Prediction step with IMU data
     */
    void predict(const ImuSample& imu, double dt);
    
    /**
     * Update with gravity gradient measurement
     */
    void updateGradient(const Eigen::Matrix3d& measured, const Eigen::Matrix3d& R);
    
    /**
     * Update with gravity anomaly
     */
    void updateAnomaly(double measured, double noise);
    
    /**
     * Update with magnetometer (fixes heading drift!)
     */
    void updateMagnetometer(const Eigen::Vector3d& mag_body, 
                           const Eigen::Vector3d& mag_ref_ECEF,
                           const Eigen::Matrix3d& R_mag);
    
    /**
     * Zero Velocity Update - constrains velocity when stationary
     */
    void updateZUPT(const Eigen::Matrix3d& R_vel);
    
    /**
     * Barometric altitude update
     */
    void updateBarometer(double pressure_altitude, double noise);
    
    /**
     * Terrain-referenced altitude from radar altimeter
     */
    void updateTerrainAltitude(double radar_alt, double terrain_height, double noise);
    
    /**
     * Gravity anomaly map matching - provides absolute position fix
     * This is the KEY to making gravity navigation work!
     */
    void updateGravityMapMatch(const Eigen::Vector3d& matched_position_ECEF,
                               const Eigen::Matrix3d& R_position);
    
    State getState() const { return nominal_state_; }
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> getCovariance() const { return P_; }
    
    // Filter integrity monitoring
    double getNEES() const { return integrity_stats_.nees; }
    double getNIS() const { return integrity_stats_.nis; }
    double getNEESPassRate() const { return integrity_stats_.nees_pass_rate; }
    double getNISPassRate() const { return integrity_stats_.nis_pass_rate; }
    bool isFilterHealthy() const { 
        return integrity_stats_.nees_pass_rate > 0.7 && integrity_stats_.nis_pass_rate > 0.7; 
    }
    
private:
    // Nominal state (16D with quaternion)
    State nominal_state_;
    
    // Error-state covariance (15x15)
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_;
    
    // Sigma points in error-state space
    struct SigmaPoint {
        State state;  // Full state
        Eigen::Matrix<double, ERROR_STATE_DIM, 1> error;  // Error state
    };
    std::vector<SigmaPoint> sigma_points_;
    
    // UKF parameters
    Config cfg_;
    double lambda_;
    Eigen::VectorXd weights_mean_;
    Eigen::VectorXd weights_cov_;
    
    // Filter integrity monitoring using IntegrityMonitor
    IntegrityMonitor::Stats integrity_stats_;
    
    // Innovation history for adaptive noise scaling  
    std::vector<double> gradient_innovation_history_;
    std::vector<double> anomaly_innovation_history_;
    
    /**
     * Generate sigma points using error-state formulation
     */
    void generateSigmaPoints();
    
    /**
     * Propagate a single state forward
     */
    State propagateState(const State& state, const ImuSample& imu, double dt);
    
    /**
     * Compute error between two states (handles quaternion properly)
     */
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> computeError(const State& x1, const State& x2);
    
    /**
     * Apply error to state (handles quaternion properly)
     */
    State applyError(const State& nominal, const Eigen::Matrix<double, ERROR_STATE_DIM, 1>& error);
    
    /**
     * Convert rotation vector to quaternion
     */
    Eigen::Quaterniond rotationVectorToQuaternion(const Eigen::Vector3d& rot_vec);
    
    /**
     * Convert quaternion difference to rotation vector
     */
    Eigen::Vector3d quaternionToRotationVector(const Eigen::Quaterniond& q);
    
    /**
     * Ensure covariance matrix remains positive definite
     */
    void enforcePositiveDefinite(Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& P);
    
    /**
     * Compute UKF weights
     */
    void computeWeights();
};