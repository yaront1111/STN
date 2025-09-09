#pragma once
#include "types.h"
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
    
    // Filter integrity monitoring (NEES/NIS)
    struct IntegrityStats {
        double nees = 0.0;          // Normalized Estimation Error Squared
        double nis = 0.0;           // Normalized Innovation Squared  
        int nees_count = 0;         // NEES sample count
        int nis_count = 0;          // NIS sample count
        double nees_pass_rate = 1.0; // NEES chi-square test pass rate
        double nis_pass_rate = 1.0;  // NIS chi-square test pass rate
        std::vector<double> recent_nees; // Recent NEES values for statistics
        std::vector<double> recent_nis;  // Recent NIS values for statistics
        
        void addNEES(double nees_val, int dof) {
            recent_nees.push_back(nees_val);
            if (recent_nees.size() > 100) recent_nees.erase(recent_nees.begin());
            nees_count++;
            
            // Chi-square test (95% confidence)
            double chi2_95 = (dof == 15) ? 24.996 : 9.488;  // 15 DOF or 3 DOF
            bool pass = (nees_val < chi2_95);
            nees_pass_rate = 0.95 * nees_pass_rate + 0.05 * (pass ? 1.0 : 0.0);
        }
        
        void addNIS(double nis_val, int dof) {
            recent_nis.push_back(nis_val);  
            if (recent_nis.size() > 100) recent_nis.erase(recent_nis.begin());
            nis_count++;
            
            // Chi-square test (99% confidence for measurements)
            double chi2_99 = (dof == 9) ? 21.666 : (dof == 3) ? 11.345 : 9.21;
            bool pass = (nis_val < chi2_99);
            nis_pass_rate = 0.95 * nis_pass_rate + 0.05 * (pass ? 1.0 : 0.0);
        }
    } integrity_stats_;
    
    // Adaptive measurement noise
    struct AdaptiveNoise {
        Eigen::Matrix<double, 9, 9> R_gradient_base;  // Base gradient noise
        double R_scale_factor = 1.0;                  // Adaptive scaling  
        std::vector<double> innovation_history;        // Recent innovations
        int adaptation_window = 50;                    // Window for adaptation
        
        void updateScale(double innovation_magnitude) {
            innovation_history.push_back(innovation_magnitude);
            if (innovation_history.size() > adaptation_window) {
                innovation_history.erase(innovation_history.begin());
            }
            
            // Compute innovation variance
            if (innovation_history.size() > 10) {
                double mean = 0.0;
                for (double val : innovation_history) mean += val;
                mean /= innovation_history.size();
                
                double var = 0.0;
                for (double val : innovation_history) var += (val - mean) * (val - mean);
                var /= (innovation_history.size() - 1);
                
                // Adaptive scaling based on innovation statistics
                R_scale_factor = std::max(0.1, std::min(10.0, var / 1.0));  // Clamp between 0.1x and 10x
            }
        }
    } adaptive_noise_;
    
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