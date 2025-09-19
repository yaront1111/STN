#pragma once
#include "types.h"
#include "integrity.hpp"
#include "ukf_config.h"
#include "ukf_math_utils.h"
#include "ukf_state_scaler_v2.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <memory>

// Forward declarations for modular architecture
class UKFSigmaPoints;
class UKFMeasurements;
class GravityGradientProvider;

/**
 * Stable UKF Implementation using Error-State Formulation
 * 
 * Key insight: Use 15-dimensional error state for covariance
 * while maintaining 16-dimensional full state
 * 
 * This avoids quaternion normalization issues in covariance
 */
class UKF {
    // Friend classes for modular architecture
    friend class UKFSigmaPoints;
    friend class UKFMeasurements;
    
public:
    // State dimensions
    static constexpr int FULL_STATE_DIM = 16;  // p(3) + v(3) + q(4) + ba(3) + bg(3)
    static constexpr int ERROR_STATE_DIM = 15; // p(3) + v(3) + θ(3) + ba(3) + bg(3)
    static constexpr int NUM_SIGMA_POINTS = 2 * ERROR_STATE_DIM + 1;

    // ENU frame operation for numerical stability
    static constexpr double ENU_REANCHOR_THRESHOLD = 10000.0;  // Re-anchor when drift exceeds 10km
    
    // State indices
    static constexpr int POS_IDX = 0;
    static constexpr int VEL_IDX = 3;
    static constexpr int ATT_IDX = 6;  // In error state, this is 3D rotation vector
    static constexpr int BA_IDX = 9;
    static constexpr int BG_IDX = 12;
    
    // Note: SigmaPoint struct is defined in ukf_sigma_points.h
    
    // Use centralized configuration
    UKF(const UKFConfig& cfg = UKFConfig());
    ~UKF();
    
    /**
     * Initialize filter with state and covariance
     * @param use_enu: Enable ENU frame operation for numerical stability
     */
    void init(const State& x0_ECEF, const Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& P0,
              bool use_enu = true);

    /**
     * Set gravity provider for measurement predictions
     */
    void setGravityProvider(GravityGradientProvider* provider);
    
    /**
     * Prediction step with IMU data
     */
    void predict(const ImuSample& imu, double dt);
    
    /**
     * Update with gravity gradient measurement
     */
    void updateGradient(const Eigen::Matrix3d& measured, const Eigen::Matrix3d& R);
    
    /**
     * Update with gravity gradient invariants (2D eigenvalue measurement)
     * More robust than full 9D tensor - reduces attitude coupling and improves acceptance
     */
    void updateGradientInvariants(const Eigen::Matrix3d& measured_tensor, const Eigen::Matrix2d& R_invariants);
    
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
     * Update with Doppler velocity measurement (critical for velocity observability!)
     */
    void updateDopplerVelocity(const Eigen::Vector3d& measured_velocity_body, 
                              const Eigen::Matrix3d& R_doppler);
    
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
    
    /**
     * Kinematic pseudo-measurements for velocity constraints
     */
    // Pseudo-measurements removed - not used in production
    
    State getState() const {
        return use_enu_ ? enuToEcef(nominal_state_) : nominal_state_;
    }
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> getCovariance() const {
        // Return P = D * S_scaled * S_scaled^T * D' from the scaled Cholesky factor
        auto S_real = state_scaler_.unscaleCholeskyFactor(S_scaled_);
        return S_real * S_real.transpose();
    }

    // Square-root specific accessors
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> getCholeksyFactor() const {
        // Return the unscaled (real-world) Cholesky factor
        return state_scaler_.unscaleCholeskyFactor(S_scaled_);
    }
    void setCholeskyFactor(const Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& S) {
        // Store as scaled Cholesky factor
        S_scaled_ = state_scaler_.scaleCholeskyFactor(S);
    }

    // Get scaled Cholesky factor (for internal use)
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> getScaledCholeskyFactor() const {
        return S_scaled_;
    }

    // Get state scaler (for access from sigma points and measurements)
    const UKFStateScalerV2& getStateScaler() const { return state_scaler_; }
    UKFStateScalerV2& getStateScaler() { return state_scaler_; }

    // Getters for modular components
    const Eigen::VectorXd& getWeightsMean() const { return weights_mean_; }
    const Eigen::VectorXd& getWeightsCov() const { return weights_cov_; }
    double getLambda() const { return lambda_; }
    UKFSigmaPoints& getSigmaManager() { return *sigma_points_manager_; }
    
    // Update state and covariance (for measurements)
    void setState(const State& state) { nominal_state_ = state; }
    void setCovariance(const Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& P) {
        // Scale covariance to scaled space first
        auto P_scaled = state_scaler_.scaleCovariance(P);

        // Convert scaled covariance to Cholesky factor
        Eigen::LLT<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> llt(P_scaled);
        if (llt.info() == Eigen::Success) {
            S_scaled_ = llt.matrixL();
        } else {
            // Fallback: enforce positive definiteness first
            Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_safe = P_scaled;
            UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P_safe, 1e-9);
            Eigen::LLT<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> llt_safe(P_safe);
            S_scaled_ = llt_safe.matrixL();
        }
    }
    
    // Filter integrity monitoring
    double getNEES() const { return integrity_stats_.nees; }
    double getNIS() const { return integrity_stats_.nis; }
    double getNEESPassRate() const { return integrity_stats_.nees_pass_rate; }
    double getNISPassRate() const { return integrity_stats_.nis_pass_rate; }
    bool isFilterHealthy() const { 
        return integrity_stats_.nees_pass_rate > 0.7 && integrity_stats_.nis_pass_rate > 0.7; 
    }

protected:
    // Nominal state (16D with quaternion)
    State nominal_state_;

    // SCALED Square-root covariance (Cholesky factor S_scaled where P_scaled = S_scaled*S_scaled^T)
    // This is the Cholesky factor in SCALED SPACE for numerical stability
    // Real covariance: P = D * P_scaled * D' = D * S_scaled * S_scaled' * D'
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> S_scaled_;

    // State scaler for numerical stability
    UKFStateScalerV2 state_scaler_;

    // UKF configuration and parameters
    UKFConfig cfg_;
    double lambda_;
    Eigen::VectorXd weights_mean_;
    Eigen::VectorXd weights_cov_;

    // Modular components (using unique_ptr)
    std::unique_ptr<UKFSigmaPoints> sigma_points_manager_;
    std::unique_ptr<UKFMeasurements> measurements_manager_;
    GravityGradientProvider* gravity_provider_ = nullptr;  // Non-owning pointer

    // Process noise matrix
    Eigen::Matrix<double, 15, 15> Q_;

    // Innovation statistics
    double last_nis_ = 0.0;

    // ENU frame operation
    bool use_enu_ = true;
    Eigen::Vector3d enu_anchor_ECEF_;  // Anchor point in ECEF
    Eigen::Matrix3d R_ECEF_ENU_;       // Rotation from ENU to ECEF

public:
    // State conversion methods (made public for UKFMeasurements access)
    State ecefToEnu(const State& state_ECEF) const;
    State enuToEcef(const State& state_ENU) const;

protected:
    void initializeEnu(const Eigen::Vector3d& anchor_ECEF);
    void checkAndReanchor();

    // Square-root specific process noise addition
    void addProcessNoiseSquareRoot(double dt);

private:

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
     * Add process noise to covariance matrix
     */
    void addProcessNoise(double dt);
    
    /**
     * Compute UKF weights
     */
    void computeWeights();
    
    /**
     * Utility functions for gradient invariants
     */
    Eigen::Vector2d computeTensorInvariants(const Eigen::Matrix3d& tensor);
    void performRobustUpdate(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& S, 
                           const Eigen::MatrixXd& T_cross, const Eigen::MatrixXd& R);
};