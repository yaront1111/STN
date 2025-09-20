#pragma once

#include "types.h"
#include "ukf_config.h"
#include "ukf_state_scaler_v2.h"
#include "ukf_math_utils.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>

// Forward declarations
class GravityGradientProvider;

/**
 * Scaled Square-Root Unscented Kalman Filter
 *
 * Complete rewrite with proper scaling architecture.
 * ALL internal operations happen in scaled space where covariance is well-conditioned.
 * Physical units only appear at input/output boundaries.
 *
 * Key principles:
 * 1. Internal state S_ is the SCALED Cholesky factor
 * 2. Sigma point generation happens in SCALED space
 * 3. Error vectors are transformed at boundaries
 * 4. All matrix operations use well-conditioned scaled matrices
 */
class ScaledUKF {
public:
    static constexpr int ERROR_STATE_DIM = 15;
    static constexpr int NUM_SIGMA_POINTS = 2 * ERROR_STATE_DIM + 1;

    // Constructor
    ScaledUKF(const UKFConfig& config = UKFConfig());
    ~ScaledUKF() = default;

    /**
     * Initialize filter with physical state and covariance
     * @param x0_physical Initial state in physical units (ECEF)
     * @param P0_physical Initial covariance in physical units
     */
    void init(const State& x0_physical,
             const Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& P0_physical);

    /**
     * Set gravity provider for measurement predictions
     */
    void setGravityProvider(GravityGradientProvider* provider) {
        gravity_provider_ = provider;
    }

    /**
     * Prediction step with IMU data
     */
    void predict(const ImuSample& imu, double dt);

    /**
     * Update with gravity gradient measurement
     */
    void updateGradient(const Eigen::Matrix3d& measured_gradient, const Eigen::Matrix3d& R);

    /**
     * Update with gravity anomaly measurement
     */
    void updateAnomaly(double measured_anomaly_mgal, double noise_mgal);

    /**
     * Update with map match position fix
     */
    void updateMapMatch(const Eigen::Vector3d& matched_position_ECEF, double uncertainty_m);

    /**
     * Update with magnetometer measurement
     */
    void updateMagnetometer(const Eigen::Vector3d& mag_body,
                           const Eigen::Vector3d& mag_ref_ECEF,
                           const Eigen::Matrix3d& R_mag);

    /**
     * Update with barometric altitude
     */
    void updateBarometer(double altitude_msl, double noise_variance);

    /**
     * Zero Velocity Update (ZUPT) - constrains velocity when stationary
     */
    void updateZUPT(const Eigen::Matrix3d& R_vel);

    /**
     * Get current state estimate (physical units)
     */
    State getState() const { return nominal_state_; }

    /**
     * Get current covariance (physical units)
     */
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> getCovariance() const;

    /**
     * Get scaled Cholesky factor (for debugging)
     */
    const Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& getScaledCholeskyFactor() const {
        return S_;
    }

    /**
     * Check filter health
     */
    bool isHealthy() const;

private:
    // ========== Core State ==========
    State nominal_state_;  // Physical state (unscaled)
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> S_;  // SCALED Cholesky factor

    // ========== Scaling ==========
    UKFStateScalerV2 scaler_;

    // ========== Configuration ==========
    UKFConfig config_;
    double lambda_;
    double alpha_;
    double beta_;
    Eigen::VectorXd weights_mean_;
    Eigen::VectorXd weights_cov_;

    // ========== Providers ==========
    GravityGradientProvider* gravity_provider_ = nullptr;

    // ========== Sigma Points ==========
    struct ScaledSigmaPoint {
        State state;  // Physical state
        Eigen::Matrix<double, ERROR_STATE_DIM, 1> error_scaled;  // Scaled error from nominal
    };
    std::vector<ScaledSigmaPoint> sigma_points_;

    // ========== Health Monitoring ==========
    mutable double last_condition_number_ = 1.0;
    mutable int divergence_count_ = 0;

    // ========== Helper Methods ==========

    /**
     * Initialize UKF weights
     */
    void computeWeights();

    /**
     * Generate sigma points (boundary transformation)
     */
    void generateSigmaPoints();

    /**
     * Propagate a single state through dynamics
     */
    State propagateState(const State& state, const ImuSample& imu, double dt) const;

    /**
     * Compute mean of propagated states
     */
    State computeMeanState(const std::vector<State>& states) const;

    /**
     * Reconstruct scaled covariance from propagated states
     * This is where the magic happens - all in scaled space!
     */
    void reconstructScaledCovariance(const std::vector<State>& propagated_states,
                                    const State& mean_state);

    /**
     * Add process noise in scaled space
     */
    void addProcessNoiseScaled(double dt);

    /**
     * Perform measurement update in scaled space
     */
    template<int MEAS_DIM>
    void measurementUpdateScaled(
        const Eigen::Matrix<double, MEAS_DIM, 1>& innovation_physical,
        const Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>& R_physical,
        const Eigen::Matrix<double, ERROR_STATE_DIM, MEAS_DIM>& Txz_physical);

    /**
     * Compute error vector between two states (physical units)
     */
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> computeErrorPhysical(
        const State& state1, const State& state2) const;

    /**
     * Apply error vector to state (physical units)
     */
    State applyErrorPhysical(const State& nominal,
                            const Eigen::Matrix<double, ERROR_STATE_DIM, 1>& error_physical) const;

    /**
     * Monitor numerical health
     */
    void monitorHealth() const;
};