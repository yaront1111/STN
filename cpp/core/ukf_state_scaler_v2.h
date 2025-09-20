#pragma once

#include <Eigen/Dense>
#include "types.h"

/**
 * UKF State Scaler V2 - Uncertainty-Based Scaling
 *
 * CRITICAL PARADIGM SHIFT: Scale the uncertainty, not the state!
 *
 * The goal is to normalize the covariance matrix, not the state vector.
 * Scale factors are based on expected standard deviations (uncertainties),
 * not on the magnitudes of the state components.
 *
 * This ensures the scaled covariance matrix P_scaled has diagonal elements ~1.0,
 * giving excellent numerical conditioning throughout the filter.
 */
class UKFStateScalerV2 {
public:
    static constexpr int STATE_DIM = 15;

    // Expected standard deviations (uncertainties) for each state component
    // These represent typical uncertainties, NOT state magnitudes
    static constexpr double SIGMA_POSITION = 10.0;      // 10m typical position uncertainty
    static constexpr double SIGMA_VELOCITY = 1.0;       // 1 m/s typical velocity uncertainty
    static constexpr double SIGMA_ATTITUDE = 0.1;       // 0.1 rad typical attitude uncertainty
    static constexpr double SIGMA_ACCEL_BIAS = 1e-3;    // 1 mm/s² typical accel bias uncertainty
    static constexpr double SIGMA_GYRO_BIAS = 1e-5;     // 1e-5 rad/s typical gyro bias uncertainty

    UKFStateScalerV2();

    // Scale an error vector from physical to normalized space
    // dx_scaled = D_inv * dx_physical
    Eigen::Matrix<double, STATE_DIM, 1> scaleError(
        const Eigen::Matrix<double, STATE_DIM, 1>& dx_physical) const;

    // Unscale an error vector from normalized to physical space
    // dx_physical = D * dx_scaled
    Eigen::Matrix<double, STATE_DIM, 1> unscaleError(
        const Eigen::Matrix<double, STATE_DIM, 1>& dx_scaled) const;

    // Scale a covariance matrix from physical to normalized space
    // P_scaled = D_inv * P_physical * D_inv'
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> scaleCovariance(
        const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P_physical) const;

    // Unscale a covariance matrix from normalized to physical space
    // P_physical = D * P_scaled * D'
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> unscaleCovariance(
        const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P_scaled) const;

    // Scale a Cholesky factor from physical to normalized space
    // S_scaled = D_inv * S_physical
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> scaleCholeskyFactor(
        const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& S_physical) const;

    // Unscale a Cholesky factor from normalized to physical space
    // S_physical = D * S_scaled
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> unscaleCholeskyFactor(
        const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& S_scaled) const;

    // Scale process noise standard deviations
    Eigen::Matrix<double, STATE_DIM, 1> scaleProcessNoise(
        const Eigen::Matrix<double, STATE_DIM, 1>& q_std_physical) const;

    // Scale a measurement Jacobian matrix
    // H_scaled = H_physical * D (chain rule: dh/dx_scaled = dh/dx_physical * dx_physical/dx_scaled)
    template<int MEAS_DIM>
    Eigen::Matrix<double, MEAS_DIM, STATE_DIM> scaleJacobian(
        const Eigen::Matrix<double, MEAS_DIM, STATE_DIM>& H_physical) const {
        return H_physical * D_;
    }

    // Get the scaling matrix D
    const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& getScalingMatrix() const {
        return D_;
    }

    // Get the inverse scaling matrix D_inv
    const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& getInverseScalingMatrix() const {
        return D_inv_;
    }

    // Get the diagonal scaling factors as a vector
    Eigen::Matrix<double, STATE_DIM, 1> getScales() const {
        return D_.diagonal();
    }

    // Compute condition number of a matrix
    double computeConditionNumber(const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& M) const;

    // Check if scaled covariance is well-conditioned
    bool isWellConditioned(const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P_scaled) const;

private:
    // Diagonal scaling matrices
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> D_;      // D = diag(σ_i)
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> D_inv_;  // D_inv = diag(1/σ_i)

    void initializeScalingMatrices();
};