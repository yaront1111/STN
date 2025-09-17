#pragma once

#include "types.h"
#include "ukf_math_utils.h"
#include <Eigen/Dense>

/**
 * ENU State Manager
 *
 * Manages the local tangent plane representation to maintain numerical stability.
 * The UKF operates in ENU coordinates while maintaining the ECEF anchor point.
 *
 * This solves the scale mismatch problem:
 * - ECEF: ~10^6 m (causes conditioning issues)
 * - ENU: ~10^3 m (numerically stable)
 */
class ENUStateManager {
public:
    ENUStateManager() : is_initialized_(false) {}

    /**
     * Initialize with an ECEF position as the anchor point
     */
    void initialize(const Eigen::Vector3d& anchor_ECEF) {
        anchor_ECEF_ = anchor_ECEF;

        // Compute ENU to ECEF rotation at anchor
        Eigen::Vector3d lla = UKFMathUtils::ecefToLla(anchor_ECEF);
        double lat = lla(0);
        double lon = lla(1);

        double sin_lat = std::sin(lat);
        double cos_lat = std::cos(lat);
        double sin_lon = std::sin(lon);
        double cos_lon = std::cos(lon);

        // Rotation matrix from ENU to ECEF
        R_ECEF_ENU_ << -sin_lon,         -sin_lat*cos_lon,  cos_lat*cos_lon,
                        cos_lon,         -sin_lat*sin_lon,  cos_lat*sin_lon,
                        0,                cos_lat,           sin_lat;

        is_initialized_ = true;
    }

    /**
     * Convert state from ECEF to ENU representation
     */
    State ecefToEnu(const State& state_ECEF) const {
        assert(is_initialized_ && "ENUStateManager not initialized");

        State state_ENU;

        // Position: convert to ENU relative to anchor
        Eigen::Vector3d delta_ECEF = state_ECEF.p_ECEF - anchor_ECEF_;
        state_ENU.p_ECEF = R_ECEF_ENU_.transpose() * delta_ECEF;  // Actually p_ENU

        // Velocity: rotate to ENU frame
        state_ENU.v_ECEF = R_ECEF_ENU_.transpose() * state_ECEF.v_ECEF;  // Actually v_ENU

        // Quaternion: compose with ENU rotation
        // q_ENU_B = q_ENU_ECEF * q_ECEF_B
        Eigen::Quaterniond q_ENU_ECEF(R_ECEF_ENU_.transpose());
        state_ENU.q_ECEF_B = q_ENU_ECEF * state_ECEF.q_ECEF_B;  // Actually q_ENU_B

        // Biases remain the same
        state_ENU.b_a = state_ECEF.b_a;
        state_ENU.b_g = state_ECEF.b_g;

        return state_ENU;
    }

    /**
     * Convert state from ENU back to ECEF representation
     */
    State enuToEcef(const State& state_ENU) const {
        assert(is_initialized_ && "ENUStateManager not initialized");

        State state_ECEF;

        // Position: convert from ENU to ECEF
        Eigen::Vector3d delta_ENU = state_ENU.p_ECEF;  // Actually p_ENU
        state_ECEF.p_ECEF = anchor_ECEF_ + R_ECEF_ENU_ * delta_ENU;

        // Velocity: rotate to ECEF frame
        state_ECEF.v_ECEF = R_ECEF_ENU_ * state_ENU.v_ECEF;  // From v_ENU

        // Quaternion: remove ENU rotation
        // q_ECEF_B = q_ECEF_ENU * q_ENU_B
        Eigen::Quaterniond q_ECEF_ENU(R_ECEF_ENU_);
        state_ECEF.q_ECEF_B = q_ECEF_ENU * state_ENU.q_ECEF_B;  // From q_ENU_B

        // Biases remain the same
        state_ECEF.b_a = state_ENU.b_a;
        state_ECEF.b_g = state_ENU.b_g;

        return state_ECEF;
    }

    /**
     * Re-anchor if position drift exceeds threshold
     * This prevents ENU coordinates from growing too large
     */
    bool shouldReanchor(const State& state_ENU, double threshold_m = 10000.0) const {
        return state_ENU.p_ECEF.norm() > threshold_m;  // p_ECEF is actually p_ENU
    }

    /**
     * Re-anchor to current position
     */
    void reanchor(const State& state_ECEF) {
        // Update anchor to current position
        initialize(state_ECEF.p_ECEF);
    }

    /**
     * Get current anchor point
     */
    const Eigen::Vector3d& getAnchor() const { return anchor_ECEF_; }

    /**
     * Check if initialized
     */
    bool isInitialized() const { return is_initialized_; }

    /**
     * Transform covariance from ECEF to ENU
     */
    Eigen::Matrix<double, 15, 15> transformCovarianceToEnu(
        const Eigen::Matrix<double, 15, 15>& P_ECEF) const {

        Eigen::Matrix<double, 15, 15> P_ENU = P_ECEF;

        // Transform position covariance
        P_ENU.block<3,3>(0,0) = R_ECEF_ENU_.transpose() *
                                P_ECEF.block<3,3>(0,0) * R_ECEF_ENU_;

        // Transform velocity covariance
        P_ENU.block<3,3>(3,3) = R_ECEF_ENU_.transpose() *
                                P_ECEF.block<3,3>(3,3) * R_ECEF_ENU_;

        // Transform cross-covariances
        P_ENU.block<3,3>(0,3) = R_ECEF_ENU_.transpose() *
                                P_ECEF.block<3,3>(0,3) * R_ECEF_ENU_;
        P_ENU.block<3,3>(3,0) = R_ECEF_ENU_.transpose() *
                                P_ECEF.block<3,3>(3,0) * R_ECEF_ENU_;

        // Attitude and biases remain in body frame

        return P_ENU;
    }

private:
    Eigen::Vector3d anchor_ECEF_;      // Anchor point in ECEF
    Eigen::Matrix3d R_ECEF_ENU_;       // Rotation from ENU to ECEF
    bool is_initialized_;
};