#include "ukf_sigma_points.h"
#include "ukf.h"
#include "ukf_math_utils.h"
#include "gravity_gradient_provider.h"
#include <iostream>

UKFSigmaPoints::UKFSigmaPoints(UKF& ukf, GravityGradientProvider* gravity_provider)
    : ukf_(ukf), gravity_provider_(gravity_provider) {
    sigma_points_.resize(UKF_NUM_SIGMA_POINTS);
}

void UKFSigmaPoints::generate(const State& nominal_state,
                              const Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM>& P,
                              double lambda) {
    // First make a copy and enforce positive definiteness
    Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM> P_safe = P;

    // Enforce symmetry and positive definiteness
    P_safe = 0.5 * (P_safe + P_safe.transpose());
    UKFMathUtils::enforcePositiveDefinite<UKF_ERROR_STATE_DIM>(P_safe, 1e-9);

    // Check if the covariance is still invalid after fixing
    if (!UKFMathUtils::checkMatrixValidity<UKF_ERROR_STATE_DIM>(P_safe)) {
        std::cerr << "WARNING: Resetting invalid covariance matrix in sigma point generation!\n";
        // Reset to a safe diagonal matrix if still invalid
        // Use balanced values appropriate for ENU operation
        P_safe = Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM>::Identity();
        P_safe.block<3,3>(0,0) *= 1.0;     // Position: 1 m²
        P_safe.block<3,3>(3,3) *= 0.1;     // Velocity: 0.1 m²/s²
        P_safe.block<3,3>(6,6) *= 0.01;    // Attitude: 0.01 rad²
        P_safe.block<3,3>(9,9) *= 0.001;   // Accel bias: 0.001 m²/s⁴
        P_safe.block<3,3>(12,12) *= 0.0001; // Gyro bias: 0.0001 rad²/s²
    }

    // Compute matrix square root using robust method
    Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM> sqrt_P =
        computeMatrixSqrt(P_safe);

    double sqrt_factor = std::sqrt(UKF_ERROR_STATE_DIM + lambda);

    // Generate sigma points
    // Central sigma point (index 0)
    sigma_points_[0] = SigmaPoint(nominal_state, 0, 0);  // weights will be set later

    // Positive and negative sigma points
    for (int i = 0; i < UKF_ERROR_STATE_DIM; ++i) {
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, 1> offset =
            sqrt_factor * sqrt_P.col(i);

        // Apply error to nominal state using UKF's method
        State pos_state = UKFMathUtils::applyErrorToState(nominal_state, offset);
        State neg_state = UKFMathUtils::applyErrorToState(nominal_state, -offset);

        // Positive sigma point (index i+1)
        sigma_points_[i + 1] = SigmaPoint(pos_state, 0, 0);  // weights will be set later

        // Negative sigma point (index i+1+ERROR_STATE_DIM)
        sigma_points_[i + 1 + UKF_ERROR_STATE_DIM] = SigmaPoint(neg_state, 0, 0);
    }
}

std::vector<State> UKFSigmaPoints::propagateStates(const ImuSample& imu, double dt) {
    std::vector<State> propagated_states;
    propagated_states.reserve(sigma_points_.size());

    // Earth's rotation vector in ECEF frame (rad/s)
    const Eigen::Vector3d omega_ie(0.0, 0.0, 7.292115e-5);

    for (const auto& sigma_point : sigma_points_) {
        State propagated = sigma_point.state;

        // --- ATTITUDE UPDATE ---
        // Corrected gyro measurement (remove bias)
        Eigen::Vector3d corrected_gyro = imu.gyro_rps - propagated.b_g;
        Eigen::Vector3d omega_dt = corrected_gyro * dt;
        if (omega_dt.norm() > 1e-10) {
            Eigen::Quaterniond delta_q = UKFMathUtils::rotationVectorToQuaternion(omega_dt);
            propagated.q_ECEF_B = propagated.q_ECEF_B * delta_q;
            propagated.q_ECEF_B.normalize();
        }

        // --- ACCELERATION COMPUTATION WITH COMPLETE PHYSICS ---
        // 1. Corrected specific force (remove accelerometer bias)
        Eigen::Vector3d acc_body = imu.acc_mps2 - propagated.b_a;

        // 2. Transform to ECEF/ENU frame
        Eigen::Vector3d acc_inertial = propagated.q_ECEF_B * acc_body;

        // 3. Get gravity from model (if available) or use simplified model
        Eigen::Vector3d gravity_vec;
        if (gravity_provider_ && !ukf_.use_enu_) {
            // In ECEF mode with gravity model:
            // Get anomaly and compute total gravity
            double anomaly_eotvos = gravity_provider_->getAnomaly(sigma_point.state.p_ECEF);
            double anomaly_mps2 = anomaly_eotvos * 1e-9;  // Convert from E to m/s²

            // Get normal gravity magnitude
            Eigen::Vector3d lla = UKFMathUtils::ecefToLla(sigma_point.state.p_ECEF);
            double normal_g = UKFMathUtils::getNormalGravity(lla(0));  // latitude in radians

            // Total gravity magnitude
            double total_g = normal_g + anomaly_mps2;

            // Direction: radial inward (simplified, should use ellipsoid normal)
            double r = sigma_point.state.p_ECEF.norm();
            gravity_vec = -total_g * (sigma_point.state.p_ECEF / r);
        } else if (ukf_.use_enu_) {
            // In ENU, gravity is downward
            gravity_vec = Eigen::Vector3d(0, 0, -9.81);
        } else {
            // Simplified gravity in ECEF (radial inward)
            double r = sigma_point.state.p_ECEF.norm();
            gravity_vec = -9.81 * (sigma_point.state.p_ECEF / r);
        }

        // 4. Compute Coriolis and centripetal accelerations (only in ECEF)
        Eigen::Vector3d coriolis_accel = Eigen::Vector3d::Zero();
        Eigen::Vector3d centripetal_accel = Eigen::Vector3d::Zero();

        if (!ukf_.use_enu_) {
            // Coriolis: -2 * Ω × v
            coriolis_accel = -2.0 * omega_ie.cross(sigma_point.state.v_ECEF);

            // Centripetal: -Ω × (Ω × r)
            centripetal_accel = -omega_ie.cross(omega_ie.cross(sigma_point.state.p_ECEF));
        }

        // 5. Total acceleration in frame
        Eigen::Vector3d total_accel = acc_inertial + gravity_vec + coriolis_accel + centripetal_accel;

        // --- POSITION AND VELOCITY UPDATE ---
        // Semi-implicit Euler integration for better stability
        propagated.v_ECEF = sigma_point.state.v_ECEF + total_accel * dt;
        propagated.p_ECEF = sigma_point.state.p_ECEF + sigma_point.state.v_ECEF * dt +
                           0.5 * total_accel * dt * dt;

        // Biases remain constant (random walk handled by process noise)
        propagated.b_a = sigma_point.state.b_a;
        propagated.b_g = sigma_point.state.b_g;

        propagated.t = sigma_point.state.t + dt;
        propagated_states.push_back(propagated);
    }
    
    return propagated_states;
}

State UKFSigmaPoints::computeMeanState(const std::vector<State>& states, 
                                      const Eigen::VectorXd& weights_mean) {
    if (states.empty()) {
        return State();
    }
    
    State mean_state;
    
    // Mean position
    mean_state.p_ECEF = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < states.size(); ++i) {
        mean_state.p_ECEF += weights_mean(i) * states[i].p_ECEF;
    }
    
    // Mean velocity  
    mean_state.v_ECEF = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < states.size(); ++i) {
        mean_state.v_ECEF += weights_mean(i) * states[i].v_ECEF;
    }
    
    // Mean quaternion (requires special handling)
    std::vector<Eigen::Quaterniond> quaternions;
    for (const auto& state : states) {
        quaternions.push_back(state.q_ECEF_B);
    }
    mean_state.q_ECEF_B = computeMeanQuaternion(states, weights_mean);
    
    // Mean biases
    mean_state.b_a = Eigen::Vector3d::Zero();
    mean_state.b_g = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < states.size(); ++i) {
        mean_state.b_a += weights_mean(i) * states[i].b_a;
        mean_state.b_g += weights_mean(i) * states[i].b_g;
    }
    
    return mean_state;
}

Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM> 
UKFSigmaPoints::computeCovariance(const std::vector<State>& states,
                                  const State& mean_state,
                                  const Eigen::VectorXd& weights_cov) {
    Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM> P = 
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM>::Zero();
    
    for (size_t i = 0; i < states.size(); ++i) {
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, 1> error = 
            UKFMathUtils::computeErrorVector(states[i], mean_state);
        P += weights_cov(i) * error * error.transpose();
    }
    
    // Ensure positive definiteness
    UKFMathUtils::enforcePositiveDefinite<UKF_ERROR_STATE_DIM>(P);
    
    return P;
}

Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM> 
UKFSigmaPoints::computeMatrixSqrt(const Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM>& matrix) {
    // Use the robust matrix sqrt from UKFMathUtils
    return UKFMathUtils::robustMatrixSqrt<UKF_ERROR_STATE_DIM>(matrix);
}

Eigen::Quaterniond UKFSigmaPoints::computeMeanQuaternion(const std::vector<State>& states,
                                                        const Eigen::VectorXd& weights_mean) {
    std::vector<Eigen::Quaterniond> quaternions;
    quaternions.reserve(states.size());
    
    for (const auto& state : states) {
        quaternions.push_back(state.q_ECEF_B);
    }
    
    return UKFMathUtils::averageQuaternions(quaternions, weights_mean);
}