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
                              const Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM>& S_scaled,
                              double lambda) {
    // S_scaled is the SCALED Cholesky factor for numerical stability
    // We need to unscale the offsets before applying to the state

    double sqrt_factor = std::sqrt(UKF_ERROR_STATE_DIM + lambda);

    // Get the state scaler from UKF
    const auto& scaler = ukf_.getStateScaler();

    // Generate sigma points
    // Central sigma point (index 0)
    sigma_points_[0] = SigmaPoint(nominal_state, 0, 0);  // weights will be set later

    // Positive and negative sigma points
    for (int i = 0; i < UKF_ERROR_STATE_DIM; ++i) {
        // Get column from SCALED Cholesky factor
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, 1> offset_scaled =
            sqrt_factor * S_scaled.col(i);

        // CRITICAL: Unscale the offset to real-world units before applying to state
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, 1> offset_real =
            scaler.unscaleError(offset_scaled);

        // Apply error to nominal state using real-world offset
        State pos_state = UKFMathUtils::applyErrorToState(nominal_state, offset_real);
        State neg_state = UKFMathUtils::applyErrorToState(nominal_state, -offset_real);

        // Positive sigma point (index i+1)
        sigma_points_[i + 1] = SigmaPoint(pos_state, 0, 0);  // weights will be set later

        // Negative sigma point (index i+1+ERROR_STATE_DIM)
        sigma_points_[i + 1 + UKF_ERROR_STATE_DIM] = SigmaPoint(neg_state, 0, 0);
    }
}

void UKFSigmaPoints::generateFromCovariance(const State& nominal_state,
                                           const Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM>& P,
                                           double lambda) {
    // Convert P to Cholesky factor and call the main method
    Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM> P_safe = P;
    P_safe = 0.5 * (P_safe + P_safe.transpose());
    UKFMathUtils::enforcePositiveDefinite<UKF_ERROR_STATE_DIM>(P_safe, 1e-9);

    Eigen::LLT<Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM>> llt(P_safe);
    if (llt.info() == Eigen::Success) {
        generate(nominal_state, llt.matrixL(), lambda);
    } else {
        std::cerr << "WARNING: Failed to compute Cholesky in generateFromCovariance, using identity!\n";
        generate(nominal_state, Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM>::Identity(), lambda);
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

Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM>
UKFSigmaPoints::computeCholeskyFactor(const std::vector<State>& states,
                                     const State& mean_state,
                                     const Eigen::VectorXd& weights_cov) {
    // Compute Cholesky factor in SCALED space for numerical stability
    // This is critical for maintaining good condition numbers

    // Get the state scaler from UKF
    const auto& scaler = ukf_.getStateScaler();

    int num_sigma = states.size();

    // First, handle the central point separately
    Eigen::Matrix<double, UKF_ERROR_STATE_DIM, 1> error0_real =
        UKFMathUtils::computeErrorVector(states[0], mean_state);

    // CRITICAL: Scale the error to scaled space
    Eigen::Matrix<double, UKF_ERROR_STATE_DIM, 1> error0_scaled =
        scaler.scaleError(error0_real);

    // For UKF, w_0^c is typically negative, but we need special handling
    double w0 = weights_cov(0);

    // Build the matrix of weighted deviations in SCALED space
    Eigen::Matrix<double, UKF_ERROR_STATE_DIM, Eigen::Dynamic> A(UKF_ERROR_STATE_DIM, num_sigma - 1);

    for (int i = 1; i < num_sigma; ++i) {
        // Compute error in real-world units
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, 1> error_real =
            UKFMathUtils::computeErrorVector(states[i], mean_state);

        // CRITICAL: Scale to scaled space
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, 1> error_scaled =
            scaler.scaleError(error_real);

        // All non-central weights should be positive
        double sqrt_weight = std::sqrt(std::abs(weights_cov(i)));
        A.col(i-1) = sqrt_weight * error_scaled;
    }

    // Perform QR decomposition on positive weight contributions (in scaled space)
    Eigen::HouseholderQR<Eigen::Matrix<double, UKF_ERROR_STATE_DIM, Eigen::Dynamic>> qr(A);

    // Extract R matrix (upper triangular) in scaled space
    Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM> R_scaled =
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM>::Zero();

    // Only extract the square part that exists
    int rank = std::min((int)A.cols(), (int)UKF_ERROR_STATE_DIM);
    if (rank > 0) {
        R_scaled.topLeftCorner(rank, rank) = qr.matrixQR()
            .topLeftCorner(rank, rank)
            .triangularView<Eigen::Upper>();
    }

    // Now handle the central point with negative weight (in scaled space)
    if (w0 < 0) {
        // This is a rank-1 downdate in scaled space
        double sqrt_abs_w0 = std::sqrt(std::abs(w0));
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, 1> v_scaled = sqrt_abs_w0 * error0_scaled;

        // Convert R to lower triangular
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM> S_scaled = R_scaled.transpose();

        // Perform rank-1 downdate in scaled space
        UKFMathUtils::cholupdate<UKF_ERROR_STATE_DIM>(S_scaled, v_scaled, -1.0);

        return S_scaled;  // Return SCALED Cholesky factor
    } else if (w0 > 0) {
        // This shouldn't happen in standard UKF, but handle it
        double sqrt_w0 = std::sqrt(w0);
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, 1> v_scaled = sqrt_w0 * error0_scaled;

        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM> S_scaled = R_scaled.transpose();
        UKFMathUtils::cholupdate<UKF_ERROR_STATE_DIM>(S_scaled, v_scaled, 1.0);

        return S_scaled;  // Return SCALED Cholesky factor
    }

    // If w0 == 0, just return the transposed R (in scaled space)
    return R_scaled.transpose();  // Return SCALED Cholesky factor
}