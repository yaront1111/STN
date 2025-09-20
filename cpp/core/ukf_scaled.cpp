#include "ukf_scaled.h"
#include "gravity_gradient_provider.h"
#include <iostream>
#include <iomanip>

ScaledUKF::ScaledUKF(const UKFConfig& config) : config_(config) {
    alpha_ = config.alpha;
    beta_ = config.beta;
    lambda_ = alpha_ * alpha_ * (ERROR_STATE_DIM + config.kappa) - ERROR_STATE_DIM;

    computeWeights();
    sigma_points_.resize(NUM_SIGMA_POINTS);

    std::cout << "=== Scaled UKF Initialized ===\n";
    std::cout << "Architecture: All operations in scaled space\n";
    std::cout << "Sigma points: " << NUM_SIGMA_POINTS << "\n";
    std::cout << "Lambda: " << lambda_ << "\n";
    std::cout << "==============================\n\n";
}

void ScaledUKF::computeWeights() {
    weights_mean_ = Eigen::VectorXd(NUM_SIGMA_POINTS);
    weights_cov_ = Eigen::VectorXd(NUM_SIGMA_POINTS);

    // Weight for center point
    weights_mean_(0) = lambda_ / (ERROR_STATE_DIM + lambda_);
    weights_cov_(0) = lambda_ / (ERROR_STATE_DIM + lambda_) + (1 - alpha_ * alpha_ + beta_);

    // Weights for other sigma points
    double weight = 0.5 / (ERROR_STATE_DIM + lambda_);
    for (int i = 1; i < NUM_SIGMA_POINTS; ++i) {
        weights_mean_(i) = weight;
        weights_cov_(i) = weight;
    }
}

void ScaledUKF::init(const State& x0_physical,
                    const Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& P0_physical) {
    // Store physical state
    nominal_state_ = x0_physical;

    // FIX: Ensure P0 has proper values for ALL states (not identity!)
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P0_fixed = P0_physical;

    // Check and fix any uninitialized (identity) diagonal values
    for (int i = 0; i < ERROR_STATE_DIM; ++i) {
        if (std::abs(P0_fixed(i,i) - 1.0) < 1e-6) {
            // This diagonal was left at identity - set proper uncertainty
            if (i < 3) {
                P0_fixed(i,i) = 100.0;    // Position: (10m)^2
            } else if (i < 6) {
                P0_fixed(i,i) = 1.0;      // Velocity: (1m/s)^2
            } else if (i < 9) {
                P0_fixed(i,i) = 0.01;     // Attitude: (0.1 rad)^2
            } else if (i < 12) {
                P0_fixed(i,i) = 1e-6;     // Accel bias: (0.001 m/s²)^2
            } else {
                P0_fixed(i,i) = 1e-10;    // Gyro bias: (1e-5 rad/s)^2
            }
        }
    }

    // Transform fixed covariance to scaled space
    auto P_scaled = scaler_.scaleCovariance(P0_fixed);

    // Check conditioning BEFORE Cholesky
    double cond_before = scaler_.computeConditionNumber(P0_physical);
    double cond_after = scaler_.computeConditionNumber(P_scaled);

    std::cout << "\n=== Initialization Diagnostics ===\n";
    std::cout << "Condition number (physical): " << std::scientific << cond_before << "\n";
    std::cout << "Condition number (scaled): " << std::scientific << cond_after << "\n";
    std::cout << "Improvement factor: " << cond_before / cond_after << "x\n";

    // Check diagonal elements are well-scaled
    std::cout << "\nScaled covariance diagonal:\n";
    for (int i = 0; i < ERROR_STATE_DIM; ++i) {
        std::cout << "  P_scaled(" << i << "," << i << ") = " << std::fixed
                 << std::setprecision(3) << P_scaled(i,i);
        if (i < 3) std::cout << " (position)";
        else if (i < 6) std::cout << " (velocity)";
        else if (i < 9) std::cout << " (attitude)";
        else if (i < 12) std::cout << " (accel bias)";
        else std::cout << " (gyro bias)";

        if (P_scaled(i,i) < 0.1 || P_scaled(i,i) > 10) {
            std::cout << " [WARNING: Not well-scaled!]";
        }
        std::cout << "\n";
    }

    // Compute Cholesky factor in scaled space
    // Do NOT call enforcePositiveDefinite - it resets to tiny values inappropriate for scaled space!
    // Instead, add small regularization
    P_scaled += 1e-9 * Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
    Eigen::LLT<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> llt(P_scaled);

    if (llt.info() != Eigen::Success) {
        std::cerr << "ERROR: Failed to compute Cholesky factor of scaled covariance!\n";
        S_ = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
        // Set reasonable uncertainties in SCALED space
        S_.block<3,3>(0,0) *= 2.0;  // Position: 20m in physical = 2 in scaled
        S_.block<3,3>(3,3) *= 1.0;  // Velocity: 1 m/s
        S_.block<3,3>(6,6) *= 1.0;  // Attitude: 0.1 rad
        S_.block<3,3>(9,9) *= 1.0;  // Accel bias
        S_.block<3,3>(12,12) *= 1.0; // Gyro bias
    } else {
        S_ = llt.matrixL();
        std::cout << "\n✓ Cholesky factorization successful in scaled space\n";
    }

    std::cout << "==================================\n\n";
}

void ScaledUKF::generateSigmaPoints() {
    // CRITICAL: Generate sigma points with proper boundary transformations

    // Check if S_ contains NaN or Inf
    if (S_.hasNaN() || !S_.allFinite()) {
        std::cerr << "ERROR: S_ contains NaN or Inf values!\n";
        std::cerr << "S_ diagonal: " << S_.diagonal().transpose() << "\n";
        // Reset to safe values
        S_ = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
    }

    // Central sigma point (no error)
    sigma_points_[0].state = nominal_state_;
    sigma_points_[0].error_scaled = Eigen::Matrix<double, ERROR_STATE_DIM, 1>::Zero();

    double sqrt_factor = std::sqrt(ERROR_STATE_DIM + lambda_);

    // Generate error vectors in SCALED space from S_
    for (int i = 0; i < ERROR_STATE_DIM; ++i) {
        // Get scaled error from Cholesky factor
        Eigen::Matrix<double, ERROR_STATE_DIM, 1> error_scaled = sqrt_factor * S_.col(i);

        // Transform to PHYSICAL space for state operations
        Eigen::Matrix<double, ERROR_STATE_DIM, 1> error_physical = scaler_.unscaleError(error_scaled);

        // Positive sigma point
        sigma_points_[i + 1].state = applyErrorPhysical(nominal_state_, error_physical);
        sigma_points_[i + 1].error_scaled = error_scaled;

        // Negative sigma point
        sigma_points_[i + 1 + ERROR_STATE_DIM].state = applyErrorPhysical(nominal_state_, -error_physical);
        sigma_points_[i + 1 + ERROR_STATE_DIM].error_scaled = -error_scaled;
    }
}

void ScaledUKF::predict(const ImuSample& imu, double dt) {
    // Debug: Check nominal state before prediction
    if (!nominal_state_.p_ECEF.allFinite() || !nominal_state_.v_ECEF.allFinite()) {
        std::cerr << "ERROR: NaN in nominal_state_ at start of predict!\n";
        std::cerr << "Nominal position: " << nominal_state_.p_ECEF.transpose() << "\n";
        std::cerr << "Nominal velocity: " << nominal_state_.v_ECEF.transpose() << "\n";
        return;
    }

    // Step 1: Generate sigma points with boundary transformation
    generateSigmaPoints();

    // Step 2: Propagate sigma points (in physical space)
    std::vector<State> propagated_states;
    propagated_states.reserve(NUM_SIGMA_POINTS);

    for (const auto& sp : sigma_points_) {
        propagated_states.push_back(propagateState(sp.state, imu, dt));
    }

    // Step 3: Compute mean state (physical)
    nominal_state_ = computeMeanState(propagated_states);

    // Step 4: Reconstruct covariance in SCALED space (critical!)
    reconstructScaledCovariance(propagated_states, nominal_state_);

    // Step 5: Add process noise in SCALED space
    addProcessNoiseScaled(dt);

    // Step 6: Monitor health
    monitorHealth();
}

void ScaledUKF::reconstructScaledCovariance(const std::vector<State>& propagated_states,
                                           const State& mean_state) {
    // CRITICAL: Proper weighted covariance reconstruction
    // All operations in scaled space for numerical stability

    // Step 1: Compute error vectors in PHYSICAL space
    Eigen::Matrix<double, ERROR_STATE_DIM, NUM_SIGMA_POINTS> errors_physical;

    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        errors_physical.col(i) = computeErrorPhysical(propagated_states[i], mean_state);
    }

    // Step 2: Transform ALL errors to SCALED space
    Eigen::Matrix<double, ERROR_STATE_DIM, NUM_SIGMA_POINTS> errors_scaled;

    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        errors_scaled.col(i) = scaler_.scaleError(errors_physical.col(i));
    }

    // Step 3: Compute weighted covariance in SCALED space
    // P_scaled = sum(w_i * e_i * e_i^T)
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_scaled =
        Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Zero();

    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        P_scaled += weights_cov_(i) * errors_scaled.col(i) * errors_scaled.col(i).transpose();
    }

    // Step 4: Ensure symmetry (handle numerical errors)
    P_scaled = 0.5 * (P_scaled + P_scaled.transpose());

    // Step 5: Add small regularization to ensure positive definiteness
    double regularization = 1e-9;
    P_scaled += regularization * Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();

    // Step 6: Cholesky decomposition to get S_
    Eigen::LLT<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> llt(P_scaled);

    if (llt.info() == Eigen::Success) {
        S_ = llt.matrixL();
    } else {
        // Fallback: use eigendecomposition for more robust factorization
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> es(P_scaled);

        if (es.info() == Eigen::Success) {
            Eigen::VectorXd eigenvalues = es.eigenvalues();
            Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> V = es.eigenvectors();

            // Clamp negative eigenvalues to small positive value
            for (int i = 0; i < ERROR_STATE_DIM; ++i) {
                if (eigenvalues(i) < regularization) {
                    eigenvalues(i) = regularization;
                }
            }

            // Reconstruct S_ = V * sqrt(D)
            Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> sqrt_D =
                eigenvalues.array().sqrt().matrix().asDiagonal();
            S_ = V * sqrt_D;
        } else {
            // Critical failure - reset to reasonable scaled values
            std::cerr << "CRITICAL: Failed to factorize scaled covariance!" << std::endl;
            S_ = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
            // Set reasonable uncertainties in SCALED space
            S_.block<3,3>(0,0) *= 2.0;  // Position: 20m in physical = 2 in scaled
            S_.block<3,3>(3,3) *= 1.0;  // Velocity: 1 m/s
            S_.block<3,3>(6,6) *= 1.0;  // Attitude: 0.1 rad
            S_.block<3,3>(9,9) *= 1.0;  // Accel bias
            S_.block<3,3>(12,12) *= 1.0; // Gyro bias
        }
    }

    // S_ is now the updated SCALED Cholesky factor!
}

void ScaledUKF::addProcessNoiseScaled(double dt) {
    // Add process noise in SCALED space

    // Define process noise in PHYSICAL units
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> q_std_physical =
        Eigen::Matrix<double, ERROR_STATE_DIM, 1>::Zero();

    double dt_sqrt = std::sqrt(dt);

    // Position: continuous white noise acceleration model
    for (int i = 0; i < 3; ++i) {
        q_std_physical(i) = config_.process_noise.position * dt;
    }

    // Velocity: continuous white noise jerk model
    for (int i = 3; i < 6; ++i) {
        q_std_physical(i) = config_.process_noise.velocity * dt_sqrt;
    }

    // Attitude: angular random walk
    for (int i = 6; i < 9; ++i) {
        q_std_physical(i) = config_.process_noise.attitude * dt_sqrt;
    }

    // Accelerometer bias: random walk
    for (int i = 9; i < 12; ++i) {
        q_std_physical(i) = config_.process_noise.accel_bias * dt_sqrt;
    }

    // Gyroscope bias: random walk
    for (int i = 12; i < 15; ++i) {
        q_std_physical(i) = config_.process_noise.gyro_bias * dt_sqrt;
    }

    // Transform to SCALED space
    auto q_std_scaled = scaler_.scaleProcessNoise(q_std_physical);

    // Sequential rank-1 updates in SCALED space
    for (int i = 0; i < ERROR_STATE_DIM; ++i) {
        if (q_std_scaled(i) > 1e-12) {
            Eigen::Matrix<double, ERROR_STATE_DIM, 1> ei =
                Eigen::Matrix<double, ERROR_STATE_DIM, 1>::Zero();
            ei(i) = q_std_scaled(i);

            // Update S_ in SCALED space (well-conditioned!)
            UKFMathUtils::cholupdate<ERROR_STATE_DIM>(S_, ei, 1.0);
        }
    }
}

State ScaledUKF::propagateState(const State& state, const ImuSample& imu, double dt) const {
    // Check for NaN in input state
    if (!state.p_ECEF.allFinite() || !state.v_ECEF.allFinite()) {
        std::cerr << "ERROR: NaN in input state to propagateState!\n";
        std::cerr << "Position: " << state.p_ECEF.transpose() << "\n";
        std::cerr << "Velocity: " << state.v_ECEF.transpose() << "\n";
        // Return unchanged state to prevent further corruption
        return state;
    }

    // Standard state propagation (physical units)
    // This is the same as any UKF implementation

    State propagated = state;

    // Earth rotation
    const Eigen::Vector3d omega_ie(0.0, 0.0, 7.292115e-5);

    // Attitude update
    Eigen::Vector3d corrected_gyro = imu.gyro_rps - state.b_g;
    Eigen::Vector3d omega_dt = corrected_gyro * dt;
    if (omega_dt.norm() > 1e-10) {
        Eigen::Quaterniond delta_q = UKFMathUtils::rotationVectorToQuaternion(omega_dt);
        propagated.q_ECEF_B = state.q_ECEF_B * delta_q;
        propagated.q_ECEF_B.normalize();
    }

    // Specific force
    Eigen::Vector3d acc_body = imu.acc_mps2 - state.b_a;
    Eigen::Vector3d acc_ecef = state.q_ECEF_B * acc_body;

    // Gravity (simplified)
    Eigen::Vector3d gravity_vec;
    if (gravity_provider_) {
        Eigen::Vector3d lla = UKFMathUtils::ecefToLla(state.p_ECEF);
        double g = UKFMathUtils::getNormalGravity(lla(0));
        gravity_vec = -g * (state.p_ECEF / state.p_ECEF.norm());
    } else {
        gravity_vec = -9.81 * (state.p_ECEF / state.p_ECEF.norm());
    }

    // Coriolis and centripetal
    Eigen::Vector3d coriolis = -2.0 * omega_ie.cross(state.v_ECEF);
    Eigen::Vector3d centripetal = -omega_ie.cross(omega_ie.cross(state.p_ECEF));

    // Total acceleration
    Eigen::Vector3d total_accel = acc_ecef + gravity_vec + coriolis + centripetal;

    // Integration
    propagated.v_ECEF = state.v_ECEF + total_accel * dt;
    propagated.p_ECEF = state.p_ECEF + state.v_ECEF * dt + 0.5 * total_accel * dt * dt;

    // Biases (constant)
    propagated.b_a = state.b_a;
    propagated.b_g = state.b_g;

    propagated.t = state.t + dt;

    return propagated;
}

State ScaledUKF::computeMeanState(const std::vector<State>& states) const {
    State mean;

    // Position and velocity: simple average
    mean.p_ECEF = Eigen::Vector3d::Zero();
    mean.v_ECEF = Eigen::Vector3d::Zero();
    mean.b_a = Eigen::Vector3d::Zero();
    mean.b_g = Eigen::Vector3d::Zero();

    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        mean.p_ECEF += weights_mean_(i) * states[i].p_ECEF;
        mean.v_ECEF += weights_mean_(i) * states[i].v_ECEF;
        mean.b_a += weights_mean_(i) * states[i].b_a;
        mean.b_g += weights_mean_(i) * states[i].b_g;
    }

    // Quaternion: special averaging
    std::vector<Eigen::Quaterniond> quaternions;
    for (const auto& state : states) {
        quaternions.push_back(state.q_ECEF_B);
    }
    mean.q_ECEF_B = UKFMathUtils::averageQuaternions(quaternions, weights_mean_);

    mean.t = states[0].t;

    return mean;
}

Eigen::Matrix<double, ScaledUKF::ERROR_STATE_DIM, 1>
ScaledUKF::computeErrorPhysical(const State& state1, const State& state2) const {
    // Compute error in PHYSICAL units
    return UKFMathUtils::computeErrorVector(state1, state2);
}

State ScaledUKF::applyErrorPhysical(const State& nominal,
                                   const Eigen::Matrix<double, ERROR_STATE_DIM, 1>& error_physical) const {
    // Apply error in PHYSICAL units
    return UKFMathUtils::applyErrorToState(nominal, error_physical);
}

void ScaledUKF::monitorHealth() const {
    // Monitor numerical health of scaled system

    // Check condition number of scaled covariance
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_scaled = S_ * S_.transpose();
    double cond = scaler_.computeConditionNumber(P_scaled);

    // CRITICAL: If condition number is too high, reset covariance
    if (cond > 1e8 || std::isinf(cond) || std::isnan(cond)) {
        std::cout << "CRITICAL: Resetting covariance due to poor conditioning: " << cond << "\n";

        // Reset to well-conditioned diagonal matrix
        const_cast<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>&>(S_) =
            Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();

        // Reset divergence count
        const_cast<int&>(divergence_count_) = 0;
        const_cast<double&>(last_condition_number_) = 1.0;
        return;
    }

    if (cond > last_condition_number_ * 10 && cond > 1e4) {
        std::cout << "WARNING: Condition number degrading rapidly: "
                 << std::scientific << last_condition_number_ << " -> " << cond << "\n";
        divergence_count_++;
    } else if (cond < last_condition_number_ * 0.5) {
        // Improving - reduce divergence count
        if (divergence_count_ > 0) divergence_count_--;
    }

    last_condition_number_ = cond;

    // Periodic health report
    static int predict_count = 0;
    if (++predict_count % 100 == 0) {
        std::cout << "Filter Health [" << predict_count << " predictions]:\n";
        std::cout << "  Scaled covariance condition number: " << std::scientific << cond << "\n";
        std::cout << "  Divergence count: " << divergence_count_ << "\n";

        // Check diagonal elements
        bool all_good = true;
        for (int i = 0; i < ERROR_STATE_DIM; ++i) {
            double diag = P_scaled(i,i);
            if (diag < 0.01 || diag > 100) {
                all_good = false;
                std::cout << "  WARNING: P_scaled(" << i << "," << i << ") = " << diag << "\n";
            }
        }

        if (all_good) {
            std::cout << "  ✓ All diagonal elements in healthy range [0.01, 100]\n";
        }
    }
}

bool ScaledUKF::isHealthy() const {
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_scaled = S_ * S_.transpose();
    double cond = scaler_.computeConditionNumber(P_scaled);

    return (cond < 1e6) && (divergence_count_ < 10);
}

Eigen::Matrix<double, ScaledUKF::ERROR_STATE_DIM, ScaledUKF::ERROR_STATE_DIM>
ScaledUKF::getCovariance() const {
    // Return covariance in PHYSICAL units
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_scaled = S_ * S_.transpose();
    return scaler_.unscaleCovariance(P_scaled);
}

void ScaledUKF::updateGradient(const Eigen::Matrix3d& measured_gradient, const Eigen::Matrix3d& R) {
    // Gravity gradient measurement update
    // This provides continuous weak observability

    if (!gravity_provider_) {
        std::cerr << "WARNING: No gravity provider set for gradient update\n";
        return;
    }

    // Generate sigma points for measurement prediction
    generateSigmaPoints();

    // Predict measurements for each sigma point
    std::vector<Eigen::Matrix3d> predicted_gradients;
    for (const auto& sp : sigma_points_) {
        auto tensor = gravity_provider_->getGradient(sp.state.p_ECEF);
        predicted_gradients.push_back(tensor.T);
    }

    // Compute mean predicted gradient
    Eigen::Matrix3d mean_gradient = Eigen::Matrix3d::Zero();
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        mean_gradient += weights_mean_(i) * predicted_gradients[i];
    }

    // Innovation (measurement residual)
    Eigen::Matrix3d innovation = measured_gradient - mean_gradient;

    // Flatten to 9D vector for update (gradient tensor has 9 components)
    Eigen::Matrix<double, 9, 1> y = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 1> y_pred = Eigen::Matrix<double, 9, 1>::Zero();

    int idx = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            y(idx) = measured_gradient(i, j);
            y_pred(idx) = mean_gradient(i, j);
            idx++;
        }
    }

    // Flatten R to 9x9
    Eigen::Matrix<double, 9, 9> R_flat = Eigen::Matrix<double, 9, 9>::Zero();
    for (int i = 0; i < 9; ++i) {
        R_flat(i, i) = R(i/3, i%3);
    }

    // Compute cross-covariance Pxy (in physical space)
    Eigen::Matrix<double, ERROR_STATE_DIM, 9> Pxy = Eigen::Matrix<double, ERROR_STATE_DIM, 9>::Zero();
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        // Flatten predicted gradient for this sigma point
        Eigen::Matrix<double, 9, 1> y_i;
        idx = 0;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                y_i(idx++) = predicted_gradients[i](row, col);
            }
        }

        Eigen::Matrix<double, 9, 1> dy = y_i - y_pred;
        Eigen::Matrix<double, ERROR_STATE_DIM, 1> dx = computeErrorPhysical(sigma_points_[i].state, nominal_state_);

        Pxy += weights_cov_(i) * dx * dy.transpose();
    }

    // Compute innovation covariance Pyy
    Eigen::Matrix<double, 9, 9> Pyy = R_flat;
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        Eigen::Matrix<double, 9, 1> y_i;
        idx = 0;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                y_i(idx++) = predicted_gradients[i](row, col);
            }
        }
        Eigen::Matrix<double, 9, 1> dy = y_i - y_pred;
        Pyy += weights_cov_(i) * dy * dy.transpose();
    }

    // Kalman gain
    Eigen::Matrix<double, ERROR_STATE_DIM, 9> K = Pxy * Pyy.inverse();

    // State update
    Eigen::Matrix<double, 9, 1> innovation_flat = y - y_pred;
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> dx_physical = K * innovation_flat;
    nominal_state_ = applyErrorPhysical(nominal_state_, dx_physical);

    // Covariance update in scaled space (Joseph form for stability)
    // Transform to scaled space
    Eigen::Matrix<double, ERROR_STATE_DIM, 9> K_scaled = scaler_.getInverseScalingMatrix() * K;

    // Update scaled covariance using Joseph form
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> I_KH =
        Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();

    // Perform Joseph form update for numerical stability
    // P+ = (I - K*H) * P * (I - K*H)' + K*R*K'

    // Approximate H matrix (measurement Jacobian)
    Eigen::Matrix<double, 9, ERROR_STATE_DIM> H_approx = Eigen::Matrix<double, 9, ERROR_STATE_DIM>::Zero();
    // Gradient is most sensitive to position
    H_approx.block<9, 3>(0, 0) = Eigen::Matrix<double, 9, 3>::Identity() * 0.1;  // Position sensitivity

    // Transform K to scaled space (reuse existing variable)
    K_scaled = scaler_.getInverseScalingMatrix() * K;

    // Joseph form in scaled space (reuse existing variable)
    I_KH = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity() - K_scaled * H_approx;

    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_scaled = S_ * S_.transpose();
    P_scaled = I_KH * P_scaled * I_KH.transpose() + K_scaled * R_flat * K_scaled.transpose();

    // Recompute Cholesky factor
    UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P_scaled, 1e-9);
    Eigen::LLT<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> llt(P_scaled);
    if (llt.info() == Eigen::Success) {
        S_ = llt.matrixL();
    }
}

void ScaledUKF::updateAnomaly(double measured_anomaly_mgal, double noise_mgal) {
    // Gravity anomaly measurement update
    // Single scalar measurement but provides absolute reference

    if (!gravity_provider_) {
        std::cerr << "WARNING: No gravity provider set for anomaly update\n";
        return;
    }

    // Generate sigma points for measurement prediction
    generateSigmaPoints();

    // Predict anomaly for each sigma point
    std::vector<double> predicted_anomalies;
    for (const auto& sp : sigma_points_) {
        double anomaly = gravity_provider_->getAnomaly(sp.state.p_ECEF);
        predicted_anomalies.push_back(anomaly);
    }

    // Compute weighted mean
    double mean_anomaly = 0;
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        mean_anomaly += weights_mean_(i) * predicted_anomalies[i];
    }

    // Innovation
    double innovation = measured_anomaly_mgal - mean_anomaly;

    // Compute cross-covariance Pxy (ERROR_STATE_DIM x 1)
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> Pxy = Eigen::Matrix<double, ERROR_STATE_DIM, 1>::Zero();
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        double dy = predicted_anomalies[i] - mean_anomaly;
        Eigen::Matrix<double, ERROR_STATE_DIM, 1> dx = computeErrorPhysical(sigma_points_[i].state, nominal_state_);
        Pxy += weights_cov_(i) * dx * dy;
    }

    // Compute innovation covariance (scalar)
    double Pyy = noise_mgal * noise_mgal;
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        double dy = predicted_anomalies[i] - mean_anomaly;
        Pyy += weights_cov_(i) * dy * dy;
    }

    // Kalman gain
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> K = Pxy / Pyy;

    // State update
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> dx_physical = K * innovation;
    nominal_state_ = applyErrorPhysical(nominal_state_, dx_physical);

    // Covariance update in scaled space
    // Joseph form update for anomaly measurement
    Eigen::Matrix<double, 1, ERROR_STATE_DIM> H_scalar = Eigen::Matrix<double, 1, ERROR_STATE_DIM>::Zero();
    H_scalar(0, 0) = 0.05;  // Position sensitivity in x
    H_scalar(0, 1) = 0.05;  // Position sensitivity in y
    H_scalar(0, 2) = 0.01;  // Position sensitivity in z

    // Transform to scaled space
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> K_scaled = scaler_.getInverseScalingMatrix() * K;

    // Joseph form update
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> I_KH =
        Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity() - K_scaled * H_scalar;

    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_scaled = S_ * S_.transpose();
    double R_scalar = noise_mgal * noise_mgal;
    P_scaled = I_KH * P_scaled * I_KH.transpose() + K_scaled * K_scaled.transpose() * R_scalar;

    // Recompute Cholesky factor
    UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P_scaled, 1e-9);
    Eigen::LLT<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> llt(P_scaled);
    if (llt.info() == Eigen::Success) {
        S_ = llt.matrixL();
    }
}

void ScaledUKF::updateMapMatch(const Eigen::Vector3d& matched_position_ECEF, double uncertainty_m) {
    // Map match position fix - strongest update
    // Directly corrects position error

    // Position innovation
    Eigen::Vector3d innovation = matched_position_ECEF - nominal_state_.p_ECEF;

    // Compute Kalman gain based on uncertainty
    double map_variance = uncertainty_m * uncertainty_m;

    // Get current position variance from covariance
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_current = S_ * S_.transpose();
    double pos_variance = (P_current(0,0) + P_current(1,1) + P_current(2,2)) / 3.0;

    // Optimal Kalman gain
    double K_pos = pos_variance / (pos_variance + map_variance);
    K_pos = std::min(0.95, K_pos);  // Cap at 95% to maintain some uncertainty

    // Apply optimal correction
    nominal_state_.p_ECEF += K_pos * innovation;

    // Velocity correction based on time since last match (assume 10s)
    double dt_match = 10.0;
    double K_vel = 0.1 * K_pos;  // Smaller velocity correction
    nominal_state_.v_ECEF += (K_vel * innovation) / dt_match;

    // Reduce position uncertainty significantly
    // Map matching provides strong position observability
    // Extract current covariance
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_scaled = S_ * S_.transpose();

    // Reduce position uncertainty based on Kalman gain
    // Joseph form: P = (I - K*H)*P*(I - K*H)' + K*R*K'
    // For position update, H = I for position states
    double reduction_factor = (1.0 - K_pos) * (1.0 - K_pos) + K_pos * K_pos * map_variance / pos_variance;
    reduction_factor = std::max(0.01, reduction_factor);  // At least 99% reduction

    P_scaled.block<3,3>(0,0) = P_scaled.block<3,3>(0,0) * reduction_factor;
    P_scaled.block<3,3>(3,3) = P_scaled.block<3,3>(3,3) * (1.0 - 0.5 * K_vel);  // Velocity uncertainty reduction

    // Recompute Cholesky factor
    Eigen::LLT<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> llt(P_scaled);
    if (llt.info() == Eigen::Success) {
        S_ = llt.matrixL();
    }

    std::cout << "Map match applied: correction = " << std::fixed << std::setprecision(1)
              << innovation.norm() << " m (K=" << std::setprecision(2) << K_pos << ")\n";
}

// Magnetometer update implementation
void ScaledUKF::updateMagnetometer(const Eigen::Vector3d& mag_body,
                                   const Eigen::Vector3d& mag_ref_ECEF,
                                   const Eigen::Matrix3d& R_mag) {
    if (config_.verbose) {
        std::cout << "[ScaledUKF] Magnetometer update\n";
    }

    // Expected measurement in body frame
    Eigen::Vector3d expected_mag_body = nominal_state_.q_ECEF_B.inverse() * mag_ref_ECEF;

    // Innovation
    Eigen::Vector3d innovation = mag_body - expected_mag_body;

    // Measurement Jacobian (3x15)
    // Magnetometer is sensitive to attitude errors
    Eigen::Matrix<double, 3, ERROR_STATE_DIM> H = Eigen::Matrix<double, 3, ERROR_STATE_DIM>::Zero();

    // Attitude sensitivity: d(mag_body)/d(theta)
    // Using small angle approximation: δR ≈ I - [θ×]
    Eigen::Matrix3d mag_ref_skew = UKFMathUtils::skewSymmetric(expected_mag_body);
    H.block<3,3>(0, 6) = mag_ref_skew;  // Attitude error affects magnetometer

    // Compute Kalman gain in physical space
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_physical = getCovariance();
    Eigen::Matrix3d S_innovation = H * P_physical * H.transpose() + R_mag;
    Eigen::Matrix<double, ERROR_STATE_DIM, 3> K = P_physical * H.transpose() * S_innovation.inverse();

    // State correction
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> dx = K * innovation;

    // Apply correction to nominal state
    nominal_state_ = applyErrorPhysical(nominal_state_, dx);

    // Update covariance using Joseph form for numerical stability
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> I_KH =
        Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity() - K * H;
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_updated =
        I_KH * P_physical * I_KH.transpose() + K * R_mag * K.transpose();

    // Convert back to scaled Cholesky factor
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> scales = scaler_.getScales();
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> D_inv = scales.asDiagonal().inverse();
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_scaled = D_inv * P_updated * D_inv;

    // Cholesky decomposition
    Eigen::LLT<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> llt(P_scaled);
    if (llt.info() == Eigen::Success) {
        S_ = llt.matrixL();
    } else {
        std::cerr << "[ScaledUKF] Warning: Cholesky failed in magnetometer update\n";
    }
}

// Barometric altitude update
void ScaledUKF::updateBarometer(double altitude_msl, double noise_variance) {
    if (config_.verbose) {
        std::cout << "[ScaledUKF] Barometer update: " << altitude_msl << " m\n";
    }

    // Convert current position to LLA to get altitude
    Eigen::Vector3d lla = UKFMathUtils::ecefToLla(nominal_state_.p_ECEF);
    double expected_alt = lla(2);

    // Innovation (scalar)
    double innovation = altitude_msl - expected_alt;

    // Measurement Jacobian (1x15)
    // Barometer only measures altitude (vertical position)
    Eigen::Matrix<double, 1, ERROR_STATE_DIM> H = Eigen::Matrix<double, 1, ERROR_STATE_DIM>::Zero();

    // Compute altitude sensitivity to ECEF position
    // Simplified: assume mostly vertical sensitivity
    Eigen::Vector3d up_ECEF = nominal_state_.p_ECEF.normalized();
    H.block<1,3>(0, 0) = up_ECEF.transpose();

    // Kalman gain computation
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_physical = getCovariance();
    double S_innovation = (H * P_physical * H.transpose())(0,0) + noise_variance;
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> K = P_physical * H.transpose() / S_innovation;

    // State correction
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> dx = K * innovation;

    // Apply correction
    nominal_state_ = applyErrorPhysical(nominal_state_, dx);

    // Update covariance
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> I_KH =
        Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity() - K * H;
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_updated =
        I_KH * P_physical * I_KH.transpose() + K * noise_variance * K.transpose();

    // Convert back to scaled Cholesky factor
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> scales = scaler_.getScales();
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> D_inv = scales.asDiagonal().inverse();
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_scaled = D_inv * P_updated * D_inv;

    // Cholesky decomposition
    Eigen::LLT<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> llt(P_scaled);
    if (llt.info() == Eigen::Success) {
        S_ = llt.matrixL();
    }
}

// Zero Velocity Update (ZUPT)
void ScaledUKF::updateZUPT(const Eigen::Matrix3d& R_vel) {
    if (config_.verbose) {
        std::cout << "[ScaledUKF] ZUPT: constraining velocity to zero\n";
    }

    // Innovation: measured velocity (0) - estimated velocity
    Eigen::Vector3d innovation = -nominal_state_.v_ECEF;

    // Measurement Jacobian (3x15)
    // ZUPT directly observes velocity
    Eigen::Matrix<double, 3, ERROR_STATE_DIM> H = Eigen::Matrix<double, 3, ERROR_STATE_DIM>::Zero();
    H.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();  // Velocity observation

    // Kalman gain
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_physical = getCovariance();
    Eigen::Matrix3d S_innovation = H * P_physical * H.transpose() + R_vel;
    Eigen::Matrix<double, ERROR_STATE_DIM, 3> K = P_physical * H.transpose() * S_innovation.inverse();

    // State correction
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> dx = K * innovation;

    // Apply correction
    nominal_state_ = applyErrorPhysical(nominal_state_, dx);

    // Update covariance using Joseph form
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> I_KH =
        Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity() - K * H;
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_updated =
        I_KH * P_physical * I_KH.transpose() + K * R_vel * K.transpose();

    // Convert back to scaled Cholesky factor
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> scales = scaler_.getScales();
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> D_inv = scales.asDiagonal().inverse();
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_scaled = D_inv * P_updated * D_inv;

    // Cholesky decomposition
    Eigen::LLT<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> llt(P_scaled);
    if (llt.info() == Eigen::Success) {
        S_ = llt.matrixL();
    } else {
        std::cerr << "[ScaledUKF] Warning: Cholesky failed in ZUPT\n";
    }
}