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

    // Transform covariance to scaled space
    auto P_scaled = scaler_.scaleCovariance(P0_physical);

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
    UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P_scaled, 1e-9);
    Eigen::LLT<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> llt(P_scaled);

    if (llt.info() != Eigen::Success) {
        std::cerr << "ERROR: Failed to compute Cholesky factor of scaled covariance!\n";
        S_ = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
    } else {
        S_ = llt.matrixL();
        std::cout << "\n✓ Cholesky factorization successful in scaled space\n";
    }

    std::cout << "==================================\n\n";
}

void ScaledUKF::generateSigmaPoints() {
    // CRITICAL: Generate sigma points with proper boundary transformations

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
            // Critical failure - reset to identity
            std::cerr << "CRITICAL: Failed to factorize scaled covariance!" << std::endl;
            S_ = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
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

    if (cond > last_condition_number_ * 10) {
        std::cout << "WARNING: Condition number degrading rapidly: "
                 << last_condition_number_ << " -> " << cond << "\n";
        divergence_count_++;
    } else {
        divergence_count_ = 0;
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
    // Simplified gradient update for testing
    // In production, this would involve full measurement model

    std::cout << "Gravity gradient update not yet implemented in ScaledUKF\n";
}

void ScaledUKF::updateAnomaly(double measured_anomaly_mgal, double noise_mgal) {
    // Simplified anomaly update for testing
    // In production, this would involve full measurement model

    std::cout << "Gravity anomaly update not yet implemented in ScaledUKF\n";
}