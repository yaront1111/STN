#include "ukf_stable.h"
#include <cmath>
#include <iostream>

UKF_Stable::UKF_Stable(const Config& cfg) : cfg_(cfg) {
    computeWeights();
    sigma_points_.resize(NUM_SIGMA_POINTS);
}

void UKF_Stable::computeWeights() {
    lambda_ = cfg_.alpha * cfg_.alpha * (ERROR_STATE_DIM + cfg_.kappa) - ERROR_STATE_DIM;
    
    weights_mean_ = Eigen::VectorXd(NUM_SIGMA_POINTS);
    weights_cov_ = Eigen::VectorXd(NUM_SIGMA_POINTS);
    
    // Weight for center point
    weights_mean_(0) = lambda_ / (ERROR_STATE_DIM + lambda_);
    weights_cov_(0) = lambda_ / (ERROR_STATE_DIM + lambda_) + (1 - cfg_.alpha * cfg_.alpha + cfg_.beta);
    
    // Weights for other sigma points
    double weight = 0.5 / (ERROR_STATE_DIM + lambda_);
    for (int i = 1; i < NUM_SIGMA_POINTS; ++i) {
        weights_mean_(i) = weight;
        weights_cov_(i) = weight;
    }
}

void UKF_Stable::init(const State& x0, const Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& P0) {
    nominal_state_ = x0;
    P_ = P0;
    enforcePositiveDefinite(P_);
    
    // Initialize sigma points
    generateSigmaPoints();
}

void UKF_Stable::generateSigmaPoints() {
    // Check input validity
    if (!P_.allFinite()) {
        std::cerr << "ERROR: P has NaN/Inf before generateSigmaPoints!\n";
        std::cerr << "P diagonal: " << P_.diagonal().transpose() << "\n";
        P_ = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity() * 0.1;
    }
    
    // Step 1: Ensure P is symmetric and positive definite
    enforcePositiveDefinite(P_);
    
    // Step 2: Compute matrix square root using Cholesky
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> sqrt_P;
    Eigen::LLT<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> llt((ERROR_STATE_DIM + lambda_) * P_);
    
    if (llt.info() == Eigen::Success) {
        sqrt_P = llt.matrixL();
    } else {
        // Fallback: use eigenvalue decomposition
        std::cerr << "Warning: Cholesky failed, using eigenvalue decomposition\n";
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> es((ERROR_STATE_DIM + lambda_) * P_);
        Eigen::Matrix<double, ERROR_STATE_DIM, 1> eigenvalues = es.eigenvalues().array().max(0).sqrt().matrix();
        sqrt_P = es.eigenvectors() * eigenvalues.asDiagonal();
    }
    
    // Step 3: Generate sigma points
    // Center point
    sigma_points_[0].state = nominal_state_;
    sigma_points_[0].error = Eigen::Matrix<double, ERROR_STATE_DIM, 1>::Zero();
    
    // Other sigma points
    for (int i = 0; i < ERROR_STATE_DIM; ++i) {
        Eigen::Matrix<double, ERROR_STATE_DIM, 1> delta = sqrt_P.col(i);
        
        // Positive direction
        sigma_points_[i + 1].error = delta;
        sigma_points_[i + 1].state = applyError(nominal_state_, delta);
        
        // Negative direction  
        sigma_points_[i + 1 + ERROR_STATE_DIM].error = -delta;
        sigma_points_[i + 1 + ERROR_STATE_DIM].state = applyError(nominal_state_, -delta);
    }
}

State UKF_Stable::applyError(const State& nominal, const Eigen::Matrix<double, ERROR_STATE_DIM, 1>& error) {
    State result = nominal;
    
    // Position and velocity: simple addition
    result.p_ECEF = nominal.p_ECEF + error.segment<3>(POS_IDX);
    result.v_ECEF = nominal.v_ECEF + error.segment<3>(VEL_IDX);
    
    // Attitude: compose rotations
    Eigen::Vector3d rot_vec = error.segment<3>(ATT_IDX);
    Eigen::Quaterniond delta_q = rotationVectorToQuaternion(rot_vec);
    result.q_ECEF_B = nominal.q_ECEF_B * delta_q;
    result.q_ECEF_B.normalize();
    
    // Biases: simple addition
    result.b_a = nominal.b_a + error.segment<3>(BA_IDX);
    result.b_g = nominal.b_g + error.segment<3>(BG_IDX);
    
    // Keep other states
    result.dt = nominal.dt;
    result.df = nominal.df;
    result.ddf = nominal.ddf;
    result.t = nominal.t;
    
    return result;
}

Eigen::Matrix<double, UKF_Stable::ERROR_STATE_DIM, 1> 
UKF_Stable::computeError(const State& x1, const State& x2) {
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> error;
    
    // Position and velocity errors
    error.segment<3>(POS_IDX) = x1.p_ECEF - x2.p_ECEF;
    error.segment<3>(VEL_IDX) = x1.v_ECEF - x2.v_ECEF;
    
    // Attitude error (rotation from x2 to x1)
    Eigen::Quaterniond delta_q = x2.q_ECEF_B.inverse() * x1.q_ECEF_B;
    error.segment<3>(ATT_IDX) = quaternionToRotationVector(delta_q);
    
    // Bias errors
    error.segment<3>(BA_IDX) = x1.b_a - x2.b_a;
    error.segment<3>(BG_IDX) = x1.b_g - x2.b_g;
    
    return error;
}

Eigen::Quaterniond UKF_Stable::rotationVectorToQuaternion(const Eigen::Vector3d& rot_vec) {
    double angle = rot_vec.norm();
    
    if (angle < 1e-10) {
        // Small angle approximation
        return Eigen::Quaterniond(1.0, rot_vec.x() / 2.0, rot_vec.y() / 2.0, rot_vec.z() / 2.0);
    } else {
        // Axis-angle to quaternion
        Eigen::Vector3d axis = rot_vec / angle;
        double half_angle = angle / 2.0;
        double sin_half = std::sin(half_angle);
        return Eigen::Quaterniond(std::cos(half_angle), 
                                   sin_half * axis.x(),
                                   sin_half * axis.y(),
                                   sin_half * axis.z());
    }
}

Eigen::Vector3d UKF_Stable::quaternionToRotationVector(const Eigen::Quaterniond& q) {
    // Ensure quaternion is normalized
    Eigen::Quaterniond q_norm = q.normalized();
    
    // Extract axis-angle
    double angle = 2.0 * std::acos(std::min(1.0, std::max(-1.0, q_norm.w())));
    
    if (angle < 1e-10) {
        // Small angle: use approximation
        return 2.0 * q_norm.vec();
    } else {
        // General case
        Eigen::Vector3d axis = q_norm.vec() / std::sin(angle / 2.0);
        return angle * axis;
    }
}

State UKF_Stable::propagateState(const State& state, const ImuSample& imu, double dt) {
    State next = state;
    
    // Remove biases from IMU measurements
    Eigen::Vector3d acc_corrected = imu.acc_mps2 - state.b_a;
    Eigen::Vector3d gyro_corrected = imu.gyro_rps - state.b_g;
    
    // Propagate attitude (integrate angular velocity)
    Eigen::Vector3d angle_increment = gyro_corrected * dt;
    Eigen::Quaterniond delta_q = rotationVectorToQuaternion(angle_increment);
    next.q_ECEF_B = state.q_ECEF_B * delta_q;
    next.q_ECEF_B.normalize();
    
    // Transform acceleration to ECEF frame
    Eigen::Vector3d acc_ecef = state.q_ECEF_B * acc_corrected;
    
    // Add gravity (simplified WGS84)
    Eigen::Vector3d gravity_ecef(0, 0, -9.80665);  // Simplified
    acc_ecef += gravity_ecef;
    
    // Add Coriolis and centrifugal forces
    const double omega_earth = 7.292115e-5;  // rad/s
    Eigen::Vector3d omega_vec(0, 0, omega_earth);
    acc_ecef -= 2.0 * omega_vec.cross(state.v_ECEF);  // Coriolis
    acc_ecef -= omega_vec.cross(omega_vec.cross(state.p_ECEF));  // Centrifugal
    
    // Propagate velocity and position
    next.v_ECEF = state.v_ECEF + acc_ecef * dt;
    next.p_ECEF = state.p_ECEF + state.v_ECEF * dt + 0.5 * acc_ecef * dt * dt;
    
    // Biases remain constant (random walk model)
    next.b_a = state.b_a;
    next.b_g = state.b_g;
    
    // Propagate clock states
    next.dt = state.dt + state.df * dt;
    next.df = state.df + state.ddf * dt;
    next.ddf = state.ddf;
    
    // Update time
    next.t = state.t + dt;
    
    return next;
}

void UKF_Stable::predict(const ImuSample& imu, double dt) {
    // Step 1: Generate sigma points
    generateSigmaPoints();
    
    // Step 2: Propagate each sigma point
    std::vector<State> propagated_states(NUM_SIGMA_POINTS);
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        propagated_states[i] = propagateState(sigma_points_[i].state, imu, dt);
    }
    
    // Step 3: Compute predicted mean state
    // For position, velocity, biases: weighted average
    Eigen::Vector3d mean_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_bg = Eigen::Vector3d::Zero();
    
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        mean_pos += weights_mean_(i) * propagated_states[i].p_ECEF;
        mean_vel += weights_mean_(i) * propagated_states[i].v_ECEF;
        mean_ba += weights_mean_(i) * propagated_states[i].b_a;
        mean_bg += weights_mean_(i) * propagated_states[i].b_g;
    }
    
    // For attitude: quaternion averaging (simplified - could use more sophisticated method)
    Eigen::Quaterniond mean_q = propagated_states[0].q_ECEF_B;
    for (int iter = 0; iter < 3; ++iter) {
        Eigen::Vector3d error_sum = Eigen::Vector3d::Zero();
        for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
            Eigen::Quaterniond delta = mean_q.inverse() * propagated_states[i].q_ECEF_B;
            error_sum += weights_mean_(i) * quaternionToRotationVector(delta);
        }
        mean_q = mean_q * rotationVectorToQuaternion(error_sum);
        mean_q.normalize();
    }
    
    // Update nominal state
    nominal_state_.p_ECEF = mean_pos;
    nominal_state_.v_ECEF = mean_vel;
    nominal_state_.q_ECEF_B = mean_q;
    nominal_state_.b_a = mean_ba;
    nominal_state_.b_g = mean_bg;
    nominal_state_.t = propagated_states[0].t;
    
    // Step 4: Compute predicted covariance
    P_ = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Zero();
    
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        Eigen::Matrix<double, ERROR_STATE_DIM, 1> error = computeError(propagated_states[i], nominal_state_);
        P_ += weights_cov_(i) * error * error.transpose();
    }
    
    // Step 5: Add process noise
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> Q = 
        Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Zero();
    
    Q.block<3,3>(POS_IDX, POS_IDX) = Eigen::Matrix3d::Identity() * cfg_.sigma_pos * cfg_.sigma_pos * dt * dt;
    Q.block<3,3>(VEL_IDX, VEL_IDX) = Eigen::Matrix3d::Identity() * cfg_.sigma_vel * cfg_.sigma_vel * dt;
    Q.block<3,3>(ATT_IDX, ATT_IDX) = Eigen::Matrix3d::Identity() * cfg_.sigma_att * cfg_.sigma_att * dt;
    Q.block<3,3>(BA_IDX, BA_IDX) = Eigen::Matrix3d::Identity() * cfg_.sigma_ba * cfg_.sigma_ba * dt;
    Q.block<3,3>(BG_IDX, BG_IDX) = Eigen::Matrix3d::Identity() * cfg_.sigma_bg * cfg_.sigma_bg * dt;
    
    P_ += Q;
    
    // Step 6: Enforce positive definiteness
    enforcePositiveDefinite(P_);
    
    // Update sigma points for next iteration
    sigma_points_[0].state = nominal_state_;
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        sigma_points_[i].state = propagated_states[i];
    }
}

void UKF_Stable::enforcePositiveDefinite(Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& P) {
    // Step 1: Enforce symmetry
    P = 0.5 * (P + P.transpose());
    
    // Step 2: Ensure positive definiteness using eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>> es(P);
    
    // If any eigenvalue is negative, fix it
    if (es.eigenvalues().minCoeff() < 1e-10) {
        Eigen::Matrix<double, ERROR_STATE_DIM, 1> eigenvalues = es.eigenvalues().array().max(1e-10).matrix();
        P = es.eigenvectors() * eigenvalues.asDiagonal() * es.eigenvectors().transpose();
        
        // Enforce symmetry again after reconstruction
        P = 0.5 * (P + P.transpose());
    }
}

void UKF_Stable::updateGradient(const Eigen::Matrix3d& measured, const Eigen::Matrix3d& R) {
    // Check P validity before update
    if (!P_.allFinite()) {
        std::cerr << "ERROR: P has NaN/Inf at start of updateGradient!\n";
        return;
    }
    
    // Transform sigma points to measurement space
    std::vector<Eigen::Matrix3d> predicted_gradients(NUM_SIGMA_POINTS);
    
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        // Simple synthetic gradient for now
        // In production, would compute from EGM2020
        Eigen::Vector3d pos = sigma_points_[i].state.p_ECEF;
        double r = pos.norm();
        double lat = std::asin(pos.z() / r);
        
        predicted_gradients[i] << 3.0, 0.1, 0.2,
                                  0.1, -1.5, 0.15,
                                  0.2, 0.15, -1.5;
        predicted_gradients[i] *= (1.0 + 0.5 * std::sin(lat));
    }
    
    // Compute mean predicted measurement
    Eigen::Matrix3d mean_gradient = Eigen::Matrix3d::Zero();
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        mean_gradient += weights_mean_(i) * predicted_gradients[i];
    }
    
    // Flatten for covariance calculations
    auto flatten = [](const Eigen::Matrix3d& m) {
        Eigen::VectorXd v(9);
        v << m(0,0), m(0,1), m(0,2), m(1,0), m(1,1), m(1,2), m(2,0), m(2,1), m(2,2);
        return v;
    };
    
    // Compute innovation covariance
    Eigen::Matrix<double, 9, 9> S = Eigen::Matrix<double, 9, 9>::Zero();
    Eigen::Matrix<double, ERROR_STATE_DIM, 9> T = Eigen::Matrix<double, ERROR_STATE_DIM, 9>::Zero();
    
    Eigen::VectorXd mean_grad_vec = flatten(mean_gradient);
    
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        Eigen::VectorXd grad_diff = flatten(predicted_gradients[i]) - mean_grad_vec;
        Eigen::Matrix<double, ERROR_STATE_DIM, 1> state_diff = 
            computeError(sigma_points_[i].state, nominal_state_);
        
        S += weights_cov_(i) * grad_diff * grad_diff.transpose();
        T += weights_cov_(i) * state_diff * grad_diff.transpose();
    }
    
    // Add measurement noise
    Eigen::Matrix<double, 9, 9> R_diag = Eigen::Matrix<double, 9, 9>::Zero();
    R_diag.diagonal() = flatten(R);
    S += R_diag;
    
    // Check if S is invertible
    Eigen::JacobiSVD<Eigen::Matrix<double, 9, 9>> svd(S);
    double cond = svd.singularValues()(0) / svd.singularValues()(8);
    if (cond > 1e10 || !S.allFinite()) {
        std::cerr << "WARNING: S matrix poorly conditioned or has NaN. Condition number: " << cond << "\n";
        // Use pseudo-inverse
        S += Eigen::Matrix<double, 9, 9>::Identity() * 1e-6;
    }
    
    // Kalman gain
    Eigen::Matrix<double, ERROR_STATE_DIM, 9> K = T * S.inverse();
    
    // Update
    Eigen::VectorXd innovation = flatten(measured - mean_gradient);
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> state_correction = K * innovation;
    
    // Apply correction to nominal state
    nominal_state_ = applyError(nominal_state_, state_correction);
    
    // Update covariance
    P_ = P_ - K * S * K.transpose();
    enforcePositiveDefinite(P_);
}

void UKF_Stable::updateAnomaly(double measured, double noise) {
    // Similar to gradient update but with scalar measurement
    std::vector<double> predicted_anomalies(NUM_SIGMA_POINTS);
    
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        // Simple synthetic anomaly
        Eigen::Vector3d pos = sigma_points_[i].state.p_ECEF;
        double r = pos.norm();
        double lat = std::asin(pos.z() / r);
        predicted_anomalies[i] = 10.0 * std::sin(lat * 10.0);
    }
    
    // Compute mean
    double mean_anomaly = 0.0;
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        mean_anomaly += weights_mean_(i) * predicted_anomalies[i];
    }
    
    // Compute covariances
    double S = 0.0;
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> T = Eigen::Matrix<double, ERROR_STATE_DIM, 1>::Zero();
    
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        double anomaly_diff = predicted_anomalies[i] - mean_anomaly;
        Eigen::Matrix<double, ERROR_STATE_DIM, 1> state_diff = 
            computeError(sigma_points_[i].state, nominal_state_);
        
        S += weights_cov_(i) * anomaly_diff * anomaly_diff;
        T += weights_cov_(i) * state_diff * anomaly_diff;
    }
    
    // Add measurement noise
    S += noise * noise;
    
    // Kalman gain
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> K = T / S;
    
    // Update
    double innovation = measured - mean_anomaly;
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> state_correction = K * innovation;
    
    // Apply correction
    nominal_state_ = applyError(nominal_state_, state_correction);
    
    // Update covariance
    P_ = P_ - K * K.transpose() * S;
    enforcePositiveDefinite(P_);
}