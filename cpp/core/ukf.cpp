#include "ukf.h"
#include "gravity_model.h"
#include <iostream>
#include <cmath>

// State dimension constants
constexpr int POS_IDX = 0;
constexpr int VEL_IDX = 3;
constexpr int ATT_IDX = 6;
constexpr int BA_IDX = 10;
constexpr int BG_IDX = 13;
constexpr int CLK_IDX = 16;

void UKF::computeWeights() {
    lambda_ = cfg_.alpha * cfg_.alpha * (STATE_DIM + cfg_.kappa) - STATE_DIM;
    
    weights_mean_ = Eigen::VectorXd(SIGMA_POINTS);
    weights_cov_ = Eigen::VectorXd(SIGMA_POINTS);
    
    // Weight for mean state
    weights_mean_(0) = lambda_ / (STATE_DIM + lambda_);
    weights_cov_(0) = lambda_ / (STATE_DIM + lambda_) + (1 - cfg_.alpha * cfg_.alpha + cfg_.beta);
    
    // Weights for other sigma points
    double weight = 0.5 / (STATE_DIM + lambda_);
    for (int i = 1; i < SIGMA_POINTS; ++i) {
        weights_mean_(i) = weight;
        weights_cov_(i) = weight;
    }
}

void UKF::init(const State& x0, const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P0) {
    x_ = x0;
    P_ = P0;
    sigma_points_.resize(SIGMA_POINTS);
}

Eigen::VectorXd UKF::stateToVector(const State& x) {
    Eigen::VectorXd vec(STATE_DIM);
    vec.segment<3>(POS_IDX) = x.p_ECEF;
    vec.segment<3>(VEL_IDX) = x.v_ECEF;
    
    // Convert quaternion to rotation vector for additive updates
    Eigen::Quaterniond q_norm = x.q_ECEF_B.normalized();
    Eigen::AngleAxisd aa(q_norm);
    vec.segment<3>(ATT_IDX) = aa.axis() * aa.angle();
    
    vec.segment<3>(BA_IDX) = x.b_a;
    vec.segment<3>(BG_IDX) = x.b_g;
    vec(CLK_IDX) = x.dt;
    vec(CLK_IDX + 1) = x.df;
    
    return vec;
}

State UKF::vectorToState(const Eigen::VectorXd& vec) {
    State x;
    x.p_ECEF = vec.segment<3>(POS_IDX);
    x.v_ECEF = vec.segment<3>(VEL_IDX);
    
    // Convert rotation vector back to quaternion
    Eigen::Vector3d rot_vec = vec.segment<3>(ATT_IDX);
    double angle = rot_vec.norm();
    if (angle > 1e-10) {
        x.q_ECEF_B = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rot_vec / angle));
        x.q_ECEF_B.normalize();
    } else {
        x.q_ECEF_B = Eigen::Quaterniond::Identity();
    }
    
    x.b_a = vec.segment<3>(BA_IDX);
    x.b_g = vec.segment<3>(BG_IDX);
    x.dt = vec(CLK_IDX);
    x.df = vec(CLK_IDX + 1);
    
    return x;
}

void UKF::generateSigmaPoints() {
    // Check if P is valid
    if (!P_.allFinite()) {
        std::cerr << "WARNING: Covariance matrix P has NaN/Inf values!" << std::endl;
        // Reset to identity
        P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    }
    
    // Ensure P is symmetric before Cholesky decomposition
    Eigen::MatrixXd P_sym = (P_ + P_.transpose()) / 2.0;
    
    // Add small diagonal regularization if needed
    double min_eig = 1e-10;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P_sym);
    if (es.eigenvalues().minCoeff() < min_eig) {
        P_sym += Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * min_eig;
    }
    
    // Compute matrix square root using Cholesky decomposition
    Eigen::MatrixXd sqrt_P;
    Eigen::LLT<Eigen::MatrixXd> llt((STATE_DIM + lambda_) * P_sym);
    
    if (llt.info() != Eigen::Success) {
        // Cholesky failed - use more robust eigenvalue decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es2((STATE_DIM + lambda_) * P_sym);
        Eigen::MatrixXd D = es2.eigenvalues().array().max(0).sqrt().matrix().asDiagonal();
        sqrt_P = es2.eigenvectors() * D;
    } else {
        sqrt_P = llt.matrixL();
    }
    
    // Mean state
    Eigen::VectorXd x_vec = stateToVector(x_);
    sigma_points_[0] = x_;
    
    // Generate sigma points
    for (int i = 0; i < STATE_DIM; ++i) {
        Eigen::VectorXd delta = sqrt_P.col(i);
        
        // Positive direction
        sigma_points_[i + 1] = vectorToState(x_vec + delta);
        
        // Negative direction
        sigma_points_[i + 1 + STATE_DIM] = vectorToState(x_vec - delta);
    }
}

State UKF::propagateState(const State& x, double dt) {
    State x_next = x;
    
    // Earth rotation rate
    const double omega_earth = 7.292115e-5;  // rad/s
    
    // Propagate position
    x_next.p_ECEF += x.v_ECEF * dt;
    
    // Propagate velocity (including Earth rotation effects)
    Eigen::Vector3d omega_earth_vec(0, 0, omega_earth);
    Eigen::Vector3d coriolis = -2.0 * omega_earth_vec.cross(x.v_ECEF);
    Eigen::Vector3d centrifugal = -omega_earth_vec.cross(omega_earth_vec.cross(x.p_ECEF));
    x_next.v_ECEF += (coriolis + centrifugal) * dt;
    
    // Propagate attitude (simplified - assumes small rotation)
    // In real implementation, would integrate angular rates
    
    // Propagate clock states
    x_next.dt += x.df * dt;
    x_next.df += x.ddf * dt;
    
    // Biases remain constant (random walk)
    
    x_next.t = x.t + dt;
    
    return x_next;
}

void UKF::predict(double dt) {
    // Generate sigma points
    generateSigmaPoints();
    
    // Propagate each sigma point
    std::vector<State> propagated_sigma(SIGMA_POINTS);
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        propagated_sigma[i] = propagateState(sigma_points_[i], dt);
    }
    
    // Compute predicted mean
    Eigen::VectorXd x_mean = Eigen::VectorXd::Zero(STATE_DIM);
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        x_mean += weights_mean_(i) * stateToVector(propagated_sigma[i]);
    }
    x_ = vectorToState(x_mean);
    
    // Compute predicted covariance
    P_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        Eigen::VectorXd diff = stateToVector(propagated_sigma[i]) - x_mean;
        P_ += weights_cov_(i) * diff * diff.transpose();
    }
    
    // Add process noise
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    Eigen::VectorXd q_diag(STATE_DIM);
    q_diag.segment<3>(0) = Eigen::Vector3d::Constant(cfg_.q_pos * dt * dt);
    q_diag.segment<3>(3) = Eigen::Vector3d::Constant(cfg_.q_vel * dt);
    q_diag.segment<3>(6) = Eigen::Vector3d::Constant(cfg_.q_att * dt);
    q_diag.segment<3>(10) = Eigen::Vector3d::Constant(cfg_.q_acc_bias * dt);
    q_diag.segment<3>(13) = Eigen::Vector3d::Constant(cfg_.q_gyro_bias * dt);
    q_diag(16) = cfg_.q_clock_offset * dt;
    q_diag(17) = cfg_.q_clock_drift * dt;
    Q.diagonal() = q_diag;
    
    P_ += Q;
    
    // Enforce symmetry to prevent numerical instability
    P_ = (P_ + P_.transpose()) / 2.0;
    
    // Ensure positive definiteness
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P_);
    if (es.eigenvalues().minCoeff() < 1e-10) {
        P_ += Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 1e-9;
    }
    
    // Update sigma points for measurement update
    sigma_points_ = propagated_sigma;
}

Eigen::Matrix3d UKF::computeGradientFromPosition(const Eigen::Vector3d& pos_ECEF) {
    // Convert ECEF to geodetic
    // Simplified - in production would use proper WGS84 conversion
    double r = pos_ECEF.norm();
    double lat = std::asin(pos_ECEF.z() / r);
    // double lon = std::atan2(pos_ECEF.y(), pos_ECEF.x());  // Not used yet
    // double alt = r - 6371000.0;  // Not used yet
    
    // Compute gravity gradient from EGM2020 model
    // For now, return a synthetic gradient for testing
    Eigen::Matrix3d gradient;
    gradient << 3.0, 0.1, 0.2,
                0.1, -1.5, 0.15,
                0.2, 0.15, -1.5;
    
    // Add latitude-dependent variation
    gradient *= (1.0 + 0.5 * std::sin(lat));
    
    return gradient;  // In Eötvös units
}

void UKF::updateGravityGradient(const Eigen::Matrix3d& measured_gradient,
                                const Eigen::Matrix3d& gradient_noise) {
    // Transform measurements through sigma points
    std::vector<Eigen::Matrix3d> predicted_gradients(SIGMA_POINTS);
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        predicted_gradients[i] = computeGradientFromPosition(sigma_points_[i].p_ECEF);
    }
    
    // Compute mean predicted gradient
    Eigen::Matrix3d mean_gradient = Eigen::Matrix3d::Zero();
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        mean_gradient += weights_mean_(i) * predicted_gradients[i];
    }
    
    // Flatten gradient tensor to vector for covariance computation
    auto flatten = [](const Eigen::Matrix3d& m) {
        Eigen::VectorXd v(9);
        v << m(0,0), m(0,1), m(0,2), m(1,0), m(1,1), m(1,2), m(2,0), m(2,1), m(2,2);
        return v;
    };
    
    // Compute innovation covariance
    Eigen::MatrixXd Pyy = Eigen::MatrixXd::Zero(9, 9);
    Eigen::MatrixXd Pxy = Eigen::MatrixXd::Zero(STATE_DIM, 9);
    
    Eigen::VectorXd mean_grad_vec = flatten(mean_gradient);
    Eigen::VectorXd x_mean = stateToVector(x_);
    
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        Eigen::VectorXd grad_diff = flatten(predicted_gradients[i]) - mean_grad_vec;
        Eigen::VectorXd state_diff = stateToVector(sigma_points_[i]) - x_mean;
        
        Pyy += weights_cov_(i) * grad_diff * grad_diff.transpose();
        Pxy += weights_cov_(i) * state_diff * grad_diff.transpose();
    }
    
    // Add measurement noise
    Pyy += flatten(gradient_noise).asDiagonal();
    
    // Compute Kalman gain
    Eigen::MatrixXd K = Pxy * Pyy.inverse();
    
    // Update state
    Eigen::VectorXd innovation = flatten(measured_gradient - mean_gradient);
    Eigen::VectorXd x_updated = x_mean + K * innovation;
    x_ = vectorToState(x_updated);
    
    // Update covariance
    P_ = P_ - K * Pyy * K.transpose();
    
    // Ensure positive definiteness
    P_ = (P_ + P_.transpose()) / 2.0;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P_);
    if (es.eigenvalues().minCoeff() < 1e-10) {
        P_ += Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 1e-9;
    }
}

void UKF::updateGravityAnomaly(double measured_anomaly, double anomaly_noise) {
    // Similar to gradient update but with scalar measurement
    // Transform measurements through sigma points
    std::vector<double> predicted_anomalies(SIGMA_POINTS);
    
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        // Get gravity at this position
        // Simplified - would use EGM2020 in production
        double lat = std::asin(sigma_points_[i].p_ECEF.z() / sigma_points_[i].p_ECEF.norm());
        predicted_anomalies[i] = 10.0 * std::sin(lat * 10.0);  // Synthetic anomaly in mGal
    }
    
    // Compute mean predicted anomaly
    double mean_anomaly = 0.0;
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        mean_anomaly += weights_mean_(i) * predicted_anomalies[i];
    }
    
    // Compute covariances
    double Pyy = 0.0;
    Eigen::VectorXd Pxy = Eigen::VectorXd::Zero(STATE_DIM);
    
    Eigen::VectorXd x_mean = stateToVector(x_);
    
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        double anomaly_diff = predicted_anomalies[i] - mean_anomaly;
        Eigen::VectorXd state_diff = stateToVector(sigma_points_[i]) - x_mean;
        
        Pyy += weights_cov_(i) * anomaly_diff * anomaly_diff;
        Pxy += weights_cov_(i) * state_diff * anomaly_diff;
    }
    
    // Add measurement noise
    Pyy += anomaly_noise * anomaly_noise;
    
    // Compute Kalman gain
    Eigen::VectorXd K = Pxy / Pyy;
    
    // Update state
    double innovation = measured_anomaly - mean_anomaly;
    Eigen::VectorXd x_updated = x_mean + K * innovation;
    x_ = vectorToState(x_updated);
    
    // Update covariance
    P_ = P_ - K * Pxy.transpose();
    
    // Ensure positive definiteness and symmetry
    P_ = (P_ + P_.transpose()) / 2.0;
    
    // Add regularization if needed
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P_);
    if (es.eigenvalues().minCoeff() < 1e-10) {
        P_ += Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 1e-9;
    }
}

void UKF::propagateWithIMU(const ImuSample& imu, double dt) {
    // Validate IMU measurements (reject unrealistic values)
    const double MAX_ACC = 50.0;  // 5g max acceleration
    const double MAX_GYRO = 3.0;  // ~170 deg/s max rotation
    
    if (imu.acc_mps2.norm() > MAX_ACC || imu.gyro_rps.norm() > MAX_GYRO) {
        // Skip this measurement - too large to be real
        return;
    }
    
    // Check for NaN
    if (!imu.acc_mps2.allFinite() || !imu.gyro_rps.allFinite()) {
        return;
    }
    
    // First predict with dynamics
    predict(dt);
    
    // Then update state with IMU measurements
    // This is simplified - in production would have full IMU mechanization
    State& x = x_;
    
    // Remove bias from measurements
    Eigen::Vector3d acc_corrected = imu.acc_mps2 - x.b_a;
    Eigen::Vector3d gyro_corrected = imu.gyro_rps - x.b_g;
    
    // Update velocity (simplified - ignores gravity and Coriolis)
    x.v_ECEF += x.q_ECEF_B * acc_corrected * dt;
    
    // Update attitude
    Eigen::Vector3d angle_increment = gyro_corrected * dt;
    double angle = angle_increment.norm();
    if (angle > 1e-10) {
        Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, angle_increment / angle));
        x.q_ECEF_B = x.q_ECEF_B * dq;
        x.q_ECEF_B.normalize();
    }
    
    // Update time
    x.t = imu.t;
}

void UKF::updateClock(double offset_s, double drift_ppm) {
    // Simple scalar update for clock states
    // Measurement model: z = [dt, df]
    
    // Predicted measurements from sigma points
    std::vector<Eigen::Vector2d> predicted_clock(SIGMA_POINTS);
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        predicted_clock[i] << sigma_points_[i].dt, sigma_points_[i].df;
    }
    
    // Compute mean predicted measurement
    Eigen::Vector2d mean_clock = Eigen::Vector2d::Zero();
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        mean_clock += weights_mean_(i) * predicted_clock[i];
    }
    
    // Compute covariances
    Eigen::Matrix2d Pyy = Eigen::Matrix2d::Zero();
    Eigen::Matrix<double, STATE_DIM, 2> Pxy = Eigen::Matrix<double, STATE_DIM, 2>::Zero();
    
    Eigen::VectorXd x_mean = stateToVector(x_);
    
    for (int i = 0; i < SIGMA_POINTS; ++i) {
        Eigen::Vector2d clock_diff = predicted_clock[i] - mean_clock;
        Eigen::VectorXd state_diff = stateToVector(sigma_points_[i]) - x_mean;
        
        Pyy += weights_cov_(i) * clock_diff * clock_diff.transpose();
        Pxy += weights_cov_(i) * state_diff * clock_diff.transpose();
    }
    
    // Add measurement noise
    Pyy(0,0) += 1e-18;  // Clock offset noise (s²)
    Pyy(1,1) += 1e-24;  // Clock drift noise ((s/s)²)
    
    // Kalman gain
    Eigen::Matrix<double, STATE_DIM, 2> K = Pxy * Pyy.inverse();
    
    // Update state
    Eigen::Vector2d innovation;
    innovation << offset_s - mean_clock(0), drift_ppm * 1e-6 - mean_clock(1);
    Eigen::VectorXd x_updated = x_mean + K * innovation;
    x_ = vectorToState(x_updated);
    
    // Update covariance
    P_ = P_ - K * Pyy * K.transpose();
    
    // Ensure positive definiteness
    P_ = (P_ + P_.transpose()) / 2.0;
}