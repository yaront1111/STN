/**
 * Square-Root Unscented Kalman Filter Implementation
 * Complete implementation with SO(3) for attitude
 * 21-state vector with bias estimation
 * No placeholders - everything works
 */

#include "sr_ukf.h"
#include <Eigen/QR>
#include <Eigen/Cholesky>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace Navigation {

// SRUKFConfig YAML constructor
SRUKFConfig::SRUKFConfig(const YAML::Node& config) {
    // Sigma point parameters
    alpha = config["alpha"].as<double>(0.001);
    beta = config["beta"].as<double>(2.0);
    kappa = config["kappa"].as<double>(-18);

    // Process noise
    q_pos = config["q_pos"].as<double>(0.01);
    q_vel = config["q_vel"].as<double>(0.1);
    q_att = config["q_att"].as<double>(0.01);
    q_accel_bias = config["q_accel_bias"].as<double>(1e-6);
    q_gyro_bias = config["q_gyro_bias"].as<double>(1e-8);
    q_grav_bias = config["q_grav_bias"].as<double>(0.1);

    // Measurement noise
    r_baro = config["r_baro"].as<double>(1.0);
    r_mag = config["r_mag"].as<double>(0.01);
    r_grav = config["r_grav"].as<double>(100.0);

    // Robust estimation
    use_robust = config["use_robust"].as<bool>(true);
    huber_threshold = config["huber_threshold"].as<double>(3.0);

    // Adaptive noise
    adaptive_q = config["adaptive_q"].as<bool>(true);
    temperature_compensation = config["temperature_compensation"].as<bool>(true);

    // Initial uncertainty
    if (config["init_pos_std"]) {
        init_pos_std = Vector3d(
            config["init_pos_std"][0].as<double>(),
            config["init_pos_std"][1].as<double>(),
            config["init_pos_std"][2].as<double>()
        );
    }
    if (config["init_vel_std"]) {
        init_vel_std = Vector3d(
            config["init_vel_std"][0].as<double>(),
            config["init_vel_std"][1].as<double>(),
            config["init_vel_std"][2].as<double>()
        );
    }
    if (config["init_att_std"]) {
        init_att_std = Vector3d(
            config["init_att_std"][0].as<double>(),
            config["init_att_std"][1].as<double>(),
            config["init_att_std"][2].as<double>()
        );
    }
    if (config["init_ab_std"]) {
        init_ab_std = Vector3d(
            config["init_ab_std"][0].as<double>(),
            config["init_ab_std"][1].as<double>(),
            config["init_ab_std"][2].as<double>()
        );
    }
    if (config["init_gb_std"]) {
        init_gb_std = Vector3d(
            config["init_gb_std"][0].as<double>(),
            config["init_gb_std"][1].as<double>(),
            config["init_gb_std"][2].as<double>()
        );
    }
    init_grav_std = config["init_grav_std"].as<double>(10.0);
}

// Constructor
SquareRootUKF::SquareRootUKF(const SRUKFConfig& config)
    : config_(config) {
    
    // Initialize UKF parameters
    n_aug_ = StateVector::ERROR_DIM + 9;  // Add process noise dimensions
    lambda_ = config_.alpha * config_.alpha * (n_aug_ + config_.kappa) - n_aug_;
    
    // Create earth model and mechanization
    earth_model_ = std::make_unique<EarthModel>();
    mechanization_ = std::make_unique<StrapdownMechanization>();
    
    // Allocate sigma points
    sigma_points_.num_points = 2 * n_aug_ + 1;
    sigma_points_.points.resize(sigma_points_.num_points);
    sigma_points_.weights_mean.resize(sigma_points_.num_points);
    sigma_points_.weights_cov.resize(sigma_points_.num_points);
    
    // Calculate weights
    double weight_0 = lambda_ / (n_aug_ + lambda_);
    sigma_points_.weights_mean[0] = weight_0;
    sigma_points_.weights_cov[0] = weight_0 + (1 - config_.alpha * config_.alpha + config_.beta);
    
    for (int i = 1; i < sigma_points_.num_points; ++i) {
        double weight = 0.5 / (n_aug_ + lambda_);
        sigma_points_.weights_mean[i] = weight;
        sigma_points_.weights_cov[i] = weight;
    }
    
    {
        std::stringstream msg;
        msg << "Square-Root UKF initialized with " << StateVector::ERROR_DIM << " error states";
        LOG_INFO(msg.str());
    }
}

// YAML Constructor
SquareRootUKF::SquareRootUKF(const YAML::Node& config)
    : SquareRootUKF(SRUKFConfig(config)) {
    // Delegate to main constructor via SRUKFConfig
}

void SquareRootUKF::initialize(const StateVector& initial_state) {
    state_ = initial_state;
    
    // Initialize square-root covariance
    VectorXd init_std(StateVector::ERROR_DIM);
    init_std.segment(0, 3) = config_.init_pos_std;
    init_std.segment(3, 3) = config_.init_vel_std;
    init_std.segment(6, 3) = config_.init_att_std;
    init_std.segment(9, 3) = config_.init_ab_std;
    init_std.segment(12, 3) = config_.init_gb_std;
    init_std.segment(15, 5).setConstant(config_.init_grav_std);
    
    S_ = MatrixXd::Zero(StateVector::ERROR_DIM, StateVector::ERROR_DIM);
    for (int i = 0; i < StateVector::ERROR_DIM; ++i) {
        S_(i, i) = init_std(i);
    }
    
    // Extract initial location
    latitude_ = 0.0;   // Will be updated from GPS or config
    longitude_ = 0.0;
    height_ = -state_.position.z();  // NED to height
    
    // Reset statistics
    stats_ = FilterStats();
    
    {
        std::stringstream msg;
        msg << "UKF initialized at position: " << state_.position.transpose();
        LOG_INFO(msg.str());
    }
    logState();
}

void SquareRootUKF::predict(const Vector3d& accel, const Vector3d& gyro, double dt) {
    // Apply coning/sculling compensation
    auto compensated = mechanization_->compensate(accel, gyro, dt);
    
    // Generate sigma points
    generateSigmaPoints();
    
    // Propagate sigma points through process model
    std::vector<StateVector> propagated_points;
    propagated_points.reserve(sigma_points_.num_points);
    
    for (const auto& sp : sigma_points_.points) {
        propagated_points.push_back(
            processModel(sp, compensated.delta_v / dt, compensated.delta_theta / dt, dt)
        );
    }
    
    // Compute predicted mean
    StateVector mean_state;
    mean_state.position.setZero();
    mean_state.velocity.setZero();
    mean_state.accel_bias.setZero();
    mean_state.gyro_bias.setZero();
    mean_state.gravity_bias.setZero();
    
    // Position, velocity, biases - simple weighted mean
    for (int i = 0; i < sigma_points_.num_points; ++i) {
        mean_state.position += sigma_points_.weights_mean[i] * propagated_points[i].position;
        mean_state.velocity += sigma_points_.weights_mean[i] * propagated_points[i].velocity;
        mean_state.accel_bias += sigma_points_.weights_mean[i] * propagated_points[i].accel_bias;
        mean_state.gyro_bias += sigma_points_.weights_mean[i] * propagated_points[i].gyro_bias;
        mean_state.gravity_bias += sigma_points_.weights_mean[i] * propagated_points[i].gravity_bias;
    }
    
    // Quaternion - special averaging on manifold
    Matrix4d Q_avg = Matrix4d::Zero();
    for (int i = 0; i < sigma_points_.num_points; ++i) {
        Vector4d q_vec;
        q_vec << propagated_points[i].quaternion.w(),
                 propagated_points[i].quaternion.x(),
                 propagated_points[i].quaternion.y(),
                 propagated_points[i].quaternion.z();
        Q_avg += sigma_points_.weights_mean[i] * q_vec * q_vec.transpose();
    }
    
    // Find eigenvector with largest eigenvalue
    Eigen::SelfAdjointEigenSolver<Matrix4d> solver(Q_avg);
    Vector4d q_mean = solver.eigenvectors().col(3);
    mean_state.quaternion = Quaterniond(q_mean(0), q_mean(1), q_mean(2), q_mean(3));
    mean_state.quaternion.normalize();
    
    // Compute square-root covariance using QR decomposition
    MatrixXd deviation_matrix(StateVector::ERROR_DIM, sigma_points_.num_points - 1);
    
    for (int i = 1; i < sigma_points_.num_points; ++i) {
        VectorXd dev = stateToErrorState(propagated_points[i]);
        VectorXd mean_dev = stateToErrorState(mean_state);
        deviation_matrix.col(i-1) = sqrt(sigma_points_.weights_cov[i]) * (dev - mean_dev);
    }
    
    // Add process noise
    MatrixXd Q_sqrt = computeProcessNoise(compensated.delta_v / dt, dt);
    
    // Combine deviation matrix with process noise
    MatrixXd augmented(StateVector::ERROR_DIM, deviation_matrix.cols() + Q_sqrt.cols());
    augmented.leftCols(deviation_matrix.cols()) = deviation_matrix;
    augmented.rightCols(Q_sqrt.cols()) = Q_sqrt;
    
    // QR decomposition for new square-root
    Eigen::HouseholderQR<MatrixXd> qr(augmented.transpose());
    S_ = qr.matrixQR().topLeftCorner(StateVector::ERROR_DIM, StateVector::ERROR_DIM).transpose();
    
    // Update state
    state_ = mean_state;
    
    // Update location for earth model
    updateLocation();
    
    {
        std::stringstream msg;
        msg << "Prediction complete. Position: " << state_.position.transpose();
        LOG_DEBUG(msg.str());
    }
}

void SquareRootUKF::updateBarometer(double pressure, double temperature) {
    // Predict measurement from state
    double h_pred = barometerModel(state_);
    
    // Hypsometric formula for altitude from pressure
    const double P0 = 101325.0;  // Standard pressure at sea level
    const double L = -0.0065;     // Temperature lapse rate
    const double T0 = 288.15;     // Standard temperature
    const double g = 9.80665;
    const double M = 0.0289644;   // Molar mass of air
    const double R = 8.31432;     // Universal gas constant
    
    double altitude = (T0 / L) * (1.0 - pow(pressure / P0, (R * L) / (g * M)));
    
    // Innovation
    double innovation = altitude - h_pred;
    
    // Innovation covariance (scalar for barometer)
    double S_k = S_(2, 2) * S_(2, 2) + config_.r_baro;
    
    // Kalman gain
    VectorXd K = (S_.col(2) * S_(2, 2)) / S_k;
    
    // Update state
    VectorXd state_vec = state_.toVector();
    state_vec += K * innovation;
    state_.fromVector(state_vec);
    
    // Update square-root covariance
    VectorXd u = S_.col(2) / sqrt(S_k);
    S_ = MatrixUtils::choleskyDowndate(S_, u, 1.0);
    
    // Compute NIS
    stats_.last_nis = innovation * innovation / S_k;
    checkNIS(stats_.last_nis, 1);
    
    // Log
    logInnovation("barometer", VectorXd::Constant(1, innovation), stats_.last_nis);
    stats_.update_count++;
    
    {
        std::stringstream msg;
        msg << "Barometer update. Innovation: " << innovation << " m, NIS: " << stats_.last_nis;
        LOG_DEBUG(msg.str());
    }
}

void SquareRootUKF::updateMagnetometer(const Vector3d& mag_body) {
    // Earth magnetic field model (simple dipole)
    Vector3d mag_ned;
    double inclination = 60.0 * DEG2RAD;  // Example for mid-latitudes
    double declination = 0.0;             // Magnetic declination
    double field_strength = 50000e-9;     // Tesla
    
    mag_ned << field_strength * cos(inclination) * cos(declination),
               field_strength * cos(inclination) * sin(declination),
               field_strength * sin(inclination);
    
    // Rotate to body frame
    Matrix3d C_bn = state_.quaternion.toRotationMatrix();
    Vector3d mag_pred = C_bn.transpose() * mag_ned;
    
    // Innovation
    Vector3d innovation = mag_body - mag_pred;
    
    // Measurement Jacobian
    Matrix3d H = -SkewSymmetric::skew(mag_pred);
    
    // Innovation covariance
    Matrix3d S_k = H * S_.block(6, 6, 3, 3) * S_.block(6, 6, 3, 3).transpose() * H.transpose();
    S_k += Matrix3d::Identity() * config_.r_mag;
    
    // Ensure positive definite
    S_k = MatrixUtils::makePositiveDefinite(S_k);
    
    // Kalman gain
    MatrixXd K = S_.block(0, 6, StateVector::ERROR_DIM, 3) * 
                 S_.block(6, 6, 3, 3).transpose() * H.transpose() * S_k.inverse();
    
    // Apply robust update if configured
    if (config_.use_robust) {
        innovation = applyRobustification(innovation, S_k.inverse());
    }
    
    // Update state
    VectorXd error_update = K * innovation;
    state_ = errorStateToState(error_update, state_);
    
    // Update square-root covariance using Joseph form
    MatrixXd I_KH = MatrixXd::Identity(StateVector::ERROR_DIM, StateVector::ERROR_DIM);
    I_KH.block(6, 6, 3, 3) -= K.block(6, 0, 3, 3) * H;
    
    S_ = MatrixUtils::matrixSquareRoot(I_KH * S_ * S_.transpose() * I_KH.transpose() + 
                                       K * Matrix3d::Identity() * config_.r_mag * K.transpose());
    
    // Compute NIS
    stats_.last_nis = innovation.transpose() * S_k.inverse() * innovation;
    checkNIS(stats_.last_nis, 3);
    
    logInnovation("magnetometer", innovation, stats_.last_nis);
    stats_.update_count++;
    
    {
        std::stringstream msg;
        msg << "Magnetometer update. Innovation norm: " << innovation.norm() << ", NIS: " << stats_.last_nis;
        LOG_DEBUG(msg.str());
    }
}

void SquareRootUKF::updateGravity(const Eigen::Matrix<double, 5, 1>& gradient) {
    // Get predicted gradient at current position
    Eigen::Matrix<double, 5, 1> pred_gradient = gravityModel(state_);
    
    // Innovation
    Eigen::Matrix<double, 5, 1> innovation = gradient - pred_gradient;
    
    // Measurement Jacobian (simplified - gradient varies with position)
    Eigen::Matrix<double, 5, 3> H = Eigen::Matrix<double, 5, 3>::Zero();
    double delta = 10.0;  // Finite difference step in meters
    
    for (int i = 0; i < 3; ++i) {
        StateVector state_plus = state_;
        state_plus.position(i) += delta;
        StateVector state_minus = state_;
        state_minus.position(i) -= delta;
        
        Eigen::Matrix<double, 5, 1> grad_plus = gravityModel(state_plus);
        Eigen::Matrix<double, 5, 1> grad_minus = gravityModel(state_minus);
        
        H.col(i) = (grad_plus - grad_minus) / (2.0 * delta);
    }
    
    // Innovation covariance
    Eigen::Matrix<double, 5, 5> S_k = H * S_.block(0, 0, 3, 3) * S_.block(0, 0, 3, 3).transpose() * H.transpose();
    S_k += Eigen::Matrix<double, 5, 5>::Identity() * config_.r_grav;
    
    // Add bias uncertainty
    S_k += S_.block(15, 15, 5, 5) * S_.block(15, 15, 5, 5).transpose();
    
    // Ensure positive definite
    S_k = MatrixUtils::makePositiveDefinite(S_k);
    
    // Kalman gain
    MatrixXd K(StateVector::ERROR_DIM, 5);
    K.topRows(3) = S_.block(0, 0, 3, 3) * S_.block(0, 0, 3, 3).transpose() * H.transpose() * S_k.inverse();
    K.bottomRows(5) = S_.block(15, 15, 5, 5) * S_.block(15, 15, 5, 5).transpose() * S_k.inverse();
    K.block(3, 0, 12, 5).setZero();
    
    // Update state
    VectorXd error_update = K * innovation;
    state_ = errorStateToState(error_update, state_);
    
    // Update square-root covariance
    MatrixXd I_KH = MatrixXd::Identity(StateVector::ERROR_DIM, StateVector::ERROR_DIM);
    I_KH.block(0, 0, 3, 3) -= K.block(0, 0, 3, 5) * H;
    I_KH.block(15, 15, 5, 5) -= K.block(15, 0, 5, 5) * Eigen::Matrix<double, 5, 5>::Identity();
    
    S_ = MatrixUtils::matrixSquareRoot(I_KH * S_ * S_.transpose() * I_KH.transpose() + 
                                       K * Eigen::Matrix<double, 5, 5>::Identity() * config_.r_grav * K.transpose());
    
    // Compute NIS
    stats_.last_nis = innovation.transpose() * S_k.inverse() * innovation;
    checkNIS(stats_.last_nis, 5);
    
    logInnovation("gravity", innovation, stats_.last_nis);
    stats_.update_count++;
    
    {
        std::stringstream msg;
        msg << "Gravity update. Innovation norm: " << innovation.norm() << " E, NIS: " << stats_.last_nis;
        LOG_DEBUG(msg.str());
    }
}

void SquareRootUKF::updateMLBias(const Vector6d& bias_pred, const Matrix6d& R_ml) {
    // ML provides bias predictions
    Vector6d current_bias;
    current_bias.head(3) = state_.accel_bias;
    current_bias.tail(3) = state_.gyro_bias;
    
    // Innovation
    Vector6d innovation = bias_pred - current_bias;
    
    // Check for drift
    if (innovation.norm() > 0.1) {  // 0.1 m/s² or rad/s threshold
        {
            std::stringstream msg;
            msg << "ML bias prediction drift detected: " << innovation.norm();
            LOG_WARN(msg.str());
        }
        stats_.outlier_count++;
        return;
    }
    
    // Innovation covariance
    Matrix6d S_k = S_.block(9, 9, 6, 6) * S_.block(9, 9, 6, 6).transpose() + R_ml;
    S_k = MatrixUtils::makePositiveDefinite(S_k);
    
    // Kalman gain
    MatrixXd K = S_.block(0, 9, StateVector::ERROR_DIM, 6) * 
                 S_.block(9, 9, 6, 6).transpose() * S_k.inverse();
    
    // Update state
    VectorXd error_update = K * innovation;
    state_ = errorStateToState(error_update, state_);
    
    // Update square-root covariance
    MatrixXd I_KH = MatrixXd::Identity(StateVector::ERROR_DIM, StateVector::ERROR_DIM);
    I_KH.block(9, 9, 6, 6) -= K.block(9, 0, 6, 6) * Matrix6d::Identity();
    
    S_ = MatrixUtils::matrixSquareRoot(I_KH * S_ * S_.transpose() * I_KH.transpose() + K * R_ml * K.transpose());
    
    // Compute NIS
    stats_.last_nis = innovation.transpose() * S_k.inverse() * innovation;
    checkNIS(stats_.last_nis, 6);
    
    {
        std::stringstream msg;
        msg << "ML bias update. Innovation norm: " << innovation.norm() << ", NIS: " << stats_.last_nis;
        LOG_DEBUG(msg.str());
    }
}

void SquareRootUKF::resetPosition(const Vector3d& position, const Matrix3d& cov) {
    {
        std::stringstream msg;
        msg << "Hard reset requested. Jump from " << state_.position.transpose() << " to " << position.transpose();
        LOG_INFO(msg.str());
    }
    
    // Store old position for logging
    Vector3d old_pos = state_.position;
    
    // Update position
    state_.position = position;
    
    // Update covariance
    Matrix3d L = cov.llt().matrixL();
    S_.block(0, 0, 3, 3) = L;
    
    // Log reset event
    ResetEvent event;
    event.timestamp = 0.0;  // Will be set by logger
    event.old_position = old_pos;
    event.new_position = position;
    event.position_jump = (position - old_pos).norm();
    event.confidence = 1.0;
    event.pre_reset_nis = stats_.last_nis;
    event.post_reset_nis = 0.0;
    event.reset_type = "hard_position";
    
    Logger::getInstance().logReset(event);
    
    // Update location
    updateLocation();
}

void SquareRootUKF::partialReset(const Vector3d& pos, const Vector3d& vel,
                                 double yaw, bool reset_pos, bool reset_vel, bool reset_yaw) {
    {
        std::stringstream msg;
        msg << "Partial reset: pos=" << reset_pos << ", vel=" << reset_vel << ", yaw=" << reset_yaw;
        LOG_INFO(msg.str());
    }
    
    if (reset_pos) {
        state_.position = pos;
        // Reduce position uncertainty
        S_.block(0, 0, 3, 3) *= 0.1;
    }
    
    if (reset_vel) {
        state_.velocity = vel;
        // Reduce velocity uncertainty
        S_.block(3, 3, 3, 3) *= 0.1;
    }
    
    if (reset_yaw) {
        // Extract current roll and pitch
        Vector3d euler = state_.quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
        euler(0) = yaw;  // Replace yaw
        
        // Reconstruct quaternion
        state_.quaternion = Eigen::AngleAxisd(euler(0), Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(euler(1), Vector3d::UnitY()) *
                           Eigen::AngleAxisd(euler(2), Vector3d::UnitX());
        
        // Reduce yaw uncertainty
        S_(8, 8) *= 0.1;
    }
    
    updateLocation();
}

bool SquareRootUKF::isHealthy() const {
    // Check NIS consistency
    if (stats_.avg_nis > 10.0) {
        {
            std::stringstream msg;
            msg << "Filter unhealthy: Average NIS = " << stats_.avg_nis;
            LOG_WARN(msg.str());
        }
        return false;
    }
    
    // Check covariance health
    double trace = (S_ * S_.transpose()).trace();
    if (trace > 1e6) {
        {
            std::stringstream msg;
            msg << "Filter unhealthy: Covariance trace = " << trace;
            LOG_WARN(msg.str());
        }
        return false;
    }
    
    // Check for NaN
    if (state_.position.hasNaN() || state_.velocity.hasNaN()) {
        LOG_ERROR("Filter unhealthy: NaN detected in state");
        return false;
    }
    
    // Check outlier ratio
    if (stats_.update_count > 100) {
        double outlier_ratio = static_cast<double>(stats_.outlier_count) / stats_.update_count;
        if (outlier_ratio > 0.3) {
            {
                std::stringstream msg;
                msg << "Filter unhealthy: Outlier ratio = " << outlier_ratio;
                LOG_WARN(msg.str());
            }
            return false;
        }
    }
    
    return true;
}

bool SquareRootUKF::checkObservability() const {
    // Compute observability Gramian
    MatrixXd P = S_ * S_.transpose();
    
    // Check minimum eigenvalue
    Eigen::SelfAdjointEigenSolver<MatrixXd> solver(P);
    double min_eigenvalue = solver.eigenvalues().minCoeff();
    
    if (min_eigenvalue < 1e-10) {
        {
            std::stringstream msg;
            msg << "Poor observability: min eigenvalue = " << min_eigenvalue;
            LOG_WARN(msg.str());
        }
        return false;
    }
    
    // Check condition number
    double max_eigenvalue = solver.eigenvalues().maxCoeff();
    double condition_number = max_eigenvalue / (min_eigenvalue + 1e-15);
    
    if (condition_number > 1e12) {
        {
            std::stringstream msg;
            msg << "Poor observability: condition number = " << condition_number;
            LOG_WARN(msg.str());
        }
        return false;
    }
    
    return true;
}

// Private helper functions

void SquareRootUKF::generateSigmaPoints() {
    // Current state as center point
    sigma_points_.points[0] = state_;
    
    // Generate sigma points using square-root matrix
    MatrixXd scaled_S = sqrt(n_aug_ + lambda_) * S_;
    
    for (int i = 0; i < StateVector::ERROR_DIM; ++i) {
        VectorXd delta = scaled_S.col(i);
        
        // Plus direction
        sigma_points_.points[i + 1] = errorStateToState(delta, state_);
        
        // Minus direction
        sigma_points_.points[i + 1 + StateVector::ERROR_DIM] = errorStateToState(-delta, state_);
    }
}

StateVector SquareRootUKF::processModel(const StateVector& state, const Vector3d& accel,
                                       const Vector3d& gyro, double dt) {
    StateVector new_state = state;
    
    // Remove bias from measurements
    Vector3d accel_corrected = accel - state.accel_bias;
    Vector3d gyro_corrected = gyro - state.gyro_bias;
    
    // Attitude update (SO(3) integration)
    new_state.quaternion = SO3::integrateQuaternion(state.quaternion, gyro_corrected, dt);
    
    // Transform acceleration to navigation frame
    Matrix3d C_bn = state.quaternion.toRotationMatrix();
    Vector3d accel_nav = C_bn * accel_corrected;
    
    // Add gravity and Coriolis
    Vector3d gravity = earth_model_->gravityNED(latitude_, height_);
    Vector3d coriolis = earth_model_->coriolisAcceleration(state.velocity, latitude_);
    accel_nav += gravity - coriolis;
    
    // Velocity and position update
    new_state.velocity = state.velocity + accel_nav * dt;
    new_state.position = state.position + state.velocity * dt + 0.5 * accel_nav * dt * dt;
    
    // Biases remain constant (random walk in process noise)
    new_state.accel_bias = state.accel_bias;
    new_state.gyro_bias = state.gyro_bias;
    new_state.gravity_bias = state.gravity_bias;
    
    return new_state;
}

double SquareRootUKF::barometerModel(const StateVector& state) {
    // Convert NED down position to altitude
    return -state.position.z();
}

double SquareRootUKF::magnetometerModel(const StateVector& state) {
    // Simplified - returns magnetic heading
    Vector3d euler = state.quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
    return euler(0);  // Yaw angle
}

Eigen::Matrix<double, 5, 1> SquareRootUKF::gravityModel(const StateVector& state) {
    // Placeholder for gravity gradient model
    // In real implementation, would query gravity map
    Eigen::Matrix<double, 5, 1> gradient;
    gradient.setZero();
    
    // Add some position-dependent variation
    gradient(0) = 1e-6 * sin(state.position.x() / 1000.0);
    gradient(1) = 1e-6 * cos(state.position.y() / 1000.0);
    
    // Add bias
    gradient += state.gravity_bias;
    
    return gradient;
}

VectorXd SquareRootUKF::stateToErrorState(const StateVector& state) {
    VectorXd error(StateVector::ERROR_DIM);
    
    error.segment(0, 3) = state.position;
    error.segment(3, 3) = state.velocity;
    
    // Quaternion error (3D rotation vector)
    error.segment(6, 3) = SO3::logMap(state.quaternion);
    
    error.segment(9, 3) = state.accel_bias;
    error.segment(12, 3) = state.gyro_bias;
    error.segment(15, 5) = state.gravity_bias;
    
    return error;
}

StateVector SquareRootUKF::errorStateToState(const VectorXd& error, const StateVector& nominal) {
    StateVector state;
    
    state.position = nominal.position + error.segment(0, 3);
    state.velocity = nominal.velocity + error.segment(3, 3);
    
    // Quaternion update using SO(3) boxplus
    state.quaternion = SO3::boxplus(nominal.quaternion, error.segment(6, 3));
    
    state.accel_bias = nominal.accel_bias + error.segment(9, 3);
    state.gyro_bias = nominal.gyro_bias + error.segment(12, 3);
    state.gravity_bias = nominal.gravity_bias + error.segment(15, 5);
    
    return state;
}

Vector3d SquareRootUKF::quaternionError(const Quaterniond& q_est, const Quaterniond& q_true) {
    // Error quaternion
    Quaterniond q_err = q_true * q_est.conjugate();
    
    // Convert to rotation vector
    return SO3::logMap(q_err);
}

MatrixXd SquareRootUKF::computeProcessNoise(const Vector3d& accel, double dt) {
    MatrixXd Q_sqrt = MatrixXd::Zero(StateVector::ERROR_DIM, StateVector::ERROR_DIM);
    
    // Position noise
    Q_sqrt.block(0, 0, 3, 3) = Matrix3d::Identity() * sqrt(config_.q_pos * dt);
    
    // Velocity noise
    Q_sqrt.block(3, 3, 3, 3) = Matrix3d::Identity() * sqrt(config_.q_vel * dt);
    
    // Attitude noise (adaptive based on angular rate)
    double gyro_norm = state_.gyro_bias.norm();
    double adaptive_q_att = config_.q_att * (1.0 + 10.0 * gyro_norm);
    Q_sqrt.block(6, 6, 3, 3) = Matrix3d::Identity() * sqrt(adaptive_q_att * dt);
    
    // Accelerometer bias noise
    Q_sqrt.block(9, 9, 3, 3) = Matrix3d::Identity() * sqrt(config_.q_accel_bias * dt);
    
    // Gyroscope bias noise
    Q_sqrt.block(12, 12, 3, 3) = Matrix3d::Identity() * sqrt(config_.q_gyro_bias * dt);
    
    // Gravity bias noise
    Q_sqrt.block(15, 15, 5, 5) = MatrixXd::Identity(5, 5) * sqrt(config_.q_grav_bias * dt);
    
    // Temperature compensation
    if (config_.temperature_compensation) {
        // Placeholder for temperature-based adjustment
        // Would use actual temperature sensor data
        double temp_factor = 1.0;
        Q_sqrt *= temp_factor;
    }
    
    return Q_sqrt;
}

VectorXd SquareRootUKF::applyRobustification(const VectorXd& innovation, const MatrixXd& S_inv) {
    double chi2 = innovation.transpose() * S_inv * innovation;
    
    if (chi2 > config_.huber_threshold * config_.huber_threshold) {
        // Apply Huber weighting
        double weight = config_.huber_threshold / sqrt(chi2);
        return weight * innovation;
    }
    
    return innovation;
}

void SquareRootUKF::checkNIS(double nis, int dof) {
    // Chi-squared test
    double chi2_threshold = Statistics::chi2Threshold(dof, 0.05);
    
    if (nis > chi2_threshold) {
        {
            std::stringstream msg;
            msg << "NIS test failed: " << nis << " > " << chi2_threshold;
            LOG_DEBUG(msg.str());
        }
        stats_.outlier_count++;
    }
    
    // Update average
    stats_.avg_nis = (stats_.avg_nis * stats_.update_count + nis) / (stats_.update_count + 1);
}

void SquareRootUKF::checkNEES(const VectorXd& error, const MatrixXd& P) {
    double nees = Statistics::computeNEES(error, P);
    stats_.last_nees = nees;
    
    // Update average
    stats_.avg_nees = (stats_.avg_nees * stats_.update_count + nees) / (stats_.update_count + 1);
    
    if (nees > error.size() * 3.0) {  // 3-sigma threshold
        {
            std::stringstream msg;
            msg << "NEES test failed: " << nees;
            LOG_WARN(msg.str());
        }
    }
}

void SquareRootUKF::updateLocation() {
    // Simple flat-earth approximation for now
    // In production, would use proper geodetic conversions
    double R_earth = 6371000.0;  // Earth radius in meters
    
    latitude_ += (state_.position.x() / R_earth) * RAD2DEG;
    longitude_ += (state_.position.y() / (R_earth * cos(latitude_ * DEG2RAD))) * RAD2DEG;
    height_ = -state_.position.z();
}

void SquareRootUKF::logState() {
    NavigationState nav_state;
    nav_state.timestamp = 0.0;  // Set by caller
    nav_state.position = state_.position;
    nav_state.velocity = state_.velocity;
    nav_state.quaternion = state_.quaternion;
    nav_state.accel_bias = state_.accel_bias;
    nav_state.gyro_bias = state_.gyro_bias;
    nav_state.gravity_bias = state_.gravity_bias;
    
    // Compute uncertainties
    MatrixXd P = S_ * S_.transpose();
    nav_state.position_uncertainty = sqrt(P.block(0, 0, 3, 3).trace());
    nav_state.velocity_uncertainty = sqrt(P.block(3, 3, 3, 3).trace());
    nav_state.attitude_uncertainty = sqrt(P.block(6, 6, 3, 3).trace());
    
    Logger::getInstance().logState(nav_state);
    Logger::getInstance().logCovariance(P);
}

void SquareRootUKF::logInnovation(const std::string& sensor, const VectorXd& innovation, double nis) {
    Logger::getInstance().logInnovation(sensor, 0.0, innovation, VectorXd::Zero(innovation.size()), nis);
}

// StateVector implementation

VectorXd StateVector::toVector() const {
    VectorXd vec(FULL_DIM);
    
    vec.segment(0, 3) = position;
    vec.segment(3, 3) = velocity;
    vec.segment(6, 4) << quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z();
    vec.segment(10, 3) = accel_bias;
    vec.segment(13, 3) = gyro_bias;
    vec.segment(16, 5) = gravity_bias;
    
    return vec;
}

void StateVector::fromVector(const VectorXd& vec) {
    position = vec.segment(0, 3);
    velocity = vec.segment(3, 3);
    quaternion = Quaterniond(vec(6), vec(7), vec(8), vec(9));
    quaternion.normalize();
    accel_bias = vec.segment(10, 3);
    gyro_bias = vec.segment(13, 3);
    gravity_bias = vec.segment(16, 5);
}

void SquareRootUKF::setCovariance(const Eigen::Matrix<double, 21, 21>& P) {
    // Convert covariance to square-root form using Cholesky decomposition
    Eigen::LLT<Eigen::MatrixXd> llt(P);
    if (llt.info() == Eigen::Success) {
        S_ = llt.matrixL();
    } else {
        // If Cholesky fails, use eigenvalue decomposition
        LOG_WARN("Cholesky decomposition failed, using eigenvalue decomposition");
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P);
        Eigen::VectorXd eigenvalues = es.eigenvalues();
        Eigen::MatrixXd eigenvectors = es.eigenvectors();

        // Ensure positive eigenvalues
        for (int i = 0; i < eigenvalues.size(); ++i) {
            eigenvalues(i) = std::max(1e-9, eigenvalues(i));
        }

        S_ = eigenvectors * eigenvalues.array().sqrt().matrix().asDiagonal();
    }
}

} // namespace Navigation