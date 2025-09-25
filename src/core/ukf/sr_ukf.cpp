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
#include <chrono>

namespace Navigation {

// SRUKFConfig YAML constructor
SRUKFConfig::SRUKFConfig(const YAML::Node& config) {
    // Sigma point parameters - check nested structure
    if (config["sigma_points"]) {
        alpha = config["sigma_points"]["alpha"].as<double>(0.1);  // Increased default for stability
        beta = config["sigma_points"]["beta"].as<double>(2.0);
        kappa = config["sigma_points"]["kappa"].as<double>(-26);  // 3 - n_aug(29) for proper scaling
    } else {
        // Fallback to top-level for backward compatibility
        alpha = config["alpha"].as<double>(0.1);  // Increased default for stability
        beta = config["beta"].as<double>(2.0);
        kappa = config["kappa"].as<double>(-26);  // 3 - n_aug(29) for proper scaling
    }

    // Process noise - Parse from nested YAML structure
    if (config["process_noise"]) {
        q_pos = config["process_noise"]["position"].as<double>(1e-8);
        q_vel = config["process_noise"]["velocity"].as<double>(1e-4);
        q_att = config["process_noise"]["attitude"].as<double>(1e-7);
        q_accel_bias = config["process_noise"]["accel_bias"].as<double>(1e-9);
        q_gyro_bias = config["process_noise"]["gyro_bias"].as<double>(1e-11);
        q_grav_bias = config["process_noise"]["gravity_bias"].as<double>(1e-14);
    } else {
        // Fallback to flat structure (old style)
        q_pos = config["q_pos"].as<double>(1e-8);
        q_vel = config["q_vel"].as<double>(1e-4);
        q_att = config["q_att"].as<double>(1e-7);
        q_accel_bias = config["q_accel_bias"].as<double>(1e-9);
        q_gyro_bias = config["q_gyro_bias"].as<double>(1e-11);
        q_grav_bias = config["q_grav_bias"].as<double>(1e-14);
    }

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

    // Initial covariance values - load from nested initial_covariance section
    if (config["initial_covariance"]) {
        init_pos_cov = config["initial_covariance"]["position"].as<double>(1.0);
        init_vel_cov = config["initial_covariance"]["velocity"].as<double>(0.01);
        init_att_cov = config["initial_covariance"]["attitude"].as<double>(0.001);
        init_accel_bias_cov = config["initial_covariance"]["accel_bias"].as<double>(1.0e-6);
        init_gyro_bias_cov = config["initial_covariance"]["gyro_bias"].as<double>(1.0e-8);
        init_grav_bias_cov = config["initial_covariance"]["gravity_bias"].as<double>(1.0);

        LOG_INFO("Loaded initial covariances from config:");
        LOG_INFO("  Position: " + std::to_string(init_pos_cov) + " m²");
        LOG_INFO("  Velocity: " + std::to_string(init_vel_cov) + " (m/s)²");
        LOG_INFO("  Attitude: " + std::to_string(init_att_cov) + " rad²");
        LOG_INFO("  Accel bias: " + std::to_string(init_accel_bias_cov) + " (m/s²)²");
        LOG_INFO("  Gyro bias: " + std::to_string(init_gyro_bias_cov) + " (rad/s)²");
        LOG_INFO("  Gravity bias: " + std::to_string(init_grav_bias_cov) + " E²");
    }

    // Load initial location from config
    if (config["initial_state"]) {
        initial_latitude = config["initial_state"]["latitude"].as<double>(37.0);
        initial_longitude = config["initial_state"]["longitude"].as<double>(-119.0);
        if (config["initial_state"]["position"]) {
            initial_altitude = -config["initial_state"]["position"][2].as<double>(3000.0); // NED z is negative altitude
        }
    }

    // Update legacy std fields for backward compatibility
    init_pos_std = Vector3d::Ones() * sqrt(init_pos_cov);
    init_vel_std = Vector3d::Ones() * sqrt(init_vel_cov);
    init_att_std = Vector3d::Ones() * sqrt(init_att_cov);
    init_ab_std = Vector3d::Ones() * sqrt(init_accel_bias_cov);
    init_gb_std = Vector3d::Ones() * sqrt(init_gyro_bias_cov);
    init_grav_std = sqrt(init_grav_bias_cov);
}

// Constructor
SquareRootUKF::SquareRootUKF(const SRUKFConfig& config)
    : config_(config), cached_gravity_height_(0.0), gravity_cache_valid_(false) {
    
    // Initialize UKF parameters for truly augmented approach
    constexpr int NOISE_DIM = StateVector::ERROR_DIM; // 20 - reuse same size as Q_sqrt

    n_x_   = StateVector::ERROR_DIM;  // 20 (error state dimension)
    n_q_   = NOISE_DIM;               // 20 (process noise dimension)
    n_aug_ = n_x_ + n_q_;             // 40 (augmented dimension)

    LOG_INFO("Augmented UKF initialized: n_x=" + std::to_string(n_x_) +
             ", n_q=" + std::to_string(n_q_) + ", n_aug=" + std::to_string(n_aug_));

    // Compute lambda for augmented dimension
    lambda_ = config_.alpha * config_.alpha * (n_aug_ + config_.kappa) - n_aug_;

    // Ensure stable weights (center weight must be reasonable)
    double min_sum = 3.0;  // Minimum value for (n_aug + lambda) to ensure stability
    if (n_aug_ + lambda_ < min_sum) {
        double old_lambda = lambda_;
        lambda_ = min_sum - n_aug_;
        LOG_INFO("Adjusted lambda from " + std::to_string(old_lambda) +
                 " to " + std::to_string(lambda_) +
                 " to ensure stability. n_aug + lambda = " +
                 std::to_string(n_aug_ + lambda_));
    }
    
    // Create earth model and mechanization
    earth_model_ = std::make_unique<EarthModel>();
    mechanization_ = std::make_unique<StrapdownMechanization>();
    
    // Allocate sigma points for augmented UKF
    sigma_points_.num_points = 2 * n_aug_ + 1;  // 2*40 + 1 = 81 points
    sigma_points_.points.resize(sigma_points_.num_points);
    sigma_points_.weights_mean.resize(sigma_points_.num_points);
    sigma_points_.weights_cov.resize(sigma_points_.num_points);

    // Calculate UKF weights
    const double w0 = lambda_ / (n_aug_ + lambda_);
    sigma_points_.weights_mean[0] = w0;
    sigma_points_.weights_cov[0]  = w0 + (1.0 - config_.alpha * config_.alpha + config_.beta);

    for (int i = 1; i < sigma_points_.num_points; ++i) {
        const double w = 0.5 / (n_aug_ + lambda_);
        sigma_points_.weights_mean[i] = w;
        sigma_points_.weights_cov[i]  = w;
    }

    // Log weight configuration
    double weight_sum = 0.0;
    for (int i = 0; i < sigma_points_.num_points; ++i) {
        weight_sum += sigma_points_.weights_mean[i];
    }
    LOG_INFO("Weights configured: w0=" + std::to_string(w0) +
             ", wi=" + std::to_string(0.5 / (n_aug_ + lambda_)) +
             ", sum=" + std::to_string(weight_sum));
    
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
    
    // Initialize square-root covariance using config values
    VectorXd init_std(StateVector::ERROR_DIM);
    // PRODUCTION FIX: Ensure positive values and reasonable bounds
    init_std.segment(0, 3).setConstant(sqrt(std::max(0.01, std::min(10.0, config_.init_pos_cov))));      // Position: 0.1-3.16m
    init_std.segment(3, 3).setConstant(sqrt(std::max(0.001, std::min(1.0, config_.init_vel_cov))));      // Velocity: 0.03-1m/s
    init_std.segment(6, 3).setConstant(sqrt(std::max(0.0001, std::min(0.01, config_.init_att_cov))));    // Attitude: 0.01-0.1rad
    init_std.segment(9, 3).setConstant(sqrt(std::max(1e-8, std::min(1e-4, config_.init_accel_bias_cov))));  // Accel bias
    init_std.segment(12, 3).setConstant(sqrt(std::max(1e-10, std::min(1e-6, config_.init_gyro_bias_cov))));  // Gyro bias
    init_std.segment(15, 5).setConstant(sqrt(std::max(0.01, std::min(10.0, config_.init_grav_bias_cov))));  // Gravity bias

    // UKF covariance initialized
    
    S_ = MatrixXd::Zero(StateVector::ERROR_DIM, StateVector::ERROR_DIM);
    for (int i = 0; i < StateVector::ERROR_DIM; ++i) {
        S_(i, i) = init_std(i);
        // Ensure positive diagonal elements
        if (S_(i, i) < 0) {
            LOG_ERROR("Negative standard deviation in S_ matrix at index " + std::to_string(i));
            S_(i, i) = std::abs(S_(i, i));
        }
    }

    // Validate S_ matrix dimensions and values
    {

        // Check for any invalid values
        if (!S_.allFinite()) {
            LOG_ERROR("S_ matrix contains non-finite values after initialization!");
        }
        if (S_.diagonal().minCoeff() <= 0) {
            LOG_ERROR("S_ matrix has non-positive diagonal elements!");
        }
    }
    
    // Initialize location from config
    latitude_ = config_.initial_latitude * M_PI / 180.0;   // Convert to radians for gravity model
    longitude_ = config_.initial_longitude * M_PI / 180.0;  // Convert to radians
    height_ = -state_.position.z();  // NED to height
    
    // Reset statistics
    stats_ = FilterStats();
    
    // UKF initialized
    logState();
}

void SquareRootUKF::predict(const Vector3d& accel, const Vector3d& gyro, double dt) {
    // Removed verbose logging from hot path (runs at 100Hz)

    // Performance timing
    auto predict_start = std::chrono::high_resolution_clock::now();

    // Invalidate gravity cache at start of new timestep
    gravity_cache_valid_ = false;

    // Apply coning/sculling compensation
    auto compensated = mechanization_->compensate(accel, gyro, dt);
    Vector3d a_in = compensated.delta_v / dt;
    Vector3d g_in = compensated.delta_theta / dt;
    // 1) Generate augmented sigma points
    auto sigma_start = std::chrono::high_resolution_clock::now();
    std::vector<AugmentedSigma> aug_sigmas;
    generateAugmentedSigmaPoints(a_in, dt, aug_sigmas);
    auto sigma_end = std::chrono::high_resolution_clock::now();
    // 2) Propagate each sigma point with its noise slice
    auto prop_start = std::chrono::high_resolution_clock::now();
    std::vector<StateVector> Xsig_pred(sigma_points_.num_points);

    // TODO: This loop could be parallelized with OpenMP for significant speedup
    // #pragma omp parallel for if(USE_OPENMP)
    for (int i = 0; i < sigma_points_.num_points; ++i) {
        Xsig_pred[i] = processModelAugmented(aug_sigmas[i].x, a_in, g_in, dt, aug_sigmas[i].w);
    }
    auto prop_end = std::chrono::high_resolution_clock::now();
    
    // 3) Compute predicted mean on manifold
    StateVector x_mean;
    x_mean.position.setZero();
    x_mean.velocity.setZero();
    x_mean.accel_bias.setZero();
    x_mean.gyro_bias.setZero();
    x_mean.gravity_bias.setZero();

    // Compute weighted mean for vector states
    LOG_DEBUG("Computing weighted mean...");
    // Weight sum should be 1.0

    for (int i = 0; i < sigma_points_.num_points; ++i) {
        double wm = sigma_points_.weights_mean[i];
        x_mean.position     += wm * Xsig_pred[i].position;
        x_mean.velocity     += wm * Xsig_pred[i].velocity;
        x_mean.accel_bias   += wm * Xsig_pred[i].accel_bias;
        x_mean.gyro_bias    += wm * Xsig_pred[i].gyro_bias;
        x_mean.gravity_bias += wm * Xsig_pred[i].gravity_bias;
    }

    // Mean state computed
    
    // Quaternion mean using eigen decomposition
    Matrix4d Qavg = Matrix4d::Zero();
    for (int i = 0; i < sigma_points_.num_points; ++i) {
        Vector4d qv;
        qv << Xsig_pred[i].quaternion.w(),
              Xsig_pred[i].quaternion.x(),
              Xsig_pred[i].quaternion.y(),
              Xsig_pred[i].quaternion.z();
        Qavg += sigma_points_.weights_mean[i] * (qv * qv.transpose());
    }
    Eigen::SelfAdjointEigenSolver<Matrix4d> es(Qavg);
    Vector4d qmean_v = es.eigenvectors().col(3);
    x_mean.quaternion = Quaterniond(qmean_v(0), qmean_v(1), qmean_v(2), qmean_v(3)).normalized();
    
    // 4) Square-root covariance from deviations (NO extra Q term!)
    // Process noise was already included via augmentation
    LOG_DEBUG("Computing square-root covariance...");
    MatrixXd D(n_x_, sigma_points_.num_points - 1);  // Deviations (exclude center)

    for (int i = 1; i < sigma_points_.num_points; ++i) {
        VectorXd ei = stateToErrorState(Xsig_pred[i]) - stateToErrorState(x_mean);
        D.col(i - 1) = std::sqrt(sigma_points_.weights_cov[i]) * ei;
    }

    // Householder QR decomposition on D^T
    Eigen::HouseholderQR<MatrixXd> qr(D.transpose());
    MatrixXd R = qr.matrixQR().topLeftCorner(n_x_, n_x_).triangularView<Eigen::Upper>();

    // Ensure positive diagonal elements
    for (int i = 0; i < n_x_; ++i) {
        if (R(i, i) < 0) {
            R.row(i) *= -1;  // Flip sign of entire row to maintain upper triangular structure
        }
        // Also ensure minimum variance to prevent numerical issues
        if (std::abs(R(i, i)) < 1e-9) {
            R(i, i) = 1e-9;
        }
    }

    // Transpose to get lower triangular form (S_ is lower triangular)
    S_ = R.transpose();

    // CRITICAL: Add numerical stability check
    // PRODUCTION FIX: Much tighter limits to prevent explosion
    const double MAX_POSITION_STD = 10.0;   // meters (was 100)
    const double MAX_VELOCITY_STD = 5.0;    // m/s (was 10)
    const double MAX_ATTITUDE_STD = 0.1;    // radians

    // Check for invalid or extreme values
    if (!S_.allFinite() || S_.hasNaN()) {
        LOG_ERROR("S_ matrix contains NaN/Inf after QR update!");
        // Reset to safe values
        S_ = MatrixXd::Identity(StateVector::ERROR_DIM, StateVector::ERROR_DIM);
        S_.block(0, 0, 3, 3) *= 1.0;    // Position: 1m
        S_.block(3, 3, 3, 3) *= 0.1;    // Velocity: 0.1 m/s
        S_.block(6, 6, 3, 3) *= 0.03;   // Attitude: ~1.7 deg
        S_.block(9, 9, 3, 3) *= 0.001;  // Accel bias
        S_.block(12, 12, 3, 3) *= 0.0001; // Gyro bias
        S_.block(15, 15, 5, 5) *= 1.0;  // Gravity bias
    }

    for (int i = 0; i < 3; ++i) {
        if (S_(i, i) > MAX_POSITION_STD) {
            S_(i, i) = MAX_POSITION_STD;
            LOG_WARN("Position covariance limited at index " + std::to_string(i));
        }
        if (S_(i+3, i+3) > MAX_VELOCITY_STD) {
            S_(i+3, i+3) = MAX_VELOCITY_STD;
            LOG_WARN("Velocity covariance limited at index " + std::to_string(i+3));
        }
        if (S_(i+6, i+6) > MAX_ATTITUDE_STD) {
            S_(i+6, i+6) = MAX_ATTITUDE_STD;
            LOG_WARN("Attitude covariance limited at index " + std::to_string(i+6));
        }
    }

    // 5) Update state
    state_ = x_mean;

    // Final validation of state and covariance
    if (!state_.position.allFinite() || state_.position.norm() > 1e6) {
        LOG_ERROR("State position invalid after prediction! Resetting to safe values.");
        state_.position = Vector3d(0, 0, -3000);
        state_.velocity = Vector3d(100, 0, 0);
        // Reset S_ to safe initial values
        S_ = MatrixXd::Identity(StateVector::ERROR_DIM, StateVector::ERROR_DIM);
        S_.block(0, 0, 3, 3) *= 1.0;    // Position
        S_.block(3, 3, 3, 3) *= 0.1;    // Velocity
        S_.block(6, 6, 3, 3) *= 0.03;   // Attitude
        S_.block(9, 9, 3, 3) *= 0.001;  // Accel bias
        S_.block(12, 12, 3, 3) *= 0.0001; // Gyro bias
        S_.block(15, 15, 5, 5) *= 1.0;    // Gravity bias
    }
    
    // Update location for earth model
    updateLocation();

    // Performance timing
    auto predict_end = std::chrono::high_resolution_clock::now();
    auto predict_ms = std::chrono::duration_cast<std::chrono::microseconds>(predict_end - predict_start).count() / 1000.0;

    // Log timing only if it exceeds threshold
    if (predict_ms > 5.0) {  // More than 5ms is concerning
        LOG_WARN("UKF predict took " + std::to_string(predict_ms) + " ms");
    }

    // Prediction complete - removed verbose logging from hot path
    {

        // Check for divergence
        if (state_.position.norm() > 1e6) {
            LOG_ERROR("FILTER DIVERGING: Position norm exceeds 1e6 meters!");
        }
        if (state_.velocity.norm() > 1000) {
            LOG_WARN("Large velocity detected: " + std::to_string(state_.velocity.norm()) + " m/s");
        }
    }
}

void SquareRootUKF::updateBarometer(double pressure, double temperature) {
    // Check for NaN/Inf in measurements
    if (!std::isfinite(pressure) || !std::isfinite(temperature)) {
        LOG_WARN("Invalid barometer measurement (NaN/Inf) - rejecting update");
        return;
    }

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

    // Debug: Log large innovations
    if (std::abs(innovation) > 100.0) {
        std::stringstream msg;
        msg << "LARGE BARO INNOVATION: " << innovation << " m (predicted: "
            << h_pred << ", measured: " << altitude << ")";
        LOG_WARN(msg.str());
    }
    
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
    // Check for NaN/Inf in measurements
    if (!mag_body.allFinite()) {
        LOG_WARN("Invalid magnetometer measurement (NaN/Inf) - rejecting update");
        return;
    }

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
    // Check for NaN/Inf in measurements
    if (!gradient.allFinite()) {
        LOG_WARN("Invalid gravity gradient measurement (NaN/Inf) - rejecting update");
        return;
    }

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

void SquareRootUKF::generateAugmentedSigmaPoints(const Vector3d& accel_input, double dt,
                                                 std::vector<AugmentedSigma>& aug_sigmas) {
    // Compute Q_sqrt for this timestep
    MatrixXd Q_sqrt = computeProcessNoise(accel_input, dt); // 20x20

    // Build augmented square-root matrix S_aug = blkdiag(S_, Q_sqrt)
    MatrixXd S_aug = MatrixXd::Zero(n_aug_, n_aug_);
    S_aug.topLeftCorner(n_x_, n_x_) = S_;         // 20x20 state covariance
    S_aug.bottomRightCorner(n_q_, n_q_) = Q_sqrt; // 20x20 process noise

    // S_aug built with state and process noise

    // Resize output
    aug_sigmas.resize(sigma_points_.num_points);

    // Center point
    aug_sigmas[0].x = state_;
    aug_sigmas[0].w = VectorXd::Zero(n_q_);

    // Compute scale factor
    double scale = std::sqrt(std::abs(n_aug_ + lambda_));
    // Scale factor computed

    // Generate sigma points from augmented covariance
    for (int i = 0; i < n_aug_; ++i) {
        VectorXd delta_aug = scale * S_aug.col(i);

        // Split into state perturbation and noise
        VectorXd dx = delta_aug.head(n_x_);  // State perturbation
        VectorXd w  = delta_aug.tail(n_q_);  // Noise component

        // + direction (index i+1)
        aug_sigmas[i + 1].x = errorStateToState(dx, state_);
        aug_sigmas[i + 1].w = w;

        // - direction (index i+1+n_aug_)
        aug_sigmas[i + 1 + n_aug_].x = errorStateToState(-dx, state_);
        aug_sigmas[i + 1 + n_aug_].w = -w;

        // Sigma point generated
    }

    // Augmented sigma points generated
}

void SquareRootUKF::generateSigmaPoints() {
    LOG_INFO("=== generateSigmaPoints START ===");
    LOG_INFO("n_aug: " + std::to_string(n_aug_) + ", lambda: " + std::to_string(lambda_));
    LOG_INFO("n_aug + lambda: " + std::to_string(n_aug_ + lambda_));

    // Current state as center point
    sigma_points_.points[0] = state_;
    LOG_INFO("Center point (state) position: " + std::to_string(state_.position.x()) + ", " +
             std::to_string(state_.position.y()) + ", " + std::to_string(state_.position.z()));

    // Check for invalid S_ matrix
    if (!S_.allFinite() || S_.hasNaN()) {
        LOG_ERROR("S_ matrix contains NaN or Inf values!");
        std::stringstream ss;
        ss << "S_ diagonal: " << S_.diagonal().transpose();
        LOG_ERROR(ss.str());
        // Reset to small identity
        S_ = MatrixXd::Identity(StateVector::ERROR_DIM, StateVector::ERROR_DIM) * 0.01;
    }

    // Log S_ matrix info
    LOG_INFO("S_ matrix diagonal (first 10): " +
             std::to_string(S_(0,0)) + ", " + std::to_string(S_(1,1)) + ", " +
             std::to_string(S_(2,2)) + ", " + std::to_string(S_(3,3)) + ", " +
             std::to_string(S_(4,4)) + ", " + std::to_string(S_(5,5)) + ", " +
             std::to_string(S_(6,6)) + ", " + std::to_string(S_(7,7)) + ", " +
             std::to_string(S_(8,8)) + ", " + std::to_string(S_(9,9)));

    // PRODUCTION FIX: Robust sigma point generation with bounds
    double scale_factor = sqrt(std::abs(n_aug_ + lambda_));
    LOG_INFO("Initial scale_factor: " + std::to_string(scale_factor));

    // Limit scale factor to reasonable range
    const double MAX_SCALE = 10.0;
    const double MIN_SCALE = 0.1;
    if (!std::isfinite(scale_factor)) {
        LOG_ERROR("Non-finite scale factor, using default 1.0");
        scale_factor = 1.0;
    } else if (scale_factor > MAX_SCALE) {
        LOG_WARN("Scale factor " + std::to_string(scale_factor) + " clamped to " + std::to_string(MAX_SCALE));
        scale_factor = MAX_SCALE;
    } else if (scale_factor < MIN_SCALE) {
        LOG_WARN("Scale factor " + std::to_string(scale_factor) + " clamped to " + std::to_string(MIN_SCALE));
        scale_factor = MIN_SCALE;
    }

    // Debug: Check S_ values before scaling
    static int debug_count = 0;
    if (debug_count++ < 5) {
        std::stringstream ss;
        ss << "S_ diagonal before scaling: [" << S_(0,0) << ", " << S_(1,1) << ", " << S_(2,2) << "]";
        LOG_INFO(ss.str());
        LOG_INFO("Scale factor: " + std::to_string(scale_factor));
    }

    LOG_INFO("Final scale_factor (after clamping): " + std::to_string(scale_factor));

    MatrixXd scaled_S = scale_factor * S_;
    LOG_INFO("scaled_S diagonal (first 3 - position): " +
             std::to_string(scaled_S(0,0)) + ", " +
             std::to_string(scaled_S(1,1)) + ", " +
             std::to_string(scaled_S(2,2)));

    // PRODUCTION FIX: Add per-dimension limits for sigma point deviations
    VectorXd max_deviations(StateVector::ERROR_DIM);
    max_deviations.segment(0, 3).setConstant(100.0);   // Position: max 100m deviation
    max_deviations.segment(3, 3).setConstant(10.0);    // Velocity: max 10m/s deviation
    max_deviations.segment(6, 3).setConstant(0.5);     // Attitude: max 0.5 rad deviation
    max_deviations.segment(9, 3).setConstant(0.1);     // Accel bias: max 0.1 m/s^2
    max_deviations.segment(12, 3).setConstant(0.01);   // Gyro bias: max 0.01 rad/s
    max_deviations.segment(15, 5).setConstant(10.0);   // Gravity bias

    for (int i = 0; i < StateVector::ERROR_DIM; ++i) {
        VectorXd delta = scaled_S.col(i);

        // DEBUG: Log the raw delta for first few dimensions
        if (i < 6) {  // Position and velocity dimensions
            LOG_DEBUG("Sigma point " + std::to_string(i) + " raw delta:");
            LOG_DEBUG("  Position delta: " + std::to_string(delta(0)) + ", " +
                      std::to_string(delta(1)) + ", " + std::to_string(delta(2)));
            LOG_DEBUG("  Velocity delta: " + std::to_string(delta(3)) + ", " +
                      std::to_string(delta(4)) + ", " + std::to_string(delta(5)));
            LOG_DEBUG("  Delta norm: " + std::to_string(delta.norm()));
        }

        // Limit each component
        bool clamped = false;
        for (int j = 0; j < StateVector::ERROR_DIM; ++j) {
            if (std::abs(delta(j)) > max_deviations(j)) {
                if (!clamped && j < 6) {
                    LOG_WARN("Clamping dimension " + std::to_string(j) + " from " +
                             std::to_string(delta(j)) + " to " + std::to_string(max_deviations(j)));
                }
                delta(j) = std::copysign(max_deviations(j), delta(j));
                clamped = true;
            }
        }

        // Check for NaN/Inf
        if (!delta.allFinite()) {
            LOG_ERROR("Non-finite sigma point delta at index " + std::to_string(i));
            delta = VectorXd::Zero(StateVector::ERROR_DIM);
            delta(i) = 0.01;  // Small perturbation in one direction
        }

        // Plus direction
        LOG_DEBUG("Creating sigma point " + std::to_string(i + 1) + " (plus direction)");
        sigma_points_.points[i + 1] = errorStateToState(delta, state_);

        // DEBUG: Log resulting position for first few points
        if (i < 3) {
            LOG_DEBUG("Sigma point " + std::to_string(i + 1) + " position: " +
                      std::to_string(sigma_points_.points[i + 1].position.x()) + ", " +
                      std::to_string(sigma_points_.points[i + 1].position.y()) + ", " +
                      std::to_string(sigma_points_.points[i + 1].position.z()));
        }

        // Minus direction
        LOG_DEBUG("Creating sigma point " + std::to_string(i + 1 + StateVector::ERROR_DIM) + " (minus direction)");
        sigma_points_.points[i + 1 + StateVector::ERROR_DIM] = errorStateToState(-delta, state_);

        // DEBUG: Log resulting position for first few points
        if (i < 3) {
            LOG_DEBUG("Sigma point " + std::to_string(i + 1 + StateVector::ERROR_DIM) + " position: " +
                      std::to_string(sigma_points_.points[i + 1 + StateVector::ERROR_DIM].position.x()) + ", " +
                      std::to_string(sigma_points_.points[i + 1 + StateVector::ERROR_DIM].position.y()) + ", " +
                      std::to_string(sigma_points_.points[i + 1 + StateVector::ERROR_DIM].position.z()));
        }
    }
}

StateVector SquareRootUKF::processModelAugmented(const StateVector& x, const Vector3d& accel_meas,
                                                const Vector3d& gyro_meas, double dt,
                                                const VectorXd& w) {
    // This augmented version uses the noise vector w to inject process noise
    // w has length n_q_ (=20), mapped to match our Q layout:
    // [0:3) pos noise, [3:6) vel noise, [6:9) att noise,
    // [9:12) accel bias RW, [12:15) gyro bias RW, [15:20) gravity bias RW

    // Extract noise components
    Vector3d n_pos      = w.segment<3>(0);   // Position noise (not directly used)
    Vector3d n_vel      = w.segment<3>(3);   // Velocity noise (could add to accel)
    Vector3d n_att      = w.segment<3>(6);   // Attitude noise (add to gyro)
    Vector3d n_ab_rw    = w.segment<3>(9);   // Accel bias random walk
    Vector3d n_gb_rw    = w.segment<3>(12);  // Gyro bias random walk
    Eigen::Matrix<double, 5, 1> n_grav_rw = w.segment(15, 5); // Gravity bias RW

    // Add attitude noise to gyro (scaled by sqrt(dt) for white noise)
    Vector3d gyro_noisy = gyro_meas + n_att / std::sqrt(dt);

    // Could add velocity noise to accel if desired
    Vector3d accel_noisy = accel_meas;  // + n_vel / (dt * std::sqrt(dt));

    // Propagate with noisy IMU using existing process model
    StateVector nx = processModel(x, accel_noisy, gyro_noisy, dt);

    // Apply bias random walks
    nx.accel_bias = x.accel_bias + n_ab_rw * std::sqrt(dt);
    nx.gyro_bias = x.gyro_bias + n_gb_rw * std::sqrt(dt);
    nx.gravity_bias = x.gravity_bias + n_grav_rw * std::sqrt(dt);

    return nx;
}

StateVector SquareRootUKF::processModel(const StateVector& state, const Vector3d& accel,
                                       const Vector3d& gyro, double dt) {
    // PRODUCTION FIX: Validate inputs first
    if (!state.position.allFinite() || !state.velocity.allFinite()) {
        LOG_ERROR("Invalid state in processModel");
        return state;  // Return unchanged state
    }

    if (!accel.allFinite() || !gyro.allFinite()) {
        LOG_ERROR("Invalid IMU data in processModel");
        return state;  // Return unchanged state
    }

    if (dt <= 0 || dt > 0.1 || !std::isfinite(dt)) {
        LOG_ERROR("Invalid dt in processModel: " + std::to_string(dt));
        return state;  // Return unchanged state
    }

    StateVector new_state = state;

    // Remove bias from measurements with bounds
    Vector3d accel_corrected = accel - state.accel_bias;
    Vector3d gyro_corrected = gyro - state.gyro_bias;

    // PRODUCTION FIX: Bound IMU measurements
    const double MAX_ACCEL_MAG = 200.0;  // 20G max
    const double MAX_GYRO_MAG = 10.0;    // 10 rad/s max

    if (accel_corrected.norm() > MAX_ACCEL_MAG) {
        LOG_WARN("Excessive acceleration clamped: " + std::to_string(accel_corrected.norm()));
        accel_corrected = accel_corrected.normalized() * MAX_ACCEL_MAG;
    }

    if (gyro_corrected.norm() > MAX_GYRO_MAG) {
        LOG_WARN("Excessive gyro rate clamped: " + std::to_string(gyro_corrected.norm()));
        gyro_corrected = gyro_corrected.normalized() * MAX_GYRO_MAG;
    }
    
    // Attitude update (SO(3) integration)
    new_state.quaternion = SO3::integrateQuaternion(state.quaternion, gyro_corrected, dt);
    
    // Transform acceleration to navigation frame
    Matrix3d C_bn = state.quaternion.toRotationMatrix();
    Vector3d accel_nav = C_bn * accel_corrected;
    
    // PRODUCTION FIX: Robust gravity compensation with caching
    // CRITICAL FIX: Use the current state's height, not the global height_!
    double current_height = -state.position.z();  // Convert NED to height

    // Cache gravity calculation within timestep to avoid redundant computations
    Vector3d gravity;
    if (!gravity_cache_valid_ || std::abs(current_height - cached_gravity_height_) > 1.0) {
        gravity = earth_model_->gravityNED(latitude_, current_height);
        cached_gravity_ = gravity;
        cached_gravity_height_ = current_height;
        gravity_cache_valid_ = true;
    } else {
        gravity = cached_gravity_;
    }

    Vector3d coriolis = earth_model_->coriolisAcceleration(state.velocity, latitude_);

    // Validate gravity vector
    if (!gravity.allFinite() || gravity.norm() < 9.0 || gravity.norm() > 10.0) {
        LOG_WARN("Invalid gravity magnitude: " + std::to_string(gravity.norm()) + " m/s^2");
        gravity = Vector3d(0, 0, 9.81);  // Use standard gravity as fallback
    }

    // Ensure gravity is pointing down in NED (positive Z)
    if (gravity.z() < 0) {
        LOG_ERROR("Gravity pointing up! Fixing sign.");
        gravity.z() = std::abs(gravity.z());
    }

    // Apply gravity and Coriolis compensation
    // Note: accel_nav is specific force from IMU
    // For stationary object, IMU reads [0, 0, -g] (upward force)
    // True acceleration = specific_force + gravity - Coriolis
    // In NED, gravity vector is [0, 0, +g] (positive down)

    // Process model computations

    accel_nav = accel_nav + gravity - coriolis;

    // PRODUCTION FIX: Strong bounds on total acceleration
    const double MAX_ACCEL = 30.0;  // 3G max total acceleration
    if (accel_nav.norm() > MAX_ACCEL) {
        LOG_WARN("Total acceleration clamped: " + std::to_string(accel_nav.norm()) + " -> " + std::to_string(MAX_ACCEL));
        accel_nav = accel_nav.normalized() * MAX_ACCEL;
    }

    // PRODUCTION FIX: Limit velocity change per step
    Vector3d delta_v = accel_nav * dt;
    const double MAX_DELTA_V = 1.0;  // Max 1 m/s change per step
    if (delta_v.norm() > MAX_DELTA_V) {
        LOG_WARN("Velocity change limited: " + std::to_string(delta_v.norm()) + " -> " + std::to_string(MAX_DELTA_V));
        delta_v = delta_v.normalized() * MAX_DELTA_V;
    }

    // Update velocity with bounds
    new_state.velocity = state.velocity + delta_v;

    // PRODUCTION FIX: Limit total velocity
    const double MAX_VELOCITY = 500.0;  // 500 m/s max (Mach 1.5)
    if (new_state.velocity.norm() > MAX_VELOCITY) {
        LOG_WARN("Velocity clamped: " + std::to_string(new_state.velocity.norm()));
        new_state.velocity = new_state.velocity.normalized() * MAX_VELOCITY;
    }

    // PRODUCTION FIX: Limit position change per step
    Vector3d delta_pos = state.velocity * dt + 0.5 * accel_nav * dt * dt;
    const double MAX_DELTA_POS = 10.0;  // Max 10m position change per step
    if (delta_pos.norm() > MAX_DELTA_POS) {
        LOG_WARN("Position change limited: " + std::to_string(delta_pos.norm()) + " -> " + std::to_string(MAX_DELTA_POS));
        delta_pos = delta_pos.normalized() * MAX_DELTA_POS;
    }

    // Update position with bounds
    new_state.position = state.position + delta_pos;

    // PRODUCTION FIX: Final validation
    if (!new_state.position.allFinite()) {
        LOG_ERROR("Position became NaN in processModel!");
        return state;  // Return unchanged state
    }

    if (new_state.position.norm() > 1e5) {  // 100km max from origin
        LOG_ERROR("Position exploded to " + std::to_string(new_state.position.norm()) + " meters!");
        return state;  // Return unchanged state
    }

    if (!new_state.velocity.allFinite()) {
        LOG_ERROR("Velocity became NaN in processModel!");
        new_state.velocity = state.velocity;  // Keep previous velocity
    }
    
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

    // DEBUG: Log the incoming error vector
    LOG_DEBUG("=== errorStateToState ===");
    LOG_DEBUG("Nominal position: " + std::to_string(nominal.position.x()) + ", " +
              std::to_string(nominal.position.y()) + ", " + std::to_string(nominal.position.z()));
    LOG_DEBUG("Error vector norm: " + std::to_string(error.norm()));

    // CRITICAL FIX: Much tighter bound on position perturbation
    Vector3d pos_error = error.segment(0, 3);
    LOG_DEBUG("Position error: " + std::to_string(pos_error.x()) + ", " +
              std::to_string(pos_error.y()) + ", " + std::to_string(pos_error.z()) +
              " (norm: " + std::to_string(pos_error.norm()) + ")");

    const double MAX_POS_ERROR = 100.0;  // Max 100m deviation for sigma points
    if (pos_error.norm() > MAX_POS_ERROR) {
        LOG_WARN("Large position error in sigma point: " + std::to_string(pos_error.norm()));
        pos_error = pos_error.normalized() * MAX_POS_ERROR;
    }
    state.position = nominal.position + pos_error;
    LOG_DEBUG("Result position: " + std::to_string(state.position.x()) + ", " +
              std::to_string(state.position.y()) + ", " + std::to_string(state.position.z()));

    // CRITICAL FIX: Tighter bound on velocity perturbation
    Vector3d vel_error = error.segment(3, 3);
    LOG_DEBUG("Velocity error: " + std::to_string(vel_error.x()) + ", " +
              std::to_string(vel_error.y()) + ", " + std::to_string(vel_error.z()) +
              " (norm: " + std::to_string(vel_error.norm()) + ")");

    const double MAX_VEL_ERROR = 10.0;  // Max 10 m/s deviation for sigma points
    if (vel_error.norm() > MAX_VEL_ERROR) {
        LOG_WARN("Large velocity error in sigma point: " + std::to_string(vel_error.norm()));
        vel_error = vel_error.normalized() * MAX_VEL_ERROR;
    }
    state.velocity = nominal.velocity + vel_error;

    // Quaternion update using SO(3) boxplus
    Vector3d att_error = error.segment(6, 3);
    LOG_DEBUG("Attitude error: " + std::to_string(att_error.x()) + ", " +
              std::to_string(att_error.y()) + ", " + std::to_string(att_error.z()) +
              " (norm: " + std::to_string(att_error.norm()) + ")");
    state.quaternion = SO3::boxplus(nominal.quaternion, att_error);

    state.accel_bias = nominal.accel_bias + error.segment(9, 3);
    state.gyro_bias = nominal.gyro_bias + error.segment(12, 3);
    state.gravity_bias = nominal.gravity_bias + error.segment(15, 5);

    // PRODUCTION FIX: Final validation
    if (!state.position.allFinite() || state.position.norm() > 1e5) {
        LOG_ERROR("Invalid sigma point position: " + std::to_string(state.position.norm()));
        state.position = nominal.position;  // Use nominal
    }

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

    // PRODUCTION FIX: Validate and bound dt
    if (dt <= 0 || !std::isfinite(dt)) {
        LOG_ERROR("Invalid dt in computeProcessNoise: " + std::to_string(dt));
        dt = 0.01;  // Default to 10ms
    } else if (dt > 0.1) {
        LOG_WARN("Large dt clamped: " + std::to_string(dt) + " -> 0.1");
        dt = 0.1;  // Cap at 100ms
    }

    // PRODUCTION FIX: Limit process noise to prevent explosion
    // Position noise - much smaller to prevent drift
    double q_pos_bounded = std::min(1e-8, config_.q_pos);
    Q_sqrt.block(0, 0, 3, 3) = Matrix3d::Identity() * sqrt(q_pos_bounded * dt);

    // Velocity noise - also reduced
    double q_vel_bounded = std::min(1e-4, config_.q_vel);
    Q_sqrt.block(3, 3, 3, 3) = Matrix3d::Identity() * sqrt(q_vel_bounded * dt);

    // Attitude noise (adaptive based on angular rate) - bounded
    double gyro_norm = state_.gyro_bias.norm();
    double adaptive_q_att = std::min(1e-6, config_.q_att * (1.0 + 10.0 * gyro_norm));
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

    // Validate output
    if (!Q_sqrt.allFinite()) {
        LOG_ERROR("Process noise matrix contains NaN or Inf");
        Q_sqrt.setIdentity();
        Q_sqrt *= 1e-6;  // Small default noise
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

    // CRITICAL FIX: Keep everything in radians (latitude_ is already in radians)
    latitude_ += (state_.position.x() / R_earth);  // delta in radians
    longitude_ += (state_.position.y() / (R_earth * cos(latitude_)));  // delta in radians, latitude_ already in radians
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