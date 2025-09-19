#include "ukf.h"
#include "ukf_sigma_points.h"
#include "ukf_measurements.h"
#include "ukf_math_utils.h"
#include "gravity_gradient_provider.h"
#include <cmath>
#include <iostream>
#include <memory>

UKF::UKF(const UKFConfig& cfg) : cfg_(cfg) {
    // Validate configuration
    if (!cfg_.validate()) {
        std::cerr << "Invalid UKF configuration, using defaults\n";
        cfg_.setDefaults();
    }
    
    computeWeights();
    
    // Initialize integrity monitoring
    integrity_stats_.nees = 0.0;
    integrity_stats_.nis = 0.0;
    integrity_stats_.nees_pass_rate = 1.0;
    integrity_stats_.nis_pass_rate = 1.0;
    
    // Initialize sigma points manager (gravity provider will be set later)
    sigma_points_manager_ = std::make_unique<UKFSigmaPoints>(*this, nullptr);
    // Defer measurements manager until gravity provider is set
    
    std::cout << "UKF initialized with modular architecture\n";
}

UKF::~UKF() {
    // unique_ptr automatically handles cleanup
}

void UKF::computeWeights() {
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

void UKF::init(const State& x0_ECEF, const Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& P0, bool use_enu) {
    use_enu_ = use_enu;

    if (use_enu_) {
        // Initialize ENU frame at initial position
        initializeEnu(x0_ECEF.p_ECEF);

        // Convert initial state to ENU
        nominal_state_ = ecefToEnu(x0_ECEF);

        // In ENU mode, we receive P0 in ECEF mindset but need to adapt it for ENU scale
        // Key insight: In ENU, position starts at [0,0,0], so we need much smaller position variance
        P_ = P0;

        // Critical: Balance covariance to achieve good condition number
        // Goal: Keep ratio of largest to smallest eigenvalue < 1e6

        // Position: 1 m² (1m uncertainty in local frame)
        P_.block<3,3>(POS_IDX, POS_IDX) = Eigen::Matrix3d::Identity() * 1.0;

        // Velocity: 0.1 m²/s² (0.316 m/s uncertainty)
        P_.block<3,3>(VEL_IDX, VEL_IDX) = Eigen::Matrix3d::Identity() * 0.1;

        // Attitude: 0.01 rad² (0.1 rad uncertainty)
        P_.block<3,3>(ATT_IDX, ATT_IDX) = Eigen::Matrix3d::Identity() * 0.01;

        // Accel bias: 0.001 m²/s⁴
        P_.block<3,3>(BA_IDX, BA_IDX) = Eigen::Matrix3d::Identity() * 0.001;

        // Gyro bias: 0.0001 rad²/s² - balanced with position
        P_.block<3,3>(BG_IDX, BG_IDX) = Eigen::Matrix3d::Identity() * 0.0001;

        std::cout << "ENU mode: Covariance scaled for local frame operation\n";

        // Debug: Check condition number after scaling
        double cond = UKFMathUtils::computeConditionNumber(P_);
        std::cout << "  Condition number after ENU scaling: " << cond << "\n";
    } else {
        nominal_state_ = x0_ECEF;
        P_ = P0;
    }
    UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P_);
}

void UKF::setGravityProvider(GravityGradientProvider* provider) {
    gravity_provider_ = provider;

    // Update sigma points manager with gravity provider for physics-correct propagation
    if (sigma_points_manager_) {
        sigma_points_manager_ = std::make_unique<UKFSigmaPoints>(*this, gravity_provider_);
    }

    // Create measurements manager now that we have the provider
    measurements_manager_ = std::make_unique<UKFMeasurements>(*this, gravity_provider_);
}

void UKF::predict(const ImuSample& imu, double dt) {
    // First ensure current covariance is valid
    if (!UKFMathUtils::checkMatrixValidity<ERROR_STATE_DIM>(P_)) {
        std::cerr << "WARNING: Invalid covariance before predict, fixing...\n";
        UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P_, cfg_.numerical.min_eigenvalue);
    }

    // Check if we need to re-anchor in ENU mode
    if (use_enu_) {
        checkAndReanchor();
    }

    // Generate sigma points using modular manager
    sigma_points_manager_->generate(nominal_state_, P_, lambda_);

    // Propagate sigma points through motion model
    auto propagated_states = sigma_points_manager_->propagateStates(imu, dt);

    // Compute predicted mean state
    nominal_state_ = sigma_points_manager_->computeMeanState(propagated_states, weights_mean_);

    // Compute predicted covariance
    P_ = sigma_points_manager_->computeCovariance(propagated_states, nominal_state_, weights_cov_);

    // Add process noise
    addProcessNoise(dt);

    // Add covariance floor to prevent overconfidence
    const double min_variance = 1e-12;  // Minimum variance on diagonal
    for (int i = 0; i < ERROR_STATE_DIM; i++) {
        if (P_(i,i) < min_variance) {
            P_(i,i) = min_variance;
        }
    }

    // Final check and fix
    if (!UKFMathUtils::checkMatrixValidity<ERROR_STATE_DIM>(P_)) {
        std::cerr << "WARNING: Invalid covariance after predict, fixing...\n";
        UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P_, cfg_.numerical.min_eigenvalue);
    }
}

void UKF::addProcessNoise(double dt) {
    // Add process noise to covariance
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> Q =
        Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Zero();

    // Adaptive noise scaling based on current uncertainty
    double pos_uncertainty = std::sqrt(P_.block<3,3>(POS_IDX, POS_IDX).trace() / 3.0);
    double vel_uncertainty = std::sqrt(P_.block<3,3>(VEL_IDX, VEL_IDX).trace() / 3.0);

    // Scale process noise adaptively - increase when uncertainty is too low
    double pos_scale = (pos_uncertainty < 10.0) ? 2.0 : 1.0;  // Boost if too certain
    double vel_scale = (vel_uncertainty < 1.0) ? 2.0 : 1.0;

    Q.block<3,3>(POS_IDX, POS_IDX) = Eigen::Matrix3d::Identity() *
        cfg_.process_noise.position * cfg_.process_noise.position * dt * dt * pos_scale;
    Q.block<3,3>(VEL_IDX, VEL_IDX) = Eigen::Matrix3d::Identity() *
        cfg_.process_noise.velocity * cfg_.process_noise.velocity * dt * vel_scale;
    Q.block<3,3>(ATT_IDX, ATT_IDX) = Eigen::Matrix3d::Identity() *
        cfg_.process_noise.attitude * cfg_.process_noise.attitude * dt;
    Q.block<3,3>(BA_IDX, BA_IDX) = Eigen::Matrix3d::Identity() *
        cfg_.process_noise.accel_bias * cfg_.process_noise.accel_bias * dt;
    Q.block<3,3>(BG_IDX, BG_IDX) = Eigen::Matrix3d::Identity() *
        cfg_.process_noise.gyro_bias * cfg_.process_noise.gyro_bias * dt;

    P_ += Q;

    // Ensure minimum uncertainty to prevent collapse
    double min_pos_var = 1.0;  // 1 m^2 minimum
    double min_vel_var = 0.01; // 0.1 m/s minimum
    for (int i = 0; i < 3; i++) {
        P_(POS_IDX + i, POS_IDX + i) = std::max(P_(POS_IDX + i, POS_IDX + i), min_pos_var);
        P_(VEL_IDX + i, VEL_IDX + i) = std::max(P_(VEL_IDX + i, VEL_IDX + i), min_vel_var);
    }

    // Enforce positive definiteness
    UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P_, cfg_.numerical.min_eigenvalue);
}

void UKF::enforcePositiveDefinite(Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& P) {
    // Use the robust implementation from math utils
    UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P, cfg_.numerical.min_eigenvalue);
}

void UKF::updateGradient(const Eigen::Matrix3d& measured, const Eigen::Matrix3d& R) {
    // Use the modular measurements manager for gradient updates
    assert(measurements_manager_ && "setGravityProvider() must be called before gradient updates");
    measurements_manager_->updateGravityGradient(measured, R);
}

void UKF::updateAnomaly(double measured_anomaly_mgal, double noise_mgal) {
    assert(measurements_manager_ && "setGravityProvider() must be called before anomaly updates");
    assert(gravity_provider_ && "Gravity provider must be set");

    Eigen::Matrix<double, 1, 1> measurement;
    measurement << measured_anomaly_mgal;

    Eigen::Matrix<double, 1, 1> R;
    R << noise_mgal * noise_mgal;

    // Correct measurement model for gravity anomaly using the gravity provider
    auto anomaly_model = [this](const State& state) -> Eigen::Matrix<double, 1, 1> {
        Eigen::Matrix<double, 1, 1> predicted_anomaly;

        // Get the gravity anomaly from the provider (returns Eötvös = 10^-9 s^-2)
        // Note: 1 Eötvös = 0.1 mGal (since 1 mGal = 10^-5 m/s^2 = 10^-8 s^-2)
        double anomaly_eotvos = gravity_provider_->getAnomaly(state.p_ECEF);

        // Convert from Eötvös to mGal for consistency with measurement
        predicted_anomaly << anomaly_eotvos * 0.1;

        return predicted_anomaly;
    };

    measurements_manager_->updateMeasurement<1>(measurement, R, anomaly_model);
}

void UKF::updateMagnetometer(const Eigen::Vector3d& mag_body, 
                           const Eigen::Vector3d& mag_ref_ECEF,
                           const Eigen::Matrix3d& R_mag) {
    // Use the modular measurements manager for magnetometer updates
    measurements_manager_->updateMagnetometer(mag_body, mag_ref_ECEF, R_mag);
}

void UKF::updateGravityMapMatch(const Eigen::Vector3d& matched_position_ECEF,
                               const Eigen::Matrix3d& R_position) {
    // Use the modular measurements manager for map matching updates
    measurements_manager_->updateGravityMapMatch(matched_position_ECEF, R_position);
}

void UKF::updateBarometer(double pressure_altitude, double noise) {
    // Use the modular measurements manager for barometer updates
    measurements_manager_->updateBarometer(pressure_altitude, noise);
}

// Production-ready implementations for additional measurement methods
void UKF::updateGradientInvariants(const Eigen::Matrix3d& measured_tensor, const Eigen::Matrix2d& R_invariants) {
    // Compute rotationally-invariant features of the gravity gradient tensor
    // These are independent of attitude, making them much more robust

    // Measurement model that computes expected invariants from gravity model
    auto invariants_model = [this](const State& state) -> Eigen::Vector2d {
        // Get predicted gravity gradient tensor at this position
        // This uses the static gravity provider in UKFMeasurements
        Eigen::Matrix3d predicted_tensor = measurements_manager_->predictGravityGradient(state);

        // Compute invariants of predicted tensor using J2 and J3
        const double J2_pred = 0.5 * (predicted_tensor * predicted_tensor).trace();
        const double J3_pred = predicted_tensor.determinant();

        Eigen::Vector2d expected_invariants;
        expected_invariants << J2_pred, J3_pred;
        return expected_invariants;
    };

    // Compute invariants of measured tensor using J2 and J3
    const double J2_meas = 0.5 * (measured_tensor * measured_tensor).trace();
    const double J3_meas = measured_tensor.determinant();

    Eigen::Vector2d measured_invariants;
    measured_invariants << J2_meas, J3_meas;

    // Update using only these 2D invariants instead of full 9D tensor
    assert(measurements_manager_ && "setGravityProvider() must be called before gradient updates");
    measurements_manager_->updateMeasurement<2>(measured_invariants, R_invariants, invariants_model);
}

void UKF::updateDopplerVelocity(const Eigen::Vector3d& measured_velocity_body, const Eigen::Matrix3d& R_doppler) {
    // Production Doppler velocity update
    auto doppler_model = [](const State& state) -> Eigen::Vector3d {
        // Transform ECEF velocity to body frame
        return state.q_ECEF_B.inverse() * state.v_ECEF;
    };
    
    measurements_manager_->updateMeasurement<3>(measured_velocity_body, R_doppler, doppler_model);
}

void UKF::updateZUPT(const Eigen::Matrix3d& R_vel) {
    // Zero velocity update - constrains velocity when stationary
    Eigen::Vector3d zero_velocity = Eigen::Vector3d::Zero();
    
    auto zupt_model = [](const State& state) -> Eigen::Vector3d {
        return state.v_ECEF;
    };
    
    measurements_manager_->updateMeasurement<3>(zero_velocity, R_vel, zupt_model);
}

void UKF::updateTerrainAltitude(double radar_alt, double terrain_height, double noise) {
    // Terrain-referenced altitude from radar altimeter
    double measured_altitude = radar_alt + terrain_height;
    Eigen::Matrix<double, 1, 1> measurement;
    measurement << measured_altitude;
    
    Eigen::Matrix<double, 1, 1> R;
    R << noise * noise;
    
    auto altitude_model = [](const State& state) -> Eigen::Matrix<double, 1, 1> {
        Eigen::Vector3d lla = UKFMathUtils::ecefToLla(state.p_ECEF);
        Eigen::Matrix<double, 1, 1> result;
        result << lla(2);  // altitude
        return result;
    };
    
    measurements_manager_->updateMeasurement<1>(measurement, R, altitude_model);
}

// REMOVED: Pseudo-measurements (updateZeroVerticalSpeed, updateZeroSideslip, updateVirtualDoppler)
// These were not used in production and have been removed to clean up the codebase

// updateScalarPseudo also removed - not used

// ============================================================================
// ENU Frame Methods for Numerical Stability
// ============================================================================

void UKF::initializeEnu(const Eigen::Vector3d& anchor_ECEF) {
    enu_anchor_ECEF_ = anchor_ECEF;

    // Compute ENU to ECEF rotation at anchor
    Eigen::Vector3d lla = UKFMathUtils::ecefToLla(anchor_ECEF);
    double lat = lla(0);
    double lon = lla(1);

    double sin_lat = std::sin(lat);
    double cos_lat = std::cos(lat);
    double sin_lon = std::sin(lon);
    double cos_lon = std::cos(lon);

    // Rotation matrix from ENU to ECEF
    R_ECEF_ENU_ << -sin_lon, -sin_lat*cos_lon, cos_lat*cos_lon,
                    cos_lon, -sin_lat*sin_lon, cos_lat*sin_lon,
                    0,        cos_lat,          sin_lat;

    // Debug rotation matrix
    if (false) {  // Set to true to enable debug
        std::cout << "[DEBUG] ENU initialization at lat=" << lat*180/M_PI
                  << "°, lon=" << lon*180/M_PI << "°\n";
        std::cout << "R_ECEF_ENU:\n" << R_ECEF_ENU_ << "\n";
    }
}

State UKF::ecefToEnu(const State& state_ECEF) const {
    State state_ENU;

    // Position: convert to ENU relative to anchor
    Eigen::Vector3d delta_ECEF = state_ECEF.p_ECEF - enu_anchor_ECEF_;
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

State UKF::enuToEcef(const State& state_ENU) const {
    State state_ECEF;

    // Position: convert from ENU to ECEF
    Eigen::Vector3d delta_ENU = state_ENU.p_ECEF;  // Actually p_ENU

    // Debug output disabled

    state_ECEF.p_ECEF = enu_anchor_ECEF_ + R_ECEF_ENU_ * delta_ENU;

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

void UKF::checkAndReanchor() {
    if (!use_enu_) return;

    // Check if position has drifted too far from anchor
    double drift = nominal_state_.p_ECEF.norm();  // p_ECEF is actually p_ENU

    if (drift > ENU_REANCHOR_THRESHOLD) {
        std::cout << "Re-anchoring ENU frame (drift: " << drift << "m)\n";

        // Convert to ECEF
        State state_ECEF = enuToEcef(nominal_state_);

        // Re-anchor at current position
        initializeEnu(state_ECEF.p_ECEF);

        // Convert back to new ENU
        nominal_state_ = ecefToEnu(state_ECEF);

        // Covariance remains valid in the new frame
    }
}
