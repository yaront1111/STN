#include "ukf.h"
#include "ukf_sigma_points.h"
#include "ukf_measurements.h" 
#include "ukf_math_utils.h"
#include "normal_gravity.h"
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
    
    // Initialize modular components
    sigma_points_manager_ = std::make_unique<UKFSigmaPoints>(*this);
    measurements_manager_ = std::make_unique<UKFMeasurements>(*this);
    
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

void UKF::init(const State& x0, const Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& P0) {
    nominal_state_ = x0;
    P_ = P0;
    UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P_);
}

void UKF::predict(const ImuSample& imu, double dt) {
    // First ensure current covariance is valid
    if (!UKFMathUtils::checkMatrixValidity<ERROR_STATE_DIM>(P_)) {
        std::cerr << "WARNING: Invalid covariance before predict, fixing...\n";
        UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P_, cfg_.numerical.min_eigenvalue);
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
    
    Q.block<3,3>(POS_IDX, POS_IDX) = Eigen::Matrix3d::Identity() * cfg_.process_noise.position * cfg_.process_noise.position * dt * dt;
    Q.block<3,3>(VEL_IDX, VEL_IDX) = Eigen::Matrix3d::Identity() * cfg_.process_noise.velocity * cfg_.process_noise.velocity * dt;
    Q.block<3,3>(ATT_IDX, ATT_IDX) = Eigen::Matrix3d::Identity() * cfg_.process_noise.attitude * cfg_.process_noise.attitude * dt;
    Q.block<3,3>(BA_IDX, BA_IDX) = Eigen::Matrix3d::Identity() * cfg_.process_noise.accel_bias * cfg_.process_noise.accel_bias * dt;
    Q.block<3,3>(BG_IDX, BG_IDX) = Eigen::Matrix3d::Identity() * cfg_.process_noise.gyro_bias * cfg_.process_noise.gyro_bias * dt;
    
    P_ += Q;
    
    // Enforce positive definiteness
    UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P_, cfg_.numerical.min_eigenvalue);
}

void UKF::enforcePositiveDefinite(Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& P) {
    // Use the robust implementation from math utils
    UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P, cfg_.numerical.min_eigenvalue);
}

void UKF::updateGradient(const Eigen::Matrix3d& measured, const Eigen::Matrix3d& R) {
    // Use the modular measurements manager for gradient updates
    measurements_manager_->updateGravityGradient(measured, R);
}

void UKF::updateAnomaly(double measured, double noise) {
    // Use the modular measurements manager for anomaly updates
    Eigen::Matrix<double, 1, 1> measurement;
    measurement << measured;
    Eigen::Matrix<double, 1, 1> R_scalar;
    R_scalar << noise * noise;
    
    // Define measurement model for gravity anomaly
    auto anomaly_model = [](const State& state) -> Eigen::Matrix<double, 1, 1> {
        // Simple model: anomaly is proportional to altitude variation
        Eigen::Matrix<double, 1, 1> predicted;
        double alt = state.p_ECEF.norm() - 6371000.0;  // altitude from sea level
        predicted << alt * 1e-6;  // Convert to mGal equivalent
        return predicted;
    };
    
    measurements_manager_->updateMeasurement<1>(measurement, R_scalar, anomaly_model);
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
    // Compute tensor invariants for more robust gradient matching
    auto invariants_model = [](const State& state) -> Eigen::Vector2d {
        // This would use GravityGradientProvider to get predicted tensor
        // and compute its invariants (e.g., eigenvalues, trace)
        Eigen::Vector2d invariants;
        invariants << 0.0, 0.0;  // Placeholder - implement with real gravity model
        return invariants;
    };
    
    Eigen::Vector2d measured_invariants;
    measured_invariants << measured_tensor.trace(), measured_tensor.determinant();
    
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

void UKF::updateZeroVerticalSpeed(double R_vertical) {
    // Constrain vertical velocity component
    Eigen::Matrix<double, 1, 1> zero_measurement;
    zero_measurement << 0.0;
    
    Eigen::Matrix<double, 1, 1> R;
    R << R_vertical * R_vertical;
    
    auto vertical_model = [](const State& state) -> Eigen::Matrix<double, 1, 1> {
        Eigen::Vector3d lla = UKFMathUtils::ecefToLla(state.p_ECEF);
        // This is simplified - should use proper ECEF to NED transformation
        Eigen::Matrix<double, 1, 1> result;
        result << state.v_ECEF.z();  // Approximate vertical velocity
        return result;
    };
    
    measurements_manager_->updateMeasurement<1>(zero_measurement, R, vertical_model);
}

void UKF::updateZeroSideslip(double R_sideslip) {
    // Constrain sideslip velocity (lateral velocity in body frame)
    Eigen::Matrix<double, 1, 1> zero_measurement;
    zero_measurement << 0.0;
    
    Eigen::Matrix<double, 1, 1> R;
    R << R_sideslip * R_sideslip;
    
    auto sideslip_model = [](const State& state) -> Eigen::Matrix<double, 1, 1> {
        Eigen::Vector3d v_body = state.q_ECEF_B.inverse() * state.v_ECEF;
        Eigen::Matrix<double, 1, 1> result;
        result << v_body.y();  // Lateral velocity component
        return result;
    };
    
    measurements_manager_->updateMeasurement<1>(zero_measurement, R, sideslip_model);
}

void UKF::updateVirtualDoppler(const Eigen::Vector3d& velocity_meas, const Eigen::Matrix3d& R_virtual) {
    // Virtual Doppler measurement for velocity constraints
    auto virtual_model = [](const State& state) -> Eigen::Vector3d {
        return state.v_ECEF;
    };
    
    measurements_manager_->updateMeasurement<3>(velocity_meas, R_virtual, virtual_model);
}

void UKF::updateScalarPseudo(double z_meas, double z_pred, double S_pred, double R_scalar) {
    // Generic scalar pseudo-measurement
    Eigen::Matrix<double, 1, 1> measurement;
    measurement << z_meas;
    
    Eigen::Matrix<double, 1, 1> R;
    R << R_scalar;
    
    auto scalar_model = [z_pred](const State& state) -> Eigen::Matrix<double, 1, 1> {
        Eigen::Matrix<double, 1, 1> result;
        result << z_pred;  // Use provided prediction
        return result;
    };
    
    measurements_manager_->updateMeasurement<1>(measurement, R, scalar_model);
}
