#pragma once

#include <Eigen/Dense>
#include "types.h"

/**
 * Gradiometer Dynamics Model
 *
 * Models the effect of platform motion on gravity gradient measurements.
 * Accounts for:
 * - Centrifugal acceleration gradients
 * - Coriolis effects
 * - Platform vibrations
 * - Sensor misalignment
 */
class GradiometerDynamics {
public:
    struct Config {
        // Sensor configuration
        double baseline_m = 1.0;           // Gradiometer baseline length
        double noise_density = 0.1;         // Eötvös per sqrt(Hz)
        double bandwidth_hz = 10.0;         // Sensor bandwidth

        // Platform dynamics
        double max_angular_rate = 0.5;     // rad/s
        double max_acceleration = 20.0;     // m/s²
        double vibration_amplitude = 0.001; // m
        double vibration_freq_hz = 50.0;    // Hz

        // Calibration parameters
        Eigen::Matrix3d scale_factor = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d misalignment = Eigen::Matrix3d::Identity();
        Eigen::Vector3d bias = Eigen::Vector3d::Zero();
    };

    GradiometerDynamics(const Config& config = Config()) : config_(config) {
        // Initialize noise generator
        noise_gen_.seed(std::random_device{}());
    }

    /**
     * Apply platform motion effects to gravity gradient measurement
     */
    Eigen::Matrix3d applyPlatformEffects(
        const Eigen::Matrix3d& true_gradient,
        const ImuSample& imu,
        const State& state,
        double dt
    ) {
        Eigen::Matrix3d measured = true_gradient;

        // 1. Add centrifugal gradient from platform rotation
        measured += computeCentrifugalGradient(imu.gyro_rps);

        // 2. Add Coriolis effect
        measured += computeCoriolisGradient(state.velocity, imu.gyro_rps);

        // 3. Add acceleration-induced gradient
        measured += computeAccelerationGradient(imu.acc_mps2);

        // 4. Apply sensor imperfections
        measured = applySensorModel(measured);

        // 5. Add vibration-induced noise
        measured += computeVibrationNoise(dt);

        // Enforce symmetry (physical constraint)
        measured = 0.5 * (measured + measured.transpose());

        return measured;
    }

    /**
     * Compensate for known platform effects
     */
    Eigen::Matrix3d compensatePlatformEffects(
        const Eigen::Matrix3d& measured_gradient,
        const ImuSample& imu,
        const State& state
    ) {
        // Remove estimated platform effects
        Eigen::Matrix3d compensated = measured_gradient;

        // Remove centrifugal gradient
        compensated -= computeCentrifugalGradient(imu.gyro_rps);

        // Remove Coriolis effect
        compensated -= computeCoriolisGradient(state.velocity, imu.gyro_rps);

        // Remove acceleration gradient (if in free fall)
        if (imu.acc_mps2.norm() < 1.0) {  // Near free fall
            compensated -= computeAccelerationGradient(imu.acc_mps2);
        }

        // Apply inverse calibration
        compensated = applyInverseCalibration(compensated);

        return compensated;
    }

    /**
     * Estimate gradient measurement uncertainty based on platform dynamics
     */
    Eigen::Matrix3d estimateMeasurementCovariance(
        const ImuSample& imu,
        const State& state,
        double dt
    ) {
        double base_variance = config_.noise_density * config_.noise_density / dt;

        // Increase uncertainty with angular rate
        double angular_factor = 1.0 + imu.gyro_rps.norm() / config_.max_angular_rate;

        // Increase uncertainty with acceleration
        double accel_factor = 1.0 + imu.acc_mps2.norm() / config_.max_acceleration;

        // Combine factors
        double total_variance = base_variance * angular_factor * accel_factor;

        return Eigen::Matrix3d::Identity() * total_variance;
    }

private:
    Config config_;
    std::mt19937 noise_gen_;
    double vibration_phase_ = 0.0;

    /**
     * Compute gradient due to centrifugal acceleration
     * T_ij = -ω_i * ω_j (for rotating platform)
     */
    Eigen::Matrix3d computeCentrifugalGradient(const Eigen::Vector3d& omega) {
        // Centrifugal gradient tensor
        Eigen::Matrix3d T_centrifugal = -omega * omega.transpose();

        // Scale by baseline (gradient is differential)
        T_centrifugal *= config_.baseline_m;

        // Convert to Eötvös (1E = 10^-9 s^-2)
        T_centrifugal *= 1e9;

        return T_centrifugal;
    }

    /**
     * Compute gradient due to Coriolis effect
     * T_ij = -2 * ε_ijk * ω_k * v_j / c (relativistic correction)
     */
    Eigen::Matrix3d computeCoriolisGradient(
        const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& omega
    ) {
        Eigen::Matrix3d T_coriolis = Eigen::Matrix3d::Zero();

        // Simplified Coriolis gradient (first-order approximation)
        Eigen::Vector3d coriolis_vec = 2.0 * omega.cross(velocity);

        // Create antisymmetric contribution
        T_coriolis(0, 1) = -coriolis_vec.z();
        T_coriolis(0, 2) = coriolis_vec.y();
        T_coriolis(1, 0) = coriolis_vec.z();
        T_coriolis(1, 2) = -coriolis_vec.x();
        T_coriolis(2, 0) = -coriolis_vec.y();
        T_coriolis(2, 1) = coriolis_vec.x();

        // Scale by baseline and convert to Eötvös
        T_coriolis *= config_.baseline_m * 1e9 / 9.81;

        return T_coriolis;
    }

    /**
     * Compute gradient due to linear acceleration
     */
    Eigen::Matrix3d computeAccelerationGradient(const Eigen::Vector3d& acceleration) {
        Eigen::Matrix3d T_accel = Eigen::Matrix3d::Zero();

        // Linear acceleration creates a uniform field, not a gradient
        // But sensor misalignment can create apparent gradients

        // Model as small perturbation proportional to acceleration
        double accel_magnitude = acceleration.norm();
        if (accel_magnitude > 0.1) {
            // Create small gradient aligned with acceleration
            Eigen::Vector3d accel_dir = acceleration.normalized();
            T_accel = 0.01 * accel_magnitude * accel_dir * accel_dir.transpose();

            // Convert to Eötvös
            T_accel *= 1e9 / 9.81;
        }

        return T_accel;
    }

    /**
     * Apply sensor model (scale, misalignment, bias)
     */
    Eigen::Matrix3d applySensorModel(const Eigen::Matrix3d& true_gradient) {
        // Apply scale factor and misalignment
        Eigen::Matrix3d measured = config_.misalignment * true_gradient *
                                  config_.misalignment.transpose();

        // Element-wise scale
        measured = measured.cwiseProduct(config_.scale_factor);

        // Add bias (only to diagonal elements for physical consistency)
        measured.diagonal() += config_.bias;

        return measured;
    }

    /**
     * Apply inverse calibration to compensate sensor effects
     */
    Eigen::Matrix3d applyInverseCalibration(const Eigen::Matrix3d& measured) {
        // Remove bias
        Eigen::Matrix3d corrected = measured;
        corrected.diagonal() -= config_.bias;

        // Apply inverse scale
        corrected = corrected.cwiseQuotient(config_.scale_factor);

        // Apply inverse misalignment
        Eigen::Matrix3d inv_misalignment = config_.misalignment.inverse();
        corrected = inv_misalignment * corrected * inv_misalignment.transpose();

        return corrected;
    }

    /**
     * Compute vibration-induced noise
     */
    Eigen::Matrix3d computeVibrationNoise(double dt) {
        // Update vibration phase
        vibration_phase_ += 2.0 * M_PI * config_.vibration_freq_hz * dt;

        // Create vibration pattern
        double vibration = config_.vibration_amplitude * std::sin(vibration_phase_);

        // Vibration creates time-varying gradients
        Eigen::Matrix3d T_vibration = Eigen::Matrix3d::Random() * vibration;

        // Add white noise
        std::normal_distribution<double> noise_dist(0.0, config_.noise_density);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                T_vibration(i, j) += noise_dist(noise_gen_);
            }
        }

        // Enforce symmetry
        T_vibration = 0.5 * (T_vibration + T_vibration.transpose());

        return T_vibration;
    }
};