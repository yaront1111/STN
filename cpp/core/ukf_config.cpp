#include "ukf_config.h"
#include <cmath>
#include <algorithm>
#include <iostream>

bool UKFConfig::validate() const {
    // Check UKF parameters
    if (alpha <= 0 || alpha > 1) {
        std::cerr << "Invalid alpha: " << alpha << " (should be 0 < alpha <= 1)\n";
        return false;
    }
    
    if (beta < 0) {
        std::cerr << "Invalid beta: " << beta << " (should be >= 0)\n";
        return false;
    }
    
    // Check process noise values
    if (process_noise.position <= 0 || process_noise.velocity <= 0 || 
        process_noise.attitude <= 0 || process_noise.accel_bias <= 0 || 
        process_noise.gyro_bias <= 0) {
        std::cerr << "Invalid process noise parameters (all should be > 0)\n";
        return false;
    }
    
    // Check measurement noise values
    if (measurement_noise.gravity_gradient <= 0 || measurement_noise.gravity_anomaly <= 0 ||
        measurement_noise.magnetometer <= 0 || measurement_noise.barometer <= 0) {
        std::cerr << "Invalid measurement noise parameters (all should be > 0)\n";
        return false;
    }
    
    // Check numerical parameters
    if (numerical.min_eigenvalue <= 0 || numerical.condition_threshold <= 1) {
        std::cerr << "Invalid numerical parameters\n";
        return false;
    }
    
    return true;
}

void UKFConfig::setDefaults() {
    // UKF parameters (conservative)
    alpha = 0.001;
    beta = 2.0;
    kappa = 0.0;
    
    // PHASE 4: Optimal process noise for Grade B (Phase 1 levels)
    process_noise.position = 0.001;    // 10x reduced from default
    process_noise.velocity = 0.01;     // 10x reduced from default
    process_noise.attitude = 0.0001;   // 10x reduced from default
    process_noise.accel_bias = 1e-6;   // Tactical-grade IMU
    process_noise.gyro_bias = 1e-8;    // Tactical-grade IMU
    process_noise.clock_drift = 1e-9;
    process_noise.clock_freq = 1e-12;
    
    // PHASE 4: Optimal measurement trust for Grade B
    measurement_noise.gravity_gradient = 0.2;      // High trust in gradients
    measurement_noise.gravity_anomaly = 50.0;      // Moderate trust
    measurement_noise.magnetometer = 0.05;         // Good magnetometer
    measurement_noise.barometer = 5.0;             // Standard barometer
    measurement_noise.radar_altimeter = 0.5;       // Good altimeter
    measurement_noise.map_match_position = 10.0;   // CRITICAL: High trust in map
    measurement_noise.zupt_velocity = 0.005;       // Standard ZUPT
    
    // Numerical stability parameters
    numerical.min_eigenvalue = 1e-12;
    numerical.condition_threshold = 1e8;
    numerical.cholesky_tolerance = 1e-10;
    numerical.innovation_outlier_chi2 = 9.21;  // 99% confidence, 3 DOF
    
    // Adaptive filtering (disabled by default)
    adaptive.enable_adaptive_Q = false;
    adaptive.enable_adaptive_R = false;
    adaptive.adaptation_rate = 0.01;
    adaptive.innovation_window = 50;
    
    // Grade A requirements
    grade_a.max_position_error = 50.0;
    grade_a.max_velocity_error = 0.5;
    grade_a.max_attitude_error = 0.5;
    grade_a.map_match_interval = 30.0;
    grade_a.min_gradient_measurements = 50;
}

void UKFConfig::setGradeAOptimal() {
    // Start with defaults
    setDefaults();

    // Optimize for Grade A performance with numerical stability
    alpha = 0.001;    // Small sigma point spread for stability
    beta = 2.0;       // Gaussian assumption
    kappa = 0.0;      // Standard setting

    // Process noise optimized for Grade B+ performance
    process_noise.position = 0.001;    // Reduced for stability (was 0.01)
    process_noise.velocity = 0.01;     // Reduced for stability (was 0.1)
    process_noise.attitude = 0.0001;   // Reduced for stability (was 0.001)
    process_noise.accel_bias = 1e-6;   // Standard tactical-grade IMU
    process_noise.gyro_bias = 1e-8;    // Standard tactical-grade IMU
    
    // FIX: Further optimized measurement noise for Grade A with ML
    // Lower values = more trust in measurements = faster convergence
    measurement_noise.gravity_gradient = 0.1;      // Very high trust (was 1.0)
    measurement_noise.gravity_anomaly = 0.5;       // Very high trust (was 1.0)
    measurement_noise.magnetometer = 25e-9;        // High trust (was 50e-9)
    measurement_noise.barometer = 2.0;             // High trust (was 5.0)
    measurement_noise.radar_altimeter = 0.2;       // Very high trust (was 0.5)
    measurement_noise.map_match_position = 20.0;   // CRITICAL: Much higher trust (was 100.0)
    
    // Tighter gating for Grade A+
    numerical.innovation_outlier_chi2 = 7.815;     // 95% confidence instead of 99%
    
    // Grade A specific settings
    grade_a.map_match_interval = 1.0;              // FIX: Map match every 1s instead of 5s
    grade_a.min_gradient_measurements = 30;        // Lower threshold for faster matching
    
    std::cout << "UKF configured for Grade A+ performance\n";
}

void UKFConfig::setGradeBPlus() {
    // Optimized for Grade B+ performance (<200m RMS)
    setDefaults();

    // UKF parameters tuned for stability
    alpha = 0.001;    // Small but stable spread
    beta = 2.0;       // Gaussian
    kappa = 0.0;      // Standard

    // Process noise - MINIMAL for GPS-denied dead reckoning
    process_noise.position = 1e-6;      // 1 nm/s^1.5 - trust physics
    process_noise.velocity = 1e-4;      // 0.1 mm/s^2.5 - high-quality IMU
    process_noise.attitude = 1e-6;      // 1 μrad/s^1.5 - tactical grade
    process_noise.accel_bias = 1e-9;    // Nearly zero drift
    process_noise.gyro_bias = 1e-12;    // Nearly zero drift

    // Measurement noise - realistic values
    measurement_noise.gravity_gradient = 5.0;       // 5 Eötvös (achievable)
    measurement_noise.gravity_anomaly = 0.5;        // 0.5 mGal (high-quality sensor)
    measurement_noise.magnetometer = 50e-9;         // 50 nT
    measurement_noise.barometer = 5.0;              // 5m altitude
    measurement_noise.map_match_position = 50.0;    // 50m map accuracy

    // Numerical stability
    numerical.min_eigenvalue = 1e-10;
    numerical.condition_threshold = 1e6;            // Tighter bound
    numerical.innovation_outlier_chi2 = 6.635;      // 90% confidence

    // Grade B+ specific
    grade_a.max_position_error = 200.0;             // Grade B+ requirement
    grade_a.map_match_interval = 10.0;              // More frequent matching
    grade_a.min_gradient_measurements = 50;

    std::cout << "UKF configured for Grade B+ performance (<200m RMS)\n";
}

void UKFConfig::setConservative() {
    // Start with defaults
    setDefaults();

    // Conservative settings for numerical stability
    alpha = 0.0001;   // Very small sigma point spread

    // Large process noise for stability
    process_noise.position = 1.0;
    process_noise.velocity = 0.1;
    process_noise.attitude = 0.01;

    // Large measurement noise to avoid rejections
    measurement_noise.gravity_gradient = 10.0;
    measurement_noise.gravity_anomaly = 1000.0;
    measurement_noise.magnetometer = 0.2;

    // Very strict numerical bounds
    numerical.min_eigenvalue = 1e-10;
    numerical.condition_threshold = 1e6;
    numerical.innovation_outlier_chi2 = 15.51;  // 99.9% confidence
    
    std::cout << "UKF configured for conservative/stable operation\n";
}

void UKFConfig::setAggressive() {
    // Start with defaults
    setDefaults();
    
    // Aggressive settings for fast convergence
    alpha = 0.01;     // Larger sigma point spread
    
    // Smaller process noise for tight tracking
    process_noise.position = 0.01;
    process_noise.velocity = 0.001;
    process_noise.attitude = 0.0001;
    
    // Tight measurement noise for aggressive updates
    measurement_noise.gravity_gradient = 0.1;
    measurement_noise.gravity_anomaly = 10.0;
    measurement_noise.magnetometer = 0.01;
    
    // Aggressive gating
    numerical.innovation_outlier_chi2 = 3.84;  // 90% confidence
    
    // Enable adaptive filtering
    adaptive.enable_adaptive_Q = true;
    adaptive.enable_adaptive_R = true;
    adaptive.adaptation_rate = 0.05;
    
    std::cout << "UKF configured for aggressive/fast convergence\n";
}