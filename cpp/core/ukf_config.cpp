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
    
    // Conservative process noise
    process_noise.position = 0.1;
    process_noise.velocity = 0.01;
    process_noise.attitude = 0.001;
    process_noise.accel_bias = 1e-6;
    process_noise.gyro_bias = 1e-8;
    process_noise.clock_drift = 1e-9;
    process_noise.clock_freq = 1e-12;
    
    // Conservative measurement noise (based on sensor specifications)
    measurement_noise.gravity_gradient = 1.0;      // 1 Eötvös
    measurement_noise.gravity_anomaly = 100.0;     // 100 mGal
    measurement_noise.magnetometer = 0.1;          // 10% of field strength
    measurement_noise.barometer = 10.0;            // 10 m
    measurement_noise.radar_altimeter = 1.0;       // 1 m
    measurement_noise.map_match_position = 50.0;   // 50 m
    measurement_noise.zupt_velocity = 0.01;        // 1 cm/s
    
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
    
    // Optimize for Grade A+ performance
    alpha = 0.001;    // Small sigma point spread for stability
    beta = 2.0;       // Gaussian assumption
    kappa = 0.0;      // Standard setting
    
    // INCREASED process noise to prevent covariance collapse
    process_noise.position = 10.0;     // Higher to maintain uncertainty
    process_noise.velocity = 50.0;     // Much higher for velocity drift
    process_noise.attitude = 0.5;      // Increased attitude uncertainty
    process_noise.accel_bias = 0.05;   // Higher bias drift
    process_noise.gyro_bias = 5e-3;    // Higher gyro drift
    
    // Calibrated measurement noise - FIXED for numerical stability
    // These values must be large enough to prevent singular covariance matrices
    measurement_noise.gravity_gradient = 10.0;     // 10 Eötvös noise (tensor values are ~50 E)
    measurement_noise.gravity_anomaly = 1.0;       // 1 mGal anomaly noise
    measurement_noise.magnetometer = 50e-9;        // 50 nT magnetometer noise (Tesla)
    measurement_noise.barometer = 5.0;             // 5m barometer altitude noise
    measurement_noise.radar_altimeter = 0.5;       // High-quality radar
    measurement_noise.map_match_position = 100.0;  // Conservative map matching uncertainty
    
    // Tighter gating for Grade A+
    numerical.innovation_outlier_chi2 = 7.815;     // 95% confidence instead of 99%
    
    // Grade A specific settings
    grade_a.map_match_interval = 5.0;              // More frequent map matching
    grade_a.min_gradient_measurements = 30;        // Lower threshold for faster matching
    
    std::cout << "UKF configured for Grade A+ performance\n";
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