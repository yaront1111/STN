#pragma once

/**
 * @brief UKF Configuration and Tuning Parameters
 * 
 * Centralized configuration for UKF parameters, noise models,
 * and tuning constants. Makes parameter tuning much easier.
 */
struct UKFConfig {
    // UKF Parameters
    double alpha = 0.001;      // Sigma point spread (1e-3 to 1)
    double beta = 2.0;         // Prior knowledge of distribution (2 for Gaussian)
    double kappa = 0.0;        // Secondary scaling parameter
    
    // Process Noise (continuous time noise densities)
    struct ProcessNoise {
        double position = 0.1;      // Position noise [m/s^1.5]
        double velocity = 0.01;     // Velocity noise [m/s^2.5] 
        double attitude = 0.001;    // Attitude noise [rad/s^1.5]
        double accel_bias = 1e-6;   // Accelerometer bias [m/s^3.5]
        double gyro_bias = 1e-8;    // Gyroscope bias [rad/s^2.5]
        double clock_drift = 1e-9;  // Clock drift [s/s^1.5]
        double clock_freq = 1e-12;  // Clock frequency [s/s^2.5]
    } process_noise;
    
    // Measurement Noise
    struct MeasurementNoise {
        double gravity_gradient = 1.0;      // [Eötvös]
        double gravity_anomaly = 100.0;     // [mGal]
        double magnetometer = 0.1;          // [unit vector]
        double barometer = 10.0;            // [m]
        double radar_altimeter = 1.0;       // [m]
        double map_match_position = 50.0;   // [m]
        double zupt_velocity = 0.01;        // [m/s]
    } measurement_noise;
    
    // Numerical Stability
    struct NumericalParams {
        double min_eigenvalue = 1e-12;      // Minimum eigenvalue for PD matrices
        double condition_threshold = 1e8;   // Matrix condition number limit
        double cholesky_tolerance = 1e-10;  // Cholesky decomposition tolerance
        double innovation_outlier_chi2 = 9.21;  // Chi-square threshold (99%, 3DOF)
    } numerical;
    
    // Adaptive Filtering (future enhancement)
    struct AdaptiveParams {
        bool enable_adaptive_Q = false;     // Adaptive process noise
        bool enable_adaptive_R = false;     // Adaptive measurement noise  
        double adaptation_rate = 0.01;      // Learning rate
        int innovation_window = 50;         // Window for innovation statistics
    } adaptive;
    
    // Grade A Navigation Specific
    struct GradeAParams {
        double max_position_error = 50.0;   // [m] - Grade A requirement
        double max_velocity_error = 0.5;    // [m/s] - Grade A requirement
        double max_attitude_error = 0.5;    // [deg] - Grade A requirement
        double map_match_interval = 30.0;   // [s] - Frequency of map matching
        int min_gradient_measurements = 50; // Minimum measurements for map match
    } grade_a;
    
    // Validation and bounds checking
    bool validate() const;
    void setDefaults();
    void setGradeAOptimal();  // Optimized settings for Grade A performance
    void setGradeBPlus();     // Optimized settings for Grade B+ (<200m RMS)
    void setConservative();   // Conservative settings for stability
    void setAggressive();     // Aggressive settings for fast convergence
};