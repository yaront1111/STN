#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * GRAVITY-PRIMARY NAVIGATION TYPES
 * Complete replacement - no TRN legacy code
 */

/**
 * Extended State for Gravity-Primary Navigation
 * Now includes clock states for CSAC integration
 */
struct State {
    // Navigation states in ECEF (Earth-Centered Earth-Fixed)
    Eigen::Vector3d p_ECEF;        // Position in ECEF frame (meters)
    Eigen::Vector3d v_ECEF;        // Velocity in ECEF frame (m/s)
    Eigen::Quaterniond q_ECEF_B;   // Attitude: Body to ECEF rotation
    
    // IMU bias states
    Eigen::Vector3d b_a;           // Accelerometer bias (m/s²)
    Eigen::Vector3d b_g;           // Gyroscope bias (rad/s)
    
    // Clock states for CSAC
    double dt;                     // Clock offset (seconds)
    double df;                     // Clock drift rate (s/s)
    double ddf;                    // Clock drift acceleration (s/s²)
    
    // Time
    double t;                      // Current time (seconds)
    
    // Constructor
    State() : 
        p_ECEF(Eigen::Vector3d::Zero()),
        v_ECEF(Eigen::Vector3d::Zero()),
        q_ECEF_B(Eigen::Quaterniond::Identity()),
        b_a(Eigen::Vector3d::Zero()),
        b_g(Eigen::Vector3d::Zero()),
        dt(0.0), df(0.0), ddf(0.0), t(0.0) {}
    
    // Convert to/from geodetic coordinates
    void fromGeodetic(double lat_rad, double lon_rad, double alt_m);
    Eigen::Vector3d toGeodetic() const;
};

/**
 * IMU Measurement
 */
struct ImuSample {
    double t;                      // Timestamp (seconds)
    Eigen::Vector3d acc_mps2;      // Specific force in body frame (m/s²)
    Eigen::Vector3d gyro_rps;      // Angular rate in body frame (rad/s)
    double temperature_c;          // Temperature for compensation (Celsius)
};

/**
 * Gravity Gradient Tensor Measurement
 * The SECRET SAUCE of gravity-primary navigation
 */
struct GravityGradientTensor {
    Eigen::Matrix3d T;             // Full 3x3 gradient tensor (Eötvös)
    double t;                      // Timestamp
    Eigen::Matrix3d R;             // Measurement covariance
    
    // Constructor
    GravityGradientTensor() : 
        T(Eigen::Matrix3d::Zero()), 
        t(0.0),
        R(Eigen::Matrix3d::Identity() * 0.1) {}  // 0.1 Eötvös noise
    
    // Get trace-free component (removes common mode noise)
    Eigen::Matrix3d getTraceFree() const {
        Eigen::Matrix3d trace_free = T;
        double trace = T.trace() / 3.0;
        trace_free.diagonal().array() -= trace;
        return trace_free;
    }
    
    // Check if measurement is valid
    bool isValid() const {
        // Gradient tensor should be symmetric and bounded
        return (T - T.transpose()).norm() < 0.01 && T.norm() < 1000.0;
    }
};

/**
 * Gravity Anomaly Scalar Measurement
 */
struct GravityAnomaly {
    double value_mgal;             // Anomaly in mGal (10^-5 m/s²)
    Eigen::Vector2d gradient;      // Horizontal gradient (mGal/km)
    double noise_mgal;             // Measurement uncertainty
    double t;                      // Timestamp
};

/**
 * CSAC (Chip Scale Atomic Clock) Measurement
 */
struct CSACMeasurement {
    double offset_s;               // Time offset from GPS (seconds)
    double drift_ppm;              // Drift rate (parts per million)
    double allan_deviation;        // Allan deviation at 1s
    double temperature_c;          // Temperature (Celsius)
    double t;                      // Timestamp
    
    CSACMeasurement() : 
        offset_s(0.0), drift_ppm(0.0), allan_deviation(1e-13),
        temperature_c(25.0), t(0.0) {}
};