/**
 * Data Validator
 * Validates sensor data integrity and detects outliers
 * Uses Mahalanobis distance and statistical checks
 */

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <deque>
#include <yaml-cpp/yaml.h>
#include "logger.h"
#include "math_utils.h"
#include "../sensors/sensor_manager.h"

namespace Navigation {

using namespace NavMath;

/**
 * Validation result with detailed failure reasons
 */
struct ValidationResult {
    bool valid = true;
    bool has_warnings = false;
    
    // Specific checks
    bool range_valid = true;
    bool finite_valid = true;
    bool timestamp_valid = true;
    bool consistency_valid = true;
    bool outlier_valid = true;
    
    // Failure details
    std::string failure_reason;
    std::vector<std::string> warnings;
    
    // Statistics
    double mahalanobis_distance = 0.0;
    double timestamp_gap = 0.0;
};

/**
 * Data Validator Configuration
 */
struct DataValidatorConfig {
    // Range checks
    double max_accel = 200.0;        // m/s² (20g)
    double max_gyro = 10.0;          // rad/s
    double max_mag_field = 100e-6;   // Tesla
    double max_gravity_gradient = 3000.0; // Eötvös
    double min_pressure = 20000.0;   // Pa
    double max_pressure = 110000.0;  // Pa
    
    // Outlier detection
    double mahalanobis_threshold = 5.0;  // Chi-squared threshold
    size_t outlier_window = 100;         // Samples for statistics
    bool use_robust_statistics = true;
    
    // Timestamp validation
    double max_time_gap = 1.0;       // seconds
    double min_time_gap = 0.001;     // seconds (1ms)
    double expected_imu_dt = 0.01;   // 100 Hz
    double dt_tolerance = 0.002;     // 2ms tolerance
    
    // Consistency checks
    double max_accel_change = 50.0;  // m/s² per sample
    double max_gyro_change = 5.0;    // rad/s per sample
    double gravity_norm_tolerance = 2.0; // m/s² from nominal
    
    // Innovation checks (for filter feedback)
    double max_position_innovation = 100.0;  // meters
    double max_velocity_innovation = 10.0;   // m/s
    double max_attitude_innovation = 0.5;    // radians
};

/**
 * Statistical accumulator for outlier detection
 */
class StatisticalAccumulator {
private:
    size_t window_size_;
    std::deque<VectorXd> samples_;
    VectorXd running_mean_;
    MatrixXd running_covariance_;
    bool initialized_ = false;
    
    // Robust statistics
    VectorXd median_;
    MatrixXd mad_covariance_;  // Median Absolute Deviation
    
public:
    StatisticalAccumulator(size_t window_size, size_t dimension);
    
    void addSample(const VectorXd& sample);
    double computeMahalanobisDistance(const VectorXd& sample, bool use_robust = false);
    
    VectorXd getMean() const { return running_mean_; }
    VectorXd getMedian() const { return median_; }
    MatrixXd getCovariance() const { return running_covariance_; }
    bool isInitialized() const { return initialized_ && samples_.size() >= window_size_ / 2; }
    
private:
    void updateStatistics();
    void computeRobustStatistics();
};

/**
 * Data Validator
 */
class DataValidator {
private:
    DataValidatorConfig config_;
    
    // Statistical accumulators for each sensor
    std::unique_ptr<StatisticalAccumulator> imu_accel_stats_;
    std::unique_ptr<StatisticalAccumulator> imu_gyro_stats_;
    std::unique_ptr<StatisticalAccumulator> mag_stats_;
    std::unique_ptr<StatisticalAccumulator> gravity_stats_;
    
    // Previous values for consistency checks
    struct PreviousData {
        Vector3d accel = Vector3d::Zero();
        Vector3d gyro = Vector3d::Zero();
        double timestamp = 0.0;
        bool initialized = false;
    } prev_data_;
    
    // Validation statistics
    struct Stats {
        uint64_t total_checks = 0;
        uint64_t passed_checks = 0;
        uint64_t range_failures = 0;
        uint64_t finite_failures = 0;
        uint64_t timestamp_failures = 0;
        uint64_t consistency_failures = 0;
        uint64_t outlier_detections = 0;
    } stats_;
    
public:
    DataValidator(const DataValidatorConfig& config);
    DataValidator(const YAML::Node& config);  // Overload for YAML
    ~DataValidator() = default;
    
    // Main validation functions
    ValidationResult validateSensorData(const SensorData& data);
    ValidationResult validateIMU(const IMUData& imu);
    ValidationResult validateBarometer(const BarometerData& baro);
    ValidationResult validateMagnetometer(const MagnetometerData& mag);
    ValidationResult validateGradiometer(const GradiometerData& grav);
    
    // Innovation validation (for filter feedback)
    bool validateInnovation(const VectorXd& innovation, const MatrixXd& S, int dof);
    bool validatePositionInnovation(const Vector3d& innovation);
    bool validateVelocityInnovation(const Vector3d& innovation);
    bool validateAttitudeInnovation(const Vector3d& innovation);
    
    // Specific checks
    bool checkFinite(const VectorXd& data);
    bool checkRange(double value, double min_val, double max_val);
    bool checkVectorMagnitude(const Vector3d& vec, double max_magnitude);
    bool checkTimestamp(double current, double previous);
    bool checkConsistency(const Vector3d& current, const Vector3d& previous, double max_change);
    
    // Statistical outlier detection
    bool detectOutlier(const VectorXd& data, StatisticalAccumulator* stats);
    double computeOutlierScore(const VectorXd& data, StatisticalAccumulator* stats);
    
    // Get statistics
    Stats getStatistics() const { return stats_; }
    void printStatistics() const;
    void resetStatistics();
    
    // Configuration
    void updateConfig(const DataValidatorConfig& config) { config_ = config; }
    DataValidatorConfig getConfig() const { return config_; }
    
private:
    // Internal validation helpers
    void addWarning(ValidationResult& result, const std::string& warning);
    void setFailure(ValidationResult& result, const std::string& reason);
    
    // Logging
    void logValidationFailure(const std::string& sensor, const ValidationResult& result);
    void logOutlierDetection(const std::string& sensor, double mahalanobis_dist);
};

} // namespace Navigation