/**
 * Data Validator Implementation
 * Complete validation with statistical outlier detection
 */

#include "data_validator.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <sstream>

namespace Navigation {

// YAML constructor for DataValidator
DataValidator::DataValidator(const YAML::Node& node) {
    // Parse YAML config
    if (node["max_accel"]) config_.max_accel = node["max_accel"].as<double>();
    if (node["max_gyro"]) config_.max_gyro = node["max_gyro"].as<double>();
    if (node["max_mag_field"]) config_.max_mag_field = node["max_mag_field"].as<double>();
    if (node["max_gravity_gradient"]) config_.max_gravity_gradient = node["max_gravity_gradient"].as<double>();
    if (node["mahalanobis_threshold"]) config_.mahalanobis_threshold = node["mahalanobis_threshold"].as<double>();
    if (node["outlier_window"]) config_.outlier_window = node["outlier_window"].as<size_t>();

    // Initialize accumulators
    imu_accel_stats_ = std::make_unique<StatisticalAccumulator>(config_.outlier_window, 3);
    imu_gyro_stats_ = std::make_unique<StatisticalAccumulator>(config_.outlier_window, 3);
    mag_stats_ = std::make_unique<StatisticalAccumulator>(config_.outlier_window, 3);
    gravity_stats_ = std::make_unique<StatisticalAccumulator>(config_.outlier_window, 5);

    LOG_INFO("DataValidator initialized from YAML config");
}

// StatisticalAccumulator Implementation

StatisticalAccumulator::StatisticalAccumulator(size_t window_size, size_t dimension)
    : window_size_(window_size) {
    running_mean_ = VectorXd::Zero(dimension);
    running_covariance_ = MatrixXd::Identity(dimension, dimension);
    median_ = VectorXd::Zero(dimension);
    mad_covariance_ = MatrixXd::Identity(dimension, dimension);
}

void StatisticalAccumulator::addSample(const VectorXd& sample) {
    samples_.push_back(sample);
    
    // Maintain window size
    while (samples_.size() > window_size_) {
        samples_.pop_front();
    }
    
    // Update statistics
    updateStatistics();
    
    // Mark as initialized once we have enough samples
    if (samples_.size() >= window_size_ / 4) {
        initialized_ = true;
    }
}

void StatisticalAccumulator::updateStatistics() {
    if (samples_.empty()) return;
    
    size_t n = samples_.size();
    
    // Compute mean
    running_mean_.setZero();
    for (const auto& s : samples_) {
        running_mean_ += s;
    }
    running_mean_ /= n;
    
    // Compute covariance
    running_covariance_.setZero();
    for (const auto& s : samples_) {
        VectorXd diff = s - running_mean_;
        running_covariance_ += diff * diff.transpose();
    }
    
    if (n > 1) {
        running_covariance_ /= (n - 1);
    }
    
    // Ensure positive definite
    running_covariance_ = MatrixUtils::makePositiveDefinite(running_covariance_, 1e-6);
    
    // Compute robust statistics
    computeRobustStatistics();
}

void StatisticalAccumulator::computeRobustStatistics() {
    if (samples_.size() < 3) return;
    
    size_t dim = running_mean_.size();
    median_ = VectorXd::Zero(dim);
    
    // Compute median for each dimension
    for (int d = 0; d < static_cast<int>(dim); ++d) {
        std::vector<double> values;
        for (const auto& s : samples_) {
            values.push_back(s(d));
        }
        
        std::sort(values.begin(), values.end());
        size_t mid = values.size() / 2;
        
        if (values.size() % 2 == 0) {
            median_(d) = (values[mid-1] + values[mid]) / 2.0;
        } else {
            median_(d) = values[mid];
        }
    }
    
    // Compute MAD covariance
    mad_covariance_.setZero();
    for (const auto& s : samples_) {
        VectorXd diff = (s - median_).cwiseAbs();
        mad_covariance_ += diff * diff.transpose();
    }
    mad_covariance_ /= samples_.size();
    mad_covariance_ *= 1.4826 * 1.4826;  // Scale factor for consistency with normal distribution
    
    // Ensure positive definite
    mad_covariance_ = MatrixUtils::makePositiveDefinite(mad_covariance_, 1e-6);
}

double StatisticalAccumulator::computeMahalanobisDistance(const VectorXd& sample, bool use_robust) {
    if (!initialized_) {
        return 0.0;  // Cannot compute without statistics
    }
    
    VectorXd mean = use_robust ? median_ : running_mean_;
    MatrixXd cov = use_robust ? mad_covariance_ : running_covariance_;
    
    // Add small regularization to avoid singularity
    cov += MatrixXd::Identity(cov.rows(), cov.cols()) * 1e-9;
    
    VectorXd diff = sample - mean;
    double distance = Statistics::mahalanobisDistance(sample, mean, cov);
    
    return distance;
}

// DataValidator Implementation

DataValidator::DataValidator(const DataValidatorConfig& config) : config_(config) {
    // Initialize statistical accumulators
    imu_accel_stats_ = std::make_unique<StatisticalAccumulator>(config.outlier_window, 3);
    imu_gyro_stats_ = std::make_unique<StatisticalAccumulator>(config.outlier_window, 3);
    mag_stats_ = std::make_unique<StatisticalAccumulator>(config.outlier_window, 3);
    gravity_stats_ = std::make_unique<StatisticalAccumulator>(config.outlier_window, 5);
    
    // Reset statistics
    stats_ = Stats();
    
    {

    
        std::stringstream msg;

    
        msg << "Data Validator initialized with Mahalanobis threshold: " << config.mahalanobis_threshold;

    
        LOG_INFO(msg.str());

    
    }
}

ValidationResult DataValidator::validateSensorData(const SensorData& data) {
    ValidationResult result;
    stats_.total_checks++;
    
    // Validate timestamp
    if (prev_data_.initialized) {
        result.timestamp_gap = data.timestamp - prev_data_.timestamp;
        
        if (!checkTimestamp(data.timestamp, prev_data_.timestamp)) {
            setFailure(result, "Invalid timestamp gap: " + std::to_string(result.timestamp_gap));
            result.timestamp_valid = false;
            stats_.timestamp_failures++;
        }
    }
    
    // Validate IMU (required)
    if (data.has_imu) {
        auto imu_result = validateIMU(data.imu);
        if (!imu_result.valid) {
            result.valid = false;
            result.failure_reason = "IMU: " + imu_result.failure_reason;
        }
        result.warnings.insert(result.warnings.end(), 
                              imu_result.warnings.begin(), 
                              imu_result.warnings.end());
    } else {
        setFailure(result, "Missing required IMU data");
    }
    
    // Validate barometer (optional)
    if (data.has_baro) {
        auto baro_result = validateBarometer(data.barometer);
        if (!baro_result.valid) {
            addWarning(result, "Barometer invalid: " + baro_result.failure_reason);
        }
    }
    
    // Validate magnetometer (optional)
    if (data.has_mag) {
        auto mag_result = validateMagnetometer(data.magnetometer);
        if (!mag_result.valid) {
            addWarning(result, "Magnetometer invalid: " + mag_result.failure_reason);
        }
    }
    
    // Validate gradiometer (optional)
    if (data.has_grad) {
        auto grav_result = validateGradiometer(data.gradiometer);
        if (!grav_result.valid) {
            addWarning(result, "Gradiometer invalid: " + grav_result.failure_reason);
        }
    }
    
    // Update previous data
    if (data.has_imu) {
        prev_data_.accel = data.imu.accel;
        prev_data_.gyro = data.imu.gyro;
        prev_data_.timestamp = data.timestamp;
        prev_data_.initialized = true;
    }
    
    // Update statistics
    if (result.valid) {
        stats_.passed_checks++;
    } else {
        logValidationFailure("SensorData", result);
    }
    
    result.has_warnings = !result.warnings.empty();
    
    return result;
}

ValidationResult DataValidator::validateIMU(const IMUData& imu) {
    ValidationResult result;
    stats_.total_checks++;

    // Check finite
    if (!checkFinite(imu.accel) || !checkFinite(imu.gyro)) {
        setFailure(result, "IMU contains NaN or Inf");
        result.finite_valid = false;
        stats_.finite_failures++;
        return result;
    }
    
    // Check ranges
    if (!checkVectorMagnitude(imu.accel, config_.max_accel)) {
        setFailure(result, "Acceleration exceeds limit: " + std::to_string(imu.accel.norm()));
        result.range_valid = false;
        stats_.range_failures++;
        return result;
    }
    
    if (!checkVectorMagnitude(imu.gyro, config_.max_gyro)) {
        setFailure(result, "Gyro rate exceeds limit: " + std::to_string(imu.gyro.norm()));
        result.range_valid = false;
        stats_.range_failures++;
        return result;
    }
    
    // Check consistency with previous sample
    if (prev_data_.initialized) {
        if (!checkConsistency(imu.accel, prev_data_.accel, config_.max_accel_change)) {
            addWarning(result, "Large acceleration change detected");
            result.consistency_valid = false;
            stats_.consistency_failures++;
        }
        
        if (!checkConsistency(imu.gyro, prev_data_.gyro, config_.max_gyro_change)) {
            addWarning(result, "Large gyro rate change detected");
            result.consistency_valid = false;
            stats_.consistency_failures++;
        }
    }
    
    // Statistical outlier detection
    imu_accel_stats_->addSample(imu.accel);
    imu_gyro_stats_->addSample(imu.gyro);
    
    if (imu_accel_stats_->isInitialized()) {
        double accel_mahal = imu_accel_stats_->computeMahalanobisDistance(
            imu.accel, config_.use_robust_statistics);
        result.mahalanobis_distance = accel_mahal;
        
        if (accel_mahal > config_.mahalanobis_threshold) {
            addWarning(result, "Acceleration outlier detected: Mahalanobis = " + 
                      std::to_string(accel_mahal));
            result.outlier_valid = false;
            stats_.outlier_detections++;
            logOutlierDetection("IMU_accel", accel_mahal);
        }
    }
    
    if (imu_gyro_stats_->isInitialized()) {
        double gyro_mahal = imu_gyro_stats_->computeMahalanobisDistance(
            imu.gyro, config_.use_robust_statistics);
        
        if (gyro_mahal > config_.mahalanobis_threshold) {
            addWarning(result, "Gyro outlier detected: Mahalanobis = " + 
                      std::to_string(gyro_mahal));
            result.outlier_valid = false;
            stats_.outlier_detections++;
            logOutlierDetection("IMU_gyro", gyro_mahal);
        }
    }
    
    // Check gravity norm (should be ~9.8 m/s² when stationary)
    double gravity_error = std::abs(imu.accel.norm() - GRAVITY_NOM);
    if (gravity_error > config_.gravity_norm_tolerance) {
        // This is a warning, not a failure
        addWarning(result, "Gravity norm deviation: " + std::to_string(gravity_error) + " m/s²");
    }
    
    // Check for saturation
    if (imu.accel_saturated) {
        addWarning(result, "Accelerometer saturated");
    }
    if (imu.gyro_saturated) {
        addWarning(result, "Gyroscope saturated");
    }
    
    return result;
}

ValidationResult DataValidator::validateBarometer(const BarometerData& baro) {
    ValidationResult result;
    
    // Check pressure range
    if (!checkRange(baro.pressure, config_.min_pressure, config_.max_pressure)) {
        setFailure(result, "Pressure out of range: " + std::to_string(baro.pressure));
        result.range_valid = false;
        stats_.range_failures++;
        return result;
    }
    
    // Check temperature (warning only)
    if (baro.temperature < -50 || baro.temperature > 60) {
        addWarning(result, "Temperature unusual: " + std::to_string(baro.temperature));
    }
    
    return result;
}

ValidationResult DataValidator::validateMagnetometer(const MagnetometerData& mag) {
    ValidationResult result;
    
    // Check finite
    if (!checkFinite(mag.field)) {
        setFailure(result, "Magnetometer contains NaN or Inf");
        result.finite_valid = false;
        stats_.finite_failures++;
        return result;
    }
    
    // Check field strength
    double field_norm = mag.field.norm();

    // Check for zero field (sensor failure)
    if (field_norm < 1e-9) {  // Less than 1 nT is essentially zero
        setFailure(result, "Magnetic field is zero (sensor failure)");
        result.range_valid = false;
        stats_.range_failures++;
        return result;
    }

    if (field_norm > config_.max_mag_field) {
        setFailure(result, "Magnetic field too strong: " + std::to_string(field_norm * 1e6) + " µT");
        result.range_valid = false;
        stats_.range_failures++;
        return result;
    }
    
    // Check for disturbance
    if (mag.is_disturbed) {
        addWarning(result, "Magnetic disturbance detected");
    }
    
    // Statistical outlier detection
    mag_stats_->addSample(mag.field);
    
    if (mag_stats_->isInitialized()) {
        double mag_mahal = mag_stats_->computeMahalanobisDistance(
            mag.field, config_.use_robust_statistics);
        result.mahalanobis_distance = mag_mahal;
        
        if (mag_mahal > config_.mahalanobis_threshold) {
            addWarning(result, "Magnetometer outlier: Mahalanobis = " + std::to_string(mag_mahal));
            result.outlier_valid = false;
            stats_.outlier_detections++;
        }
    }
    
    return result;
}

ValidationResult DataValidator::validateGradiometer(const GradiometerData& grav) {
    ValidationResult result;
    
    // Check finite
    if (!grav.gradient_tensor.allFinite()) {
        setFailure(result, "Gradiometer contains NaN or Inf");
        result.finite_valid = false;
        stats_.finite_failures++;
        return result;
    }
    
    // Check gradient magnitude
    double grad_norm = grav.gradient_tensor.norm();
    if (grad_norm > config_.max_gravity_gradient) {
        setFailure(result, "Gravity gradient too large: " + std::to_string(grad_norm) + " E");
        result.range_valid = false;
        stats_.range_failures++;
        return result;
    }
    
    // Check trace (should be near zero)
    if (std::abs(grav.trace) > 10.0) {  // Eötvös
        addWarning(result, "Tensor not trace-free: trace = " + std::to_string(grav.trace));
    }
    
    // Check confidence
    if (grav.confidence < 0.5) {
        addWarning(result, "Low confidence measurement: " + std::to_string(grav.confidence));
    }
    
    // Statistical outlier detection
    // gradient_tensor is a 3x3 matrix, but stats expects a vector
    // Convert to vector form (5 independent components for symmetric trace-free)
    Eigen::VectorXd tensor_vec(5);
    tensor_vec << grav.gradient_tensor(0,0), grav.gradient_tensor(0,1),
                  grav.gradient_tensor(0,2), grav.gradient_tensor(1,1),
                  grav.gradient_tensor(1,2);
    gravity_stats_->addSample(tensor_vec);
    
    if (gravity_stats_->isInitialized()) {
        // Convert to vector form for Mahalanobis distance
        Eigen::VectorXd tensor_vec2(5);
        tensor_vec2 << grav.gradient_tensor(0,0), grav.gradient_tensor(0,1),
                      grav.gradient_tensor(0,2), grav.gradient_tensor(1,1),
                      grav.gradient_tensor(1,2);
        double grav_mahal = gravity_stats_->computeMahalanobisDistance(
            tensor_vec2, config_.use_robust_statistics);
        result.mahalanobis_distance = grav_mahal;
        
        if (grav_mahal > config_.mahalanobis_threshold) {
            addWarning(result, "Gradiometer outlier: Mahalanobis = " + std::to_string(grav_mahal));
            result.outlier_valid = false;
            stats_.outlier_detections++;
        }
    }
    
    return result;
}

bool DataValidator::validateInnovation(const VectorXd& innovation, const MatrixXd& S, int dof) {
    // Normalized Innovation Squared test
    double nis = Statistics::computeNIS(innovation, S);
    double chi2_threshold = Statistics::chi2Threshold(dof, 0.05);
    
    if (nis > chi2_threshold) {
        {

            std::stringstream msg;

            msg << "Innovation test failed: NIS = " << nis << " > " << chi2_threshold;

            LOG_WARN(msg.str());

        }
        return false;
    }
    
    return true;
}

bool DataValidator::validatePositionInnovation(const Vector3d& innovation) {
    return innovation.norm() < config_.max_position_innovation;
}

bool DataValidator::validateVelocityInnovation(const Vector3d& innovation) {
    return innovation.norm() < config_.max_velocity_innovation;
}

bool DataValidator::validateAttitudeInnovation(const Vector3d& innovation) {
    return innovation.norm() < config_.max_attitude_innovation;
}

bool DataValidator::checkFinite(const VectorXd& data) {
    return data.allFinite();
}

bool DataValidator::checkRange(double value, double min_val, double max_val) {
    return value >= min_val && value <= max_val;
}

bool DataValidator::checkVectorMagnitude(const Vector3d& vec, double max_magnitude) {
    return vec.norm() <= max_magnitude;
}

bool DataValidator::checkTimestamp(double current, double previous) {
    double dt = current - previous;
    
    // Check for negative time
    if (dt < 0) {
        {

            std::stringstream msg;

            msg << "Negative timestamp delta: " << dt;

            LOG_ERROR(msg.str());

        }
        return false;
    }
    
    // Check for too large gap
    if (dt > config_.max_time_gap) {
        {

            std::stringstream msg;

            msg << "Large timestamp gap: " << dt << " seconds";

            LOG_WARN(msg.str());

        }
        return false;
    }
    
    // Check for too small gap (duplicate data?)
    if (dt < config_.min_time_gap) {
        {

            std::stringstream msg;

            msg << "Timestamp gap too small: " << dt << " seconds";

            LOG_WARN(msg.str());

        }
        return false;
    }
    
    return true;
}

bool DataValidator::checkConsistency(const Vector3d& current, const Vector3d& previous, 
                                     double max_change) {
    Vector3d delta = current - previous;
    return delta.norm() <= max_change;
}

bool DataValidator::detectOutlier(const VectorXd& data, StatisticalAccumulator* stats) {
    if (!stats || !stats->isInitialized()) {
        return false;  // Cannot detect without statistics
    }
    
    double mahal_dist = stats->computeMahalanobisDistance(data, config_.use_robust_statistics);
    return mahal_dist > config_.mahalanobis_threshold;
}

double DataValidator::computeOutlierScore(const VectorXd& data, StatisticalAccumulator* stats) {
    if (!stats || !stats->isInitialized()) {
        return 0.0;
    }
    
    return stats->computeMahalanobisDistance(data, config_.use_robust_statistics);
}

void DataValidator::printStatistics() const {
    LOG_INFO("=== Data Validator Statistics ===");
    {

        std::stringstream msg;

        msg << "Total checks: " << stats_.total_checks;

        LOG_INFO(msg.str());

    }
    {
        std::stringstream msg;
        msg << "Passed checks: " << stats_.passed_checks
             << " (" << 100.0 * stats_.passed_checks / stats_.total_checks << "%)";
        LOG_INFO(msg.str());
    }
    {

        std::stringstream msg;

        msg << "Range failures: " << stats_.range_failures;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Finite failures: " << stats_.finite_failures;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Timestamp failures: " << stats_.timestamp_failures;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Consistency failures: " << stats_.consistency_failures;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Outlier detections: " << stats_.outlier_detections;

        LOG_INFO(msg.str());

    }
}

void DataValidator::resetStatistics() {
    stats_ = Stats();
    
    // Reset accumulators
    imu_accel_stats_ = std::make_unique<StatisticalAccumulator>(config_.outlier_window, 3);
    imu_gyro_stats_ = std::make_unique<StatisticalAccumulator>(config_.outlier_window, 3);
    mag_stats_ = std::make_unique<StatisticalAccumulator>(config_.outlier_window, 3);
    gravity_stats_ = std::make_unique<StatisticalAccumulator>(config_.outlier_window, 5);
}

void DataValidator::addWarning(ValidationResult& result, const std::string& warning) {
    result.warnings.push_back(warning);
    result.has_warnings = true;
}

void DataValidator::setFailure(ValidationResult& result, const std::string& reason) {
    result.valid = false;
    result.failure_reason = reason;
}

void DataValidator::logValidationFailure(const std::string& sensor, const ValidationResult& result) {
    {

        std::stringstream msg;

        msg << "Validation failed for " << sensor << ": " << result.failure_reason;

        LOG_WARN(msg.str());

    }
    
    for (const auto& warning : result.warnings) {
        {

            std::stringstream msg;

            msg << "  Warning: " << warning;

            LOG_DEBUG(msg.str());

        }
    }
}

void DataValidator::logOutlierDetection(const std::string& sensor, double mahalanobis_dist) {
    {

        std::stringstream msg;

        msg << "Outlier detected in " << sensor << ": Mahalanobis distance = " << mahalanobis_dist;

        LOG_DEBUG(msg.str());

    }
}

} // namespace Navigation