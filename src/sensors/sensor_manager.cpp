/**
 * Sensor Manager Implementation
 * Synchronizes all sensor streams
 */

#include "sensor_manager.h"
#include <thread>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <sstream>

namespace Navigation {

SensorManager::SensorManager(const YAML::Node& node) {
    // Parse YAML config
    if (node["imu"]) {
        config_.imu_config.csv_path = node["imu"]["file"].as<std::string>("");
        config_.imu_rate = node["imu"]["rate"].as<double>(100.0);
    }
    if (node["barometer"]) {
        config_.baro_config.csv_path = node["barometer"]["file"].as<std::string>("");
        config_.baro_rate = node["barometer"]["rate"].as<double>(10.0);
    }
    if (node["magnetometer"]) {
        config_.mag_config.csv_path = node["magnetometer"]["file"].as<std::string>("");
        config_.mag_rate = node["magnetometer"]["rate"].as<double>(10.0);
    }
    if (node["gradiometer"]) {
        config_.grav_config.csv_path = node["gradiometer"]["file"].as<std::string>("");
        config_.grav_rate = node["gradiometer"]["rate"].as<double>(1.0);
    }

    LOG_INFO("SensorManager initialized from YAML config");
}

SensorManager::SensorManager(const SensorManagerConfig& config) : config_(config) {
    LOG_INFO("Sensor Manager initialized");
}

SensorManager::~SensorManager() {
    shutdown();
}

bool SensorManager::initialize() {
    LOG_INFO("Initializing sensor manager...");
    
    // Initialize IMU reader
    imu_reader_ = std::make_unique<IMUReader>(config_.imu_config);
    if (!imu_reader_->open()) {
        LOG_ERROR("Failed to open IMU reader");
        return false;
    }
    
    // Initialize barometer reader
    baro_reader_ = std::make_unique<BarometerReader>(config_.baro_config);
    if (!baro_reader_->open()) {
        LOG_WARN("Failed to open barometer reader - continuing without barometer");
        baro_reader_.reset();
    }
    
    // Initialize magnetometer reader
    mag_reader_ = std::make_unique<MagnetometerReader>(config_.mag_config);
    if (!mag_reader_->open()) {
        LOG_WARN("Failed to open magnetometer reader - continuing without magnetometer");
        mag_reader_.reset();
    }
    
    // Initialize gradiometer reader
    grav_reader_ = std::make_unique<GradiometerReader>(config_.grav_config);
    if (!grav_reader_->open()) {
        LOG_WARN("Failed to open gradiometer reader - continuing without gradiometer");
        grav_reader_.reset();
    }
    
    // Reset statistics
    stats_ = Stats();
    current_time_ = 0.0;
    last_valid_time_ = 0.0;
    in_dropout_ = false;
    
    // Initial buffer fill
    if (!bufferData(config_.min_buffer_fill)) {
        LOG_ERROR("Failed to fill initial buffers");
        return false;
    }
    
    LOG_INFO("Sensor manager initialized successfully");
    {

        std::stringstream msg;

        msg << "Available sensors: IMU=" << hasIMU() << ", Baro=" << hasBarometer();

        LOG_INFO(msg.str());

    }
    
    return true;
}

void SensorManager::shutdown() {
    LOG_INFO("Shutting down sensor manager");
    
    if (imu_reader_) {
        imu_reader_->close();
    }
    if (baro_reader_) {
        baro_reader_->close();
    }
    if (mag_reader_) {
        mag_reader_->close();
    }
    if (grav_reader_) {
        grav_reader_->close();
    }
    
    clearBuffers();
    printStatistics();
}

bool SensorManager::readNext(SensorData& data) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    // Must have IMU data
    if (imu_buffer_.empty()) {
        fillIMUBuffer();
        if (imu_buffer_.empty()) {
            LOG_ERROR("No IMU data available");
            return false;
        }
    }
    
    // Get next IMU sample
    IMUData imu = imu_buffer_.front();
    imu_buffer_.pop_front();
    
    // Set target synchronization time
    double target_time = imu.timestamp;
    
    // Synchronize all sensors to IMU time
    if (!synchronizeSensors(target_time, data)) {
        {

            std::stringstream msg;

            msg << "Sensor synchronization failed at time " << target_time;

            LOG_WARN(msg.str());

        }
        stats_.sync_failures++;
    }
    
    // Always include IMU
    data.imu = imu;
    data.has_imu = true;
    data.timestamp = target_time;
    
    // Compute dt
    if (last_imu_time_ > 0) {
        data.dt = target_time - last_imu_time_;
    } else {
        data.dt = 1.0 / config_.imu_rate;
    }
    last_imu_time_ = target_time;
    
    // Validate timestamps
    if (config_.validate_timestamps) {
        validateTimestamps(data);
    }
    
    // Check for dropouts
    if (config_.check_dropouts) {
        detectDropout(target_time);
    }
    
    // Compute overall quality
    // Quality assessment not part of SensorPacket - could add if needed
    // double quality = computeDataQuality(data);
    
    // Update statistics
    stats_.total_packets++;
    stats_.imu_samples++;
    if (data.has_baro) stats_.baro_samples++;
    if (data.has_mag) stats_.mag_samples++;
    if (data.has_grad) stats_.grav_samples++;
    
    current_time_ = target_time;
    
    return true;
}

bool SensorManager::readNextTimeout(SensorData& data, int timeout_ms) {
    auto start = std::chrono::steady_clock::now();
    
    while (true) {
        if (readNext(data)) {
            return true;
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > timeout_ms) {
            return false;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool SensorManager::bufferData(size_t samples_to_buffer) {
    {

        std::stringstream msg;

        msg << "Buffering " << samples_to_buffer << " samples";

        LOG_DEBUG(msg.str());

    }
    
    size_t buffered = 0;
    
    while (buffered < samples_to_buffer) {
        // Fill IMU buffer (highest rate)
        fillIMUBuffer();
        
        // Fill other sensors
        fillBaroBuffer();
        fillMagBuffer();
        fillGravBuffer();
        
        buffered = imu_buffer_.size();
        
        if (imu_buffer_.empty()) {
            LOG_WARN("No more data to buffer");
            break;
        }
    }
    
    {

    
        std::stringstream msg;

    
        msg << "Buffered " << buffered << " IMU samples";

    
        LOG_DEBUG(msg.str());

    
    }
    return buffered > 0;
}

size_t SensorManager::getBufferSize() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(buffer_mutex_));
    return imu_buffer_.size();
}

void SensorManager::clearBuffers() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    imu_buffer_.clear();
    baro_buffer_.clear();
    mag_buffer_.clear();
    grav_buffer_.clear();
}

bool SensorManager::synchronizeSensors(double target_time, SensorData& data) {
    // Initialize all sensors as unavailable
    data.has_baro = false;
    data.has_mag = false;
    data.has_grad = false;
    
    // Synchronize barometer
    if (hasBarometer() && !baro_buffer_.empty()) {
        // Check if we have data near target time
        double front_time = baro_buffer_.front().timestamp;
        
        if (std::abs(front_time - target_time) < config_.max_time_offset) {
            data.barometer = baro_buffer_.front();
            data.has_baro = true;
            baro_buffer_.pop_front();
        } else if (config_.interpolate_sensors && front_time < target_time) {
            // Try interpolation
            data.barometer = interpolateBaro(target_time);
            data.has_baro = data.barometer.valid;
            if (data.has_baro) {
                stats_.interpolations++;
            }
        }
    }
    
    // Synchronize magnetometer
    if (hasMagnetometer() && !mag_buffer_.empty()) {
        double front_time = mag_buffer_.front().timestamp;
        
        if (std::abs(front_time - target_time) < config_.max_time_offset) {
            data.magnetometer = mag_buffer_.front();
            data.has_mag = true;
            mag_buffer_.pop_front();
        } else if (config_.interpolate_sensors && front_time < target_time) {
            data.magnetometer = interpolateMag(target_time);
            data.has_mag = data.magnetometer.valid;
            if (data.has_mag) {
                stats_.interpolations++;
            }
        }
    }
    
    // Synchronize gradiometer
    if (hasGradiometer() && !grav_buffer_.empty()) {
        double front_time = grav_buffer_.front().timestamp;
        
        if (std::abs(front_time - target_time) < config_.max_time_offset) {
            data.gradiometer = grav_buffer_.front();
            data.has_grad = true;
            grav_buffer_.pop_front();
        } else if (config_.interpolate_sensors && front_time < target_time) {
            data.gradiometer = interpolateGrav(target_time);
            data.has_grad = data.gradiometer.valid;
            if (data.has_grad) {
                stats_.interpolations++;
            }
        }
    }
    
    return true;  // IMU is always synchronized
}

void SensorManager::fillIMUBuffer() {
    if (!hasIMU()) return;
    
    size_t target_size = std::min(config_.buffer_size, size_t(100));
    
    while (imu_buffer_.size() < target_size) {
        IMUData data;
        if (imu_reader_->readNext(data)) {
            imu_buffer_.push_back(data);
        } else {
            break;  // No more data
        }
    }
}

void SensorManager::fillBaroBuffer() {
    if (!hasBarometer()) return;
    
    size_t target_size = std::min(config_.buffer_size / 10, size_t(10));
    
    while (baro_buffer_.size() < target_size) {
        BarometerData data;
        if (baro_reader_->readNext(data)) {
            baro_buffer_.push_back(data);
        } else {
            break;
        }
    }
}

void SensorManager::fillMagBuffer() {
    if (!hasMagnetometer()) return;
    
    size_t target_size = std::min(config_.buffer_size / 10, size_t(10));
    
    while (mag_buffer_.size() < target_size) {
        MagnetometerData data;
        if (mag_reader_->readNext(data)) {
            mag_buffer_.push_back(data);
        } else {
            break;
        }
    }
}

void SensorManager::fillGravBuffer() {
    if (!hasGradiometer()) return;
    
    size_t target_size = std::min(config_.buffer_size / 100, size_t(5));
    
    while (grav_buffer_.size() < target_size) {
        GradiometerData data;
        if (grav_reader_->readNext(data)) {
            grav_buffer_.push_back(data);
        } else {
            break;
        }
    }
}

IMUData SensorManager::interpolateIMU(double target_time) {
    // IMU interpolation not typically needed due to high rate
    // Return nearest sample
    if (!imu_buffer_.empty()) {
        return imu_buffer_.front();
    }
    
    IMUData invalid;
    invalid.valid = false;
    return invalid;
}

BarometerData SensorManager::interpolateBaro(double target_time) {
    // Simple linear interpolation
    if (baro_buffer_.size() < 2) {
        BarometerData invalid;
        invalid.valid = false;
        return invalid;
    }
    
    // This is simplified - in production, keep previous sample for interpolation
    BarometerData result = baro_buffer_.front();
    result.timestamp = target_time;
    return result;
}

MagnetometerData SensorManager::interpolateMag(double target_time) {
    if (mag_buffer_.size() < 2) {
        MagnetometerData invalid;
        invalid.valid = false;
        return invalid;
    }
    
    MagnetometerData result = mag_buffer_.front();
    result.timestamp = target_time;
    return result;
}

GradiometerData SensorManager::interpolateGrav(double target_time) {
    if (grav_buffer_.empty()) {
        GradiometerData invalid;
        invalid.valid = false;
        return invalid;
    }
    
    GradiometerData result = grav_buffer_.front();
    result.timestamp = target_time;
    return result;
}

bool SensorManager::validateTimestamps(const SensorData& data) {
    bool valid = true;
    
    // Check for time jumps
    if (data.dt > config_.max_dropout_duration) {
        {

            std::stringstream msg;

            msg << "Large time jump detected: " << data.dt << " seconds";

            LOG_WARN(msg.str());

        }
        valid = false;
    }
    
    // Check for negative time
    if (data.dt < 0) {
        {

            std::stringstream msg;

            msg << "Negative time step: " << data.dt;

            LOG_ERROR(msg.str());

        }
        valid = false;
    }
    
    return valid;
}

bool SensorManager::detectDropout(double timestamp) {
    if (last_valid_time_ > 0) {
        double gap = timestamp - last_valid_time_;
        
        if (gap > config_.max_dropout_duration) {
            if (!in_dropout_) {
                {

                    std::stringstream msg;

                    msg << "Data dropout detected at t=" << timestamp << ", gap=" << gap << "s";

                    LOG_WARN(msg.str());

                }
                in_dropout_ = true;
            }
            return true;
        } else if (in_dropout_) {
            {

                std::stringstream msg;

                msg << "Data dropout recovered at t=" << timestamp;

                LOG_INFO(msg.str());

            }
            in_dropout_ = false;
        }
    }
    
    last_valid_time_ = timestamp;
    return false;
}

double SensorManager::computeDataQuality(const SensorData& data) {
    double quality = 1.0;
    
    // IMU quality
    if (data.has_imu) {
        if (data.imu.accel_saturated || data.imu.gyro_saturated) {
            quality *= 0.5;
        }
    } else {
        quality = 0.0;  // No IMU = no navigation
    }
    
    // Bonus for additional sensors
    if (data.has_baro) quality = std::min(1.0, quality * 1.1);
    if (data.has_mag && !data.magnetometer.is_disturbed) quality = std::min(1.0, quality * 1.1);
    if (data.has_grad) quality = std::min(1.0, quality * 1.2);
    
    // Penalty for dropouts
    if (in_dropout_) {
        quality *= 0.3;
    }
    
    return quality;
}

void SensorManager::logSynchronizationError(double error, const std::string& sensor) {
    {

        std::stringstream msg;

        msg << "Synchronization error for " << sensor << ": " << error << " seconds";

        LOG_WARN(msg.str());

    }
    stats_.max_sync_error = std::max(stats_.max_sync_error, std::abs(error));
}

void SensorManager::printStatistics() const {
    LOG_INFO("=== Sensor Manager Statistics ===");
    {

        std::stringstream msg;

        msg << "Total packets: " << stats_.total_packets;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "IMU samples: " << stats_.imu_samples;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Barometer samples: " << stats_.baro_samples;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Magnetometer samples: " << stats_.mag_samples;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Gradiometer samples: " << stats_.grav_samples;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Sync failures: " << stats_.sync_failures;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Interpolations: " << stats_.interpolations;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Max sync error: " << stats_.max_sync_error << " seconds";

        LOG_INFO(msg.str());

    }
    
    if (stats_.total_packets > 0) {
        LOG_INFO("Sensor availability:");
        {

            std::stringstream msg;

            msg << "  Barometer: " << 100.0 * stats_.baro_samples / stats_.total_packets << "%";

            LOG_INFO(msg.str());

        }
        {

            std::stringstream msg;

            msg << "  Magnetometer: " << 100.0 * stats_.mag_samples / stats_.total_packets << "%";

            LOG_INFO(msg.str());

        }
        {

            std::stringstream msg;

            msg << "  Gradiometer: " << 100.0 * stats_.grav_samples / stats_.total_packets << "%";

            LOG_INFO(msg.str());

        }
    }
}

bool SensorManager::validateDataFiles() {
    LOG_INFO("Validating sensor data files...");

    bool valid = true;

    if (hasIMU()) {
        LOG_INFO("  IMU data: AVAILABLE");
    } else {
        LOG_WARN("  IMU data: NOT AVAILABLE");
        valid = false;  // IMU is required
    }

    if (hasBarometer()) {
        LOG_INFO("  Barometer data: AVAILABLE");
    } else {
        LOG_WARN("  Barometer data: NOT AVAILABLE");
    }

    if (hasMagnetometer()) {
        LOG_INFO("  Magnetometer data: AVAILABLE");
    } else {
        LOG_WARN("  Magnetometer data: NOT AVAILABLE");
    }

    if (hasGradiometer()) {
        LOG_INFO("  Gradiometer data: AVAILABLE");
    } else {
        LOG_WARN("  Gradiometer data: NOT AVAILABLE");
    }

    return valid;
}

Vector3d SensorManager::getInitialPosition() {
    // Try to get initial position from first valid reading
    // This is a simplified version - in practice, might use GPS for initial fix

    Vector3d initial_pos = Vector3d::Zero();

    // Default to origin or configured initial position
    {

        std::stringstream msg;

        msg << "Using default initial position: " << initial_pos.transpose();

        LOG_INFO(msg.str());

    }

    return initial_pos;
}

double SensorManager::getFirstTimestamp() {
    double first_time = 0.0;

    // Get the earliest timestamp from available sensors
    if (!imu_buffer_.empty()) {
        first_time = imu_buffer_.front().timestamp;
    } else if (hasIMU()) {
        // Read first IMU sample
        fillIMUBuffer();
        if (!imu_buffer_.empty()) {
            first_time = imu_buffer_.front().timestamp;
        }
    }

    {


        std::stringstream msg;


        msg << "First timestamp: " << first_time;


        LOG_DEBUG(msg.str());


    }
    return first_time;
}

bool SensorManager::hasMoreData() const {
    // Check if any sensor has more data
    return hasIMU() || !imu_buffer_.empty() ||
           hasBarometer() || !baro_buffer_.empty() ||
           hasMagnetometer() || !mag_buffer_.empty() ||
           hasGradiometer() || !grav_buffer_.empty();
}

std::vector<IMUData> SensorManager::getIMUBuffer(size_t max_samples) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    std::vector<IMUData> result;
    size_t count = std::min(max_samples, imu_buffer_.size());

    // Copy most recent samples
    auto it = imu_buffer_.end();
    std::advance(it, -static_cast<int>(count));

    while (it != imu_buffer_.end()) {
        result.push_back(*it);
        ++it;
    }

    return result;
}

std::vector<GradiometerData> SensorManager::getGravityBuffer(size_t max_samples) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    std::vector<GradiometerData> result;
    size_t count = std::min(max_samples, grav_buffer_.size());

    // Copy most recent samples
    auto it = grav_buffer_.end();
    std::advance(it, -static_cast<int>(count));

    while (it != grav_buffer_.end()) {
        result.push_back(*it);
        ++it;
    }

    return result;
}

} // namespace Navigation