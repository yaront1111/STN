/**
 * Sensor Manager
 * Synchronizes multi-rate sensor data
 * Handles time alignment and buffering
 */

#pragma once

#include <memory>
#include <queue>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <yaml-cpp/yaml.h>
#include "sensor_data.h"
#include "imu_reader.h"
#include "barometer_reader.h"
#include "magnetometer_reader.h"
#include "gradiometer_reader.h"
#include "../utils/logger.h"

namespace Navigation {

// Use SensorPacket from sensor_data.h
using SensorData = SensorPacket;

/**
 * Sensor Manager Configuration
 */
struct SensorManagerConfig {
    // Sensor configurations
    IMUConfig imu_config;
    BarometerConfig baro_config;
    MagnetometerConfig mag_config;
    GradiometerConfig grav_config;
    
    // Synchronization parameters
    double imu_rate = 100.0;       // Hz
    double baro_rate = 10.0;       // Hz
    double mag_rate = 10.0;        // Hz
    double grav_rate = 1.0;        // Hz
    
    // Time alignment
    double max_time_offset = 0.1;  // Maximum time misalignment [s]
    bool interpolate_sensors = true;
    
    // Buffering
    size_t buffer_size = 1000;     // Number of samples to buffer
    size_t min_buffer_fill = 100;  // Minimum buffer fill before processing
    
    // Data validation
    bool validate_timestamps = true;
    bool check_dropouts = true;
    double max_dropout_duration = 1.0;  // seconds
};

/**
 * Sensor Manager - Synchronizes all sensor streams
 */
class SensorManager {
private:
    // Configuration
    SensorManagerConfig config_;
    
    // Sensor readers
    std::unique_ptr<IMUReader> imu_reader_;
    std::unique_ptr<BarometerReader> baro_reader_;
    std::unique_ptr<MagnetometerReader> mag_reader_;
    std::unique_ptr<GradiometerReader> grav_reader_;
    
    // Data buffers for each sensor
    std::deque<IMUData> imu_buffer_;
    std::deque<BarometerData> baro_buffer_;
    std::deque<MagnetometerData> mag_buffer_;
    std::deque<GradiometerData> grav_buffer_;
    
    // Synchronization
    std::mutex buffer_mutex_;
    std::condition_variable data_ready_;
    double current_time_ = 0.0;
    double last_imu_time_ = 0.0;
    
    // Statistics
    struct Stats {
        uint64_t total_packets = 0;
        uint64_t imu_samples = 0;
        uint64_t baro_samples = 0;
        uint64_t mag_samples = 0;
        uint64_t grav_samples = 0;
        uint64_t sync_failures = 0;
        uint64_t interpolations = 0;
        double max_sync_error = 0.0;
    } stats_;
    
    // Dropout detection
    double last_valid_time_ = 0.0;
    bool in_dropout_ = false;
    
public:
    SensorManager(const SensorManagerConfig& config);
    SensorManager(const YAML::Node& config);  // YAML constructor
    ~SensorManager();
    
    // Initialize and open all sensors
    bool initialize();
    void shutdown();
    
    // Read synchronized data
    bool readNext(SensorData& data);
    bool readNextTimeout(SensorData& data, int timeout_ms);

    // Navigation system interface (convenience methods)
    SensorData readNext() {
        SensorData data;
        readNext(data);
        return data;
    }

    // Data validation
    bool validateDataFiles();

    // Initial state helpers
    Vector3d getInitialPosition();
    double getFirstTimestamp();

    // Data availability
    bool hasMoreData() const;
    
    // Buffer management
    bool bufferData(size_t samples_to_buffer);
    size_t getBufferSize() const;
    void clearBuffers();
    
    // Time synchronization
    bool synchronizeSensors(double target_time, SensorData& data);
    
    // Statistics
    Stats getStatistics() const { return stats_; }
    void printStatistics() const;
    
    // Sensor availability
    bool hasIMU() const { return imu_reader_ && imu_reader_->isOpen(); }
    bool hasBarometer() const { return baro_reader_ && baro_reader_->isOpen(); }
    bool hasMagnetometer() const { return mag_reader_ && mag_reader_->isOpen(); }
    bool hasGradiometer() const { return grav_reader_ && grav_reader_->isOpen(); }

    // Buffer access for ML and RBPF
    std::vector<IMUData> getIMUBuffer(size_t max_samples = 1000);
    std::vector<GradiometerData> getGravityBuffer(size_t max_samples = 100);

private:
    // Buffer filling from files
    void fillIMUBuffer();
    void fillBaroBuffer();
    void fillMagBuffer();
    void fillGravBuffer();
    
    // Interpolation
    IMUData interpolateIMU(double target_time);
    BarometerData interpolateBaro(double target_time);
    MagnetometerData interpolateMag(double target_time);
    GradiometerData interpolateGrav(double target_time);
    
    // Find nearest samples for interpolation
    template<typename T>
    bool findBracketingSamples(const std::queue<T>& buffer, double target_time,
                               T& before, T& after);
    
    // Validation
    bool validateTimestamps(const SensorData& data);
    bool detectDropout(double timestamp);
    double computeDataQuality(const SensorData& data);
    
    // Logging
    void logSynchronizationError(double error, const std::string& sensor);
};

} // namespace Navigation