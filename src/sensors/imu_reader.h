/**
 * IMU Data Reader
 * Reads real IMU data from CSV files
 * No mocks - real data only
 */

#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "../utils/logger.h"
#include "sensor_data.h"

namespace Navigation {

using Vector3d = Eigen::Vector3d;

// IMUData is defined in sensor_data.h

/**
 * IMU Configuration
 */
struct IMUConfig {
    // Data format
    std::string csv_path;
    char delimiter = ',';
    bool has_header = true;
    
    // Column indices (0-based)
    int col_timestamp = 0;
    int col_accel_x = 1;
    int col_accel_y = 2;
    int col_accel_z = 3;
    int col_gyro_x = 4;
    int col_gyro_y = 5;
    int col_gyro_z = 6;
    int col_temperature = 7;
    int col_status = -1;  // Optional status column
    
    // Unit conversions
    double accel_scale = 1.0;     // Convert to m/s²
    double gyro_scale = 1.0;      // Convert to rad/s
    double time_scale = 1.0;      // Convert to seconds
    bool gyro_in_degrees = false; // If true, convert from deg/s
    
    // Data validation
    double max_accel = 200.0;     // m/s² (20g)
    double max_gyro = 10.0;       // rad/s
    double min_temperature = -40.0;
    double max_temperature = 85.0;
    
    // Alignment/mounting
    Eigen::Matrix3d mounting_rotation = Eigen::Matrix3d::Identity();
    Vector3d lever_arm = Vector3d::Zero();  // IMU to vehicle center [m]
};

/**
 * IMU Data Reader
 */
class IMUReader {
private:
    IMUConfig config_;
    std::ifstream file_;
    std::vector<IMUData> buffer_;
    size_t current_index_ = 0;
    
    // Statistics
    struct Stats {
        uint64_t total_samples = 0;
        uint64_t valid_samples = 0;
        uint64_t saturated_samples = 0;
        uint64_t temperature_warnings = 0;
        double min_dt = 1e9;
        double max_dt = 0;
        double avg_dt = 0;
    } stats_;
    
    // Previous sample for delta computation
    IMUData prev_sample_;
    bool has_prev_ = false;
    
public:
    IMUReader(const IMUConfig& config);
    ~IMUReader();
    
    // Open and validate file
    bool open();
    void close();
    bool isOpen() const { return file_.is_open(); }
    
    // Read operations
    bool readNext(IMUData& data);
    bool readAll(std::vector<IMUData>& data);
    bool readTimeRange(double start_time, double end_time, std::vector<IMUData>& data);
    
    // Buffered reading for efficiency
    bool bufferNextChunk(size_t chunk_size = 1000);
    bool getBuffered(IMUData& data);
    
    // Get statistics
    Stats getStatistics() const { return stats_; }
    void printStatistics() const;
    
    // Utility functions
    static IMUData interpolate(const IMUData& d1, const IMUData& d2, double t);
    static double computeSampleRate(const std::vector<IMUData>& data);
    
private:
    // Parsing
    bool parseLine(const std::string& line, IMUData& data);
    std::vector<std::string> splitString(const std::string& str, char delimiter);
    
    // Validation
    bool validateData(IMUData& data);
    bool checkSaturation(IMUData& data) const;
    
    // Apply corrections
    void applyMountingCorrection(IMUData& data);
    void applyLeverArm(IMUData& data, const Vector3d& angular_rate);
    
    // Logging
    void logInvalidData(const IMUData& data, const std::string& reason);
};

} // namespace Navigation