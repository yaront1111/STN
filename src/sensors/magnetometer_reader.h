/**
 * Magnetometer Data Reader
 * Reads real magnetic field data from CSV
 */

#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "../utils/logger.h"
#include "sensor_data.h"

namespace Navigation {

using Vector3d = Eigen::Vector3d;

// MagnetometerData is defined in sensor_data.h

/**
 * Magnetometer Configuration
 */
struct MagnetometerConfig {
    // File format
    std::string csv_path;
    char delimiter = ',';
    bool has_header = true;
    
    // Column indices
    int col_timestamp = 0;
    int col_mag_x = 1;
    int col_mag_y = 2;
    int col_mag_z = 3;
    int col_temperature = -1;  // Optional
    
    // Unit conversions
    double field_scale = 1.0;     // Convert to Tesla
    double time_scale = 1.0;      // Convert to seconds
    bool field_in_gauss = false;  // If true, multiply by 1e-4
    bool field_in_ut = false;     // If true, multiply by 1e-6
    
    // Data validation
    double max_field = 100e-6;    // Tesla (~100 µT max)
    double min_field = 10e-6;     // Tesla (~10 µT min)
    double disturbance_threshold = 70e-6;  // Disturbance detection
    
    // Calibration
    Vector3d hard_iron_offset = Vector3d::Zero();
    Eigen::Matrix3d soft_iron_matrix = Eigen::Matrix3d::Identity();
};

/**
 * Magnetometer Reader
 */
class MagnetometerReader {
private:
    MagnetometerConfig config_;
    std::ifstream file_;
    
    struct Stats {
        uint64_t total_samples = 0;
        uint64_t valid_samples = 0;
        uint64_t disturbed_samples = 0;
        Vector3d min_field = Vector3d::Constant(1e9);
        Vector3d max_field = Vector3d::Constant(-1e9);
    } stats_;
    
public:
    MagnetometerReader(const MagnetometerConfig& config);
    ~MagnetometerReader();
    
    bool open();
    void close();
    bool isOpen() const { return file_.is_open(); }
    
    bool readNext(MagnetometerData& data);
    bool readAll(std::vector<MagnetometerData>& data);
    
    // Calibration
    void applyCalibration(MagnetometerData& data);
    
    // Magnetic field analysis
    static void computeMagneticElements(MagnetometerData& data);
    
    Stats getStatistics() const { return stats_; }
    void printStatistics() const;
    
private:
    bool parseLine(const std::string& line, MagnetometerData& data);
    std::vector<std::string> splitString(const std::string& str, char delimiter);
    bool validateData(MagnetometerData& data);
    bool detectDisturbance(const MagnetometerData& data);
};

} // namespace Navigation