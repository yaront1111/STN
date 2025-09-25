/**
 * Barometer Data Reader
 * Reads real barometric pressure data from CSV
 */

#pragma once

#include <string>
#include <fstream>
#include <vector>
#include "../utils/logger.h"
#include "sensor_data.h"

namespace Navigation {

// BarometerData is defined in sensor_data.h

/**
 * Barometer Configuration
 */
struct BarometerConfig {
    // File format
    std::string csv_path;
    char delimiter = ',';
    bool has_header = true;
    
    // Column indices
    int col_timestamp = 0;
    int col_pressure = 1;
    int col_temperature = 2;
    int col_altitude = -1;  // Optional
    
    // Unit conversions
    double pressure_scale = 1.0;  // Convert to Pa
    double time_scale = 1.0;      // Convert to seconds
    bool pressure_in_hpa = false; // If true, multiply by 100
    
    // Data validation
    double min_pressure = 20000.0;   // Pa (~2000m below sea level)
    double max_pressure = 110000.0;  // Pa (~15000m altitude)
    double min_temperature = -50.0;
    double max_temperature = 60.0;
    
    // Noise model
    double pressure_noise_std = 10.0;  // Pa
    double temperature_noise_std = 0.5; // °C
};

/**
 * Barometer Reader
 */
class BarometerReader {
private:
    BarometerConfig config_;
    std::ifstream file_;
    
    // Statistics
    struct Stats {
        uint64_t total_samples = 0;
        uint64_t valid_samples = 0;
        double min_pressure = 1e9;
        double max_pressure = 0;
        double avg_pressure = 0;
        double min_altitude = 1e9;
        double max_altitude = -1e9;
    } stats_;
    
public:
    BarometerReader(const BarometerConfig& config);
    ~BarometerReader();
    
    // File operations
    bool open();
    void close();
    bool isOpen() const { return file_.is_open(); }
    
    // Read operations
    bool readNext(BarometerData& data);
    bool readAll(std::vector<BarometerData>& data);
    
    // Altitude computation
    static double pressureToAltitude(double pressure, double temperature, double sea_level_pressure = 101325.0);
    static double altitudeToPressure(double altitude, double temperature, double sea_level_pressure = 101325.0);
    
    // Statistics
    Stats getStatistics() const { return stats_; }
    void printStatistics() const;
    
private:
    bool parseLine(const std::string& line, BarometerData& data);
    std::vector<std::string> splitString(const std::string& str, char delimiter);
    bool validateData(BarometerData& data);
};

} // namespace Navigation