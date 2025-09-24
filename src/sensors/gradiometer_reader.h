/**
 * Gradiometer Data Reader
 * Reads gravity gradient tensor data from CSV
 */

#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "../utils/logger.h"
#include "../utils/math_utils.h"
#include "sensor_data.h"

namespace Navigation {

using namespace NavMath;

// GradiometerData is defined in sensor_data.h
// Note: The field is called 'tensor' not 'gradient_tensor' in sensor_data.h

/**
 * Gradiometer Configuration
 */
struct GradiometerConfig {
    // File format
    std::string csv_path;
    char delimiter = ',';
    bool has_header = true;
    
    // Column indices for 5 STF components
    // T_xx, T_xy, T_xz, T_yy, T_yz (T_zz = -(T_xx + T_yy) for trace-free)
    int col_timestamp = 0;
    int col_txx = 1;
    int col_txy = 2;
    int col_txz = 3;
    int col_tyy = 4;
    int col_tyz = 5;
    int col_temperature = -1;  // Optional
    int col_confidence = -1;   // Optional
    
    // Unit conversions
    double gradient_scale = 1.0;  // Convert to Eötvös (1E = 10^-9 s^-2)
    double time_scale = 1.0;      // Convert to seconds
    bool in_eotvos = true;        // If false, assume SI units
    
    // Data validation
    double max_gradient = 3000.0;  // Eötvös
    double trace_tolerance = 10.0; // Eötvös (for trace-free check)
    double min_confidence = 0.3;
    
    // Sensor alignment
    Matrix3d mounting_rotation = Matrix3d::Identity();
};

/**
 * Gradiometer Reader
 */
class GradiometerReader {
private:
    GradiometerConfig config_;
    std::ifstream file_;
    
    struct Stats {
        uint64_t total_samples = 0;
        uint64_t valid_samples = 0;
        uint64_t low_confidence_samples = 0;
        uint64_t trace_violations = 0;
        double min_gradient = 1e9;
        double max_gradient = -1e9;
        double avg_confidence = 0.0;
    } stats_;
    
public:
    GradiometerReader(const GradiometerConfig& config);
    ~GradiometerReader();
    
    bool open();
    void close();
    bool isOpen() const { return file_.is_open(); }
    
    bool readNext(GradiometerData& data);
    bool readAll(std::vector<GradiometerData>& data);
    
    // Tensor operations
    static void reconstructFullTensor(GradiometerData& data);
    static bool validateTraceFree(const GradiometerData& data, double tolerance);
    static void computeInvariants(GradiometerData& data);
    
    // Apply sensor corrections
    void applyMountingCorrection(GradiometerData& data);
    
    Stats getStatistics() const { return stats_; }
    void printStatistics() const;
    
private:
    bool parseLine(const std::string& line, GradiometerData& data);
    std::vector<std::string> splitString(const std::string& str, char delimiter);
    bool validateData(GradiometerData& data);
};

} // namespace Navigation