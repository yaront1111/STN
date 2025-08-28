#pragma once
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <iostream>

// EGM2008 Gravity Anomaly Provider
// Implements the true "spacetime" part of STN
class GravityAnomalyProvider {
private:
    // Grid parameters for EGM2008 (1 arc-minute resolution)
    static constexpr int GRID_ROWS = 10801;  // -90 to +90 degrees
    static constexpr int GRID_COLS = 21601;  // -180 to +180 degrees
    static constexpr double GRID_RES = 1.0 / 60.0;  // 1 arc-minute in degrees
    
    // Anomaly data (mGal converted to m/s^2)
    std::vector<float> anomaly_grid_;
    bool loaded_ = false;
    
    // Bounds
    double lat_min_ = -90.0;
    double lat_max_ = 90.0;
    double lon_min_ = -180.0;
    double lon_max_ = 180.0;
    
public:
    // Load EGM2008 data file
    bool loadEGM2008(const std::string& data_file) {
        std::ifstream file(data_file, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "ERROR: EGM2008 file required but not found: " << data_file << std::endl;
            std::cerr << "Please download EGM2008 data for production use" << std::endl;
            return false;
        }
        
        // Read binary grid (actual EGM2008 format)
        anomaly_grid_.resize(GRID_ROWS * GRID_COLS);
        file.read(reinterpret_cast<char*>(anomaly_grid_.data()), 
                 anomaly_grid_.size() * sizeof(float));
        
        if (!file) {
            std::cerr << "ERROR: Failed to read EGM2008 data" << std::endl;
            return false;
        }
        
        // Convert from mGal to m/s^2 (1 mGal = 1e-5 m/s^2)
        for (auto& val : anomaly_grid_) {
            val *= 1e-5;
        }
        
        loaded_ = true;
        std::cout << "Loaded EGM2008 gravity anomalies" << std::endl;
        return true;
    }
    
    // Get gravity anomaly at specific location
    double getAnomaly(double lat_deg, double lon_deg) const {
        if (!loaded_) {
            // No data available - return zero anomaly
            return 0.0;
        }
        
        // Wrap longitude to [-180, 180]
        while (lon_deg > 180.0) lon_deg -= 360.0;
        while (lon_deg < -180.0) lon_deg += 360.0;
        
        // Clamp latitude
        lat_deg = std::max(-90.0, std::min(90.0, lat_deg));
        
        // Convert to grid indices
        double row = (lat_deg - lat_min_) / GRID_RES;
        double col = (lon_deg - lon_min_) / GRID_RES;
        
        // Bilinear interpolation
        int r0 = static_cast<int>(row);
        int c0 = static_cast<int>(col);
        
        // Bounds checking
        r0 = std::max(0, std::min(GRID_ROWS - 2, r0));
        c0 = std::max(0, std::min(GRID_COLS - 2, c0));
        
        double fr = row - r0;
        double fc = col - c0;
        
        // Get four corner values
        double v00 = anomaly_grid_[r0 * GRID_COLS + c0];
        double v01 = anomaly_grid_[r0 * GRID_COLS + (c0 + 1)];
        double v10 = anomaly_grid_[(r0 + 1) * GRID_COLS + c0];
        double v11 = anomaly_grid_[(r0 + 1) * GRID_COLS + (c0 + 1)];
        
        // Bilinear interpolation
        return v00 * (1 - fr) * (1 - fc) +
               v01 * (1 - fr) * fc +
               v10 * fr * (1 - fc) +
               v11 * fr * fc;
    }
    
    // Get gradient of gravity anomaly (for advanced filtering)
    struct AnomalyGradient {
        double dgdlat;  // Gradient in latitude direction (m/s^2 per degree)
        double dgdlon;  // Gradient in longitude direction (m/s^2 per degree)
    };
    
    AnomalyGradient getGradient(double lat_deg, double lon_deg) const {
        const double delta = 0.01;  // 0.01 degree offset for numerical gradient
        
        double g_north = getAnomaly(lat_deg + delta, lon_deg);
        double g_south = getAnomaly(lat_deg - delta, lon_deg);
        double g_east = getAnomaly(lat_deg, lon_deg + delta);
        double g_west = getAnomaly(lat_deg, lon_deg - delta);
        
        AnomalyGradient grad;
        grad.dgdlat = (g_north - g_south) / (2 * delta);
        grad.dgdlon = (g_east - g_west) / (2 * delta);
        
        return grad;
    }
    
    // Get statistics for current region
    struct AnomalyStats {
        double mean;
        double std_dev;
        double max_gradient;
    };
    
    AnomalyStats getRegionStats(double lat_center, double lon_center, double radius_deg) const {
        AnomalyStats stats = {0, 0, 0};
        
        const int samples = 100;
        std::vector<double> values;
        
        for (int i = 0; i < samples; i++) {
            double angle = 2 * M_PI * i / samples;
            double lat = lat_center + radius_deg * sin(angle);
            double lon = lon_center + radius_deg * cos(angle);
            
            double anomaly = getAnomaly(lat, lon);
            values.push_back(anomaly);
            stats.mean += anomaly;
            
            auto grad = getGradient(lat, lon);
            double grad_mag = sqrt(grad.dgdlat * grad.dgdlat + grad.dgdlon * grad.dgdlon);
            stats.max_gradient = std::max(stats.max_gradient, grad_mag);
        }
        
        stats.mean /= samples;
        
        // Compute standard deviation
        for (double val : values) {
            stats.std_dev += (val - stats.mean) * (val - stats.mean);
        }
        stats.std_dev = sqrt(stats.std_dev / samples);
        
        return stats;
    }
    
private:
    // Create synthetic gravity anomalies for testing
    // Production system requires real EGM2008 data - no synthetic fallback
};