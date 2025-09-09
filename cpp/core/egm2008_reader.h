#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

// EGM2008 gravity anomaly grid reader
class EGM2008Reader {
private:
    std::vector<float> anomalies_;
    int n_lat_, n_lon_;
    float lat_min_, lat_max_, lon_min_, lon_max_;
    float resolution_;
    bool loaded_ = false;
    
public:
    explicit EGM2008Reader(const std::string& data_file = "data/egm2008.dat") {
        loadGrid(data_file);
    }
    
    bool loadGrid(const std::string& filename) {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            // Silent fail - will use synthetic anomalies
            return false;
        }
        
        // Read header
        file.read(reinterpret_cast<char*>(&n_lat_), sizeof(int));
        file.read(reinterpret_cast<char*>(&n_lon_), sizeof(int));
        file.read(reinterpret_cast<char*>(&lat_min_), sizeof(float));
        file.read(reinterpret_cast<char*>(&lat_max_), sizeof(float));
        file.read(reinterpret_cast<char*>(&lon_min_), sizeof(float));
        file.read(reinterpret_cast<char*>(&lon_max_), sizeof(float));
        file.read(reinterpret_cast<char*>(&resolution_), sizeof(float));
        
        // Validate header values
        if (n_lat_ <= 0 || n_lat_ > 10000 || n_lon_ <= 0 || n_lon_ > 10000) {
            std::cerr << "Invalid EGM2008 grid dimensions: " << n_lat_ << "x" << n_lon_ << std::endl;
            return false;
        }
        
        // Read data
        anomalies_.resize(n_lat_ * n_lon_);
        file.read(reinterpret_cast<char*>(anomalies_.data()), 
                  anomalies_.size() * sizeof(float));
        
        if (!file.good() && !file.eof()) {
            std::cerr << "Error reading EGM2008 data" << std::endl;
            return false;
        }
        
        loaded_ = true;
        std::cout << "Loaded EGM2008 gravity grid: " << n_lat_ << "x" << n_lon_ 
                  << " at " << resolution_ << " degree resolution" << std::endl;
        
        return true;
    }
    
    // Get gravity anomaly at location (in mGal)
    double getAnomaly(double lat, double lon) const {
        if (!loaded_) {
            // Fallback to synthetic anomalies
            return getSyntheticAnomaly(lat, lon);
        }
        
        // Normalize longitude to [-180, 180]
        while (lon > 180) lon -= 360;
        while (lon < -180) lon += 360;
        
        // Convert to grid coordinates
        double lat_idx = (lat - lat_min_) / resolution_;
        double lon_idx = (lon - lon_min_) / resolution_;
        
        // Bounds checking
        if (lat_idx < 0) lat_idx = 0;
        if (lat_idx >= n_lat_ - 1) lat_idx = n_lat_ - 1.001;
        if (lon_idx < 0) lon_idx = 0;
        if (lon_idx >= n_lon_ - 1) lon_idx = n_lon_ - 1.001;
        
        // Bilinear interpolation
        int i0 = static_cast<int>(lat_idx);
        int j0 = static_cast<int>(lon_idx);
        double fx = lon_idx - j0;
        double fy = lat_idx - i0;
        
        double v00 = anomalies_[i0 * n_lon_ + j0];
        double v01 = anomalies_[i0 * n_lon_ + (j0 + 1)];
        double v10 = anomalies_[(i0 + 1) * n_lon_ + j0];
        double v11 = anomalies_[(i0 + 1) * n_lon_ + (j0 + 1)];
        
        return v00 * (1 - fx) * (1 - fy) +
               v01 * fx * (1 - fy) +
               v10 * (1 - fx) * fy +
               v11 * fx * fy;
    }
    
    // Get gravity gradient (mGal per meter)
    Eigen::Vector2d getGradient(double lat, double lon) const {
        const double delta = 0.00001;  // ~1 meter at equator
        
        double g_north = getAnomaly(lat + delta, lon);
        double g_south = getAnomaly(lat - delta, lon);
        double g_east = getAnomaly(lat, lon + delta);
        double g_west = getAnomaly(lat, lon - delta);
        
        // Convert to mGal per meter
        const double lat_to_m = 111320.0;
        const double lon_to_m = 111320.0 * cos(lat * M_PI / 180.0);
        
        double dg_dlat = (g_north - g_south) / (2 * delta * lat_to_m);
        double dg_dlon = (g_east - g_west) / (2 * delta * lon_to_m);
        
        return Eigen::Vector2d(dg_dlat, dg_dlon);
    }
    
    // Fallback synthetic anomalies
    double getSyntheticAnomaly(double lat, double lon) const {
        // Simple model with latitude dependence and some features
        double anomaly = 0;
        
        // Latitude effect
        anomaly += 10 * sin(lat * M_PI / 180);
        
        // Add some regional features
        anomaly += 5 * sin(lon * M_PI / 60) * cos(lat * M_PI / 45);
        
        return anomaly;
    }
    
    bool isLoaded() const { return loaded_; }
};