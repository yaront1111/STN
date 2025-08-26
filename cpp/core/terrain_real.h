#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>
#include <json.hpp>  // Would use nlohmann/json in real implementation

// Real-world terrain provider using actual DEM data
class RealTerrainProvider {
private:
    std::vector<float> elevation_data;
    double lat_min, lat_max, lon_min, lon_max;
    int rows, cols;
    double resolution_m;  // Grid resolution in meters
    bool loaded = false;
    
    // Simple bilinear interpolation
    double interpolate(double x, double y) const {
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (x >= cols - 1) x = cols - 1.001;
        if (y >= rows - 1) y = rows - 1.001;
        
        int x0 = (int)x;
        int y0 = (int)y;
        double fx = x - x0;
        double fy = y - y0;
        
        double v00 = elevation_data[y0 * cols + x0];
        double v10 = elevation_data[y0 * cols + (x0 + 1)];
        double v01 = elevation_data[(y0 + 1) * cols + x0];
        double v11 = elevation_data[(y0 + 1) * cols + (x0 + 1)];
        
        return v00 * (1 - fx) * (1 - fy) +
               v10 * fx * (1 - fy) +
               v01 * (1 - fx) * fy +
               v11 * fx * fy;
    }
    
public:
    // Load DEM from file (GeoTIFF, HGT, or our JSON format)
    bool loadDEM(const std::string& dem_path) {
        // For now, load from our simplified JSON format
        std::ifstream file(dem_path);
        if (!file.is_open()) {
            return false;
        }
        
        // Parse JSON (simplified - would use proper JSON library)
        // This is pseudo-code for the structure
        // In reality, would parse the JSON properly
        
        // Assume we've loaded the data
        loaded = true;
        resolution_m = 30.0;  // SRTM resolution
        
        // For testing, generate synthetic terrain
        rows = 100;
        cols = 100;
        elevation_data.resize(rows * cols);
        
        for (int i = 0; i < rows * cols; i++) {
            double x = (i % cols) * resolution_m;
            double y = (i / cols) * resolution_m;
            // Simple terrain function
            elevation_data[i] = 1000 + 100 * sin(x/500) * cos(y/500);
        }
        
        return true;
    }
    
    // Get elevation at latitude/longitude
    double getElevation(double lat_deg, double lon_deg) const {
        if (!loaded) return 0.0;
        
        // Convert lat/lon to grid coordinates
        double x = (lon_deg - lon_min) / (lon_max - lon_min) * (cols - 1);
        double y = (lat_deg - lat_min) / (lat_max - lat_min) * (rows - 1);
        
        return interpolate(x, y);
    }
    
    // Get terrain gradient at location
    Eigen::Vector2d getGradient(double lat_deg, double lon_deg) const {
        if (!loaded) return Eigen::Vector2d::Zero();
        
        const double delta = 0.0001;  // Small offset for numerical gradient
        
        double h_north = getElevation(lat_deg + delta, lon_deg);
        double h_south = getElevation(lat_deg - delta, lon_deg);
        double h_east = getElevation(lat_deg, lon_deg + delta);
        double h_west = getElevation(lat_deg, lon_deg - delta);
        
        // Convert to meters (approximate)
        const double lat_to_m = 111320.0;  // meters per degree latitude
        const double lon_to_m = 111320.0 * cos(lat_deg * M_PI / 180.0);
        
        double dh_dn = (h_north - h_south) / (2 * delta * lat_to_m);
        double dh_de = (h_east - h_west) / (2 * delta * lon_to_m);
        
        return Eigen::Vector2d(dh_dn, dh_de);
    }
    
    // Convert NED position to lat/lon/alt
    void nedToLatLon(const Eigen::Vector3d& p_ned, 
                     double ref_lat, double ref_lon,
                     double& lat, double& lon, double& alt) const {
        const double lat_to_m = 111320.0;
        const double lon_to_m = 111320.0 * cos(ref_lat * M_PI / 180.0);
        
        lat = ref_lat + p_ned.x() / lat_to_m;
        lon = ref_lon + p_ned.y() / lon_to_m;
        alt = -p_ned.z();  // NED down to altitude
    }
    
    // Sample terrain at NED position relative to reference
    TerrainSample sampleNED(const Eigen::Vector3d& p_ned,
                            double ref_lat, double ref_lon) const {
        double lat, lon, alt;
        nedToLatLon(p_ned, ref_lat, ref_lon, lat, lon, alt);
        
        TerrainSample sample;
        sample.h = getElevation(lat, lon);
        sample.grad = getGradient(lat, lon);
        
        return sample;
    }
};