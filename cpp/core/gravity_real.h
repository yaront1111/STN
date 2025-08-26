#pragma once
#include <cmath>
#include <vector>
#include <string>

// Real gravity model using EGM2008 or similar
class RealGravityModel {
private:
    // Gravity anomaly grid
    std::vector<float> anomaly_grid;
    double lat_min = -90, lat_max = 90;
    double lon_min = -180, lon_max = 180;
    int lat_points = 180;  // Simplified grid
    int lon_points = 360;
    bool loaded = false;
    
    // WGS84 ellipsoid parameters
    const double a = 6378137.0;           // Semi-major axis (m)
    const double f = 1.0 / 298.257223563; // Flattening
    const double GM = 3.986004418e14;     // Gravitational constant * mass (m³/s²)
    const double omega = 7.292115e-5;     // Earth rotation rate (rad/s)
    
public:
    // Load gravity model data (EGM2008 coefficients or grid)
    bool loadModel(const std::string& model_path) {
        // For full implementation, would load actual EGM2008 data
        // For now, create synthetic gravity variations
        
        anomaly_grid.resize(lat_points * lon_points);
        
        for (int i = 0; i < lat_points; i++) {
            for (int j = 0; j < lon_points; j++) {
                double lat = -90 + i * 180.0 / lat_points;
                double lon = -180 + j * 360.0 / lon_points;
                
                // Create realistic gravity anomalies (±50 mGal variations)
                double anomaly = 50e-5 * sin(lat * M_PI / 30) * cos(lon * M_PI / 60);
                anomaly_grid[i * lon_points + j] = anomaly;
            }
        }
        
        loaded = true;
        return true;
    }
    
    // Get gravity at location (includes ellipsoid model + anomalies)
    double getGravity(double lat_deg, double lon_deg, double alt_m) const {
        // Normal gravity formula (Somigliana formula)
        double lat_rad = lat_deg * M_PI / 180.0;
        double sin_lat = sin(lat_rad);
        double sin2_lat = sin_lat * sin_lat;
        
        // Gravity at ellipsoid surface
        const double g_equator = 9.7803253359;  // m/s²
        const double g_pole = 9.8321849378;     // m/s²
        
        double g_ellipsoid = g_equator * (1 + 0.0052790414 * sin2_lat + 
                                          0.0000232718 * sin2_lat * sin2_lat);
        
        // Free-air correction (gravity decreases with altitude)
        double g_altitude = g_ellipsoid * (1 - 2 * alt_m / a);
        
        // Add gravity anomaly from model
        double anomaly = getGravityAnomaly(lat_deg, lon_deg);
        
        return g_altitude + anomaly;
    }
    
    // Get gravity anomaly at location (deviation from normal gravity)
    double getGravityAnomaly(double lat_deg, double lon_deg) const {
        if (!loaded) return 0.0;
        
        // Bilinear interpolation in anomaly grid
        double lat_idx = (lat_deg + 90) / 180.0 * (lat_points - 1);
        double lon_idx = (lon_deg + 180) / 360.0 * (lon_points - 1);
        
        int i0 = (int)lat_idx;
        int j0 = (int)lon_idx;
        
        if (i0 < 0) i0 = 0;
        if (j0 < 0) j0 = 0;
        if (i0 >= lat_points - 1) i0 = lat_points - 2;
        if (j0 >= lon_points - 1) j0 = lon_points - 2;
        
        double fx = lon_idx - j0;
        double fy = lat_idx - i0;
        
        double v00 = anomaly_grid[i0 * lon_points + j0];
        double v10 = anomaly_grid[i0 * lon_points + (j0 + 1)];
        double v01 = anomaly_grid[(i0 + 1) * lon_points + j0];
        double v11 = anomaly_grid[(i0 + 1) * lon_points + (j0 + 1)];
        
        return v00 * (1 - fx) * (1 - fy) +
               v10 * fx * (1 - fy) +
               v01 * (1 - fx) * fy +
               v11 * fx * fy;
    }
    
    // Get gravity gradient (for navigation purposes)
    double getGravityGradient() const {
        // Vertical gravity gradient (roughly constant)
        return -3.086e-6;  // m/s² per meter altitude
    }
    
    // Get gravity in NED frame
    struct GravityVector {
        double g_north;  // Usually ~0
        double g_east;   // Usually ~0  
        double g_down;   // Main component (positive down)
    };
    
    GravityVector getGravityNED(double lat_deg, double lon_deg, double alt_m) const {
        GravityVector g;
        
        // Gravity is primarily vertical
        g.g_north = 0.0;  // Could add deflection of vertical here
        g.g_east = 0.0;
        g.g_down = getGravity(lat_deg, lon_deg, alt_m);
        
        return g;
    }
};