/**
 * XGM2019e Gravity Map Manager
 * Provides gravity anomaly and gradient data
 * Based on spherical harmonics up to degree/order 5399
 * Incorporates GOCE, GRACE, and newer satellite data
 */

#pragma once

#include "map_manager.h"
#include <vector>
#include <complex>

namespace Navigation {

/**
 * Gravity field query result
 */
struct GravityQueryResult : public MapQueryResult {
    double gravity_anomaly;    // mGal
    Eigen::Matrix<double, 5, 1> gradient_tensor;  // E (STF components)
    double geoid_height;       // meters
    double disturbance;        // mGal
};

/**
 * XGM2019e configuration
 */
struct XGM2019eConfig {
    std::string coefficient_file;  // Path to XGM2019e coefficients
    std::string grid_file;         // Optional pre-computed grid
    int max_degree = 5399;         // Maximum spherical harmonic degree
    bool use_tide_free = true;     // Use tide-free system
    bool cache_grid = true;        // Cache computed values in grid
    double grid_resolution = 0.5;  // degrees
    bool ocean_enhanced = true;    // Use enhanced ocean model
    bool use_goce_data = true;     // Include GOCE satellite data
};

/**
 * Spherical harmonic coefficients
 */
struct SphericalHarmonicCoeff {
    int n;  // Degree
    int m;  // Order
    double C;  // Cosine coefficient
    double S;  // Sine coefficient
};

/**
 * XGM2019e Gravity Map Manager
 */
class XGM2019eMap : public MapManager {
private:
    XGM2019eConfig config_;
    
    // Spherical harmonic coefficients
    std::vector<std::vector<double>> C_nm_;  // Cosine coefficients
    std::vector<std::vector<double>> S_nm_;  // Sine coefficients
    
    // Pre-computed normalization factors
    std::vector<std::vector<double>> norm_factors_;
    
    // Constants
    static constexpr double GM = 3.986004418e14;  // Earth gravitational constant [m³/s²]
    static constexpr double a = 6378137.0;        // Semi-major axis [m]
    static constexpr double omega = 7.292115e-5;  // Earth rotation rate [rad/s]
    
    // Cached Legendre polynomials for efficiency
    mutable std::vector<std::vector<double>> P_nm_cache_;
    mutable double last_lat_cache_ = -999;
    
public:
    XGM2019eMap(const std::string& data_path, const XGM2019eConfig& config);
    ~XGM2019eMap() = default;
    
    // MapManager interface
    bool initialize() override;
    MapQueryResult query(double latitude, double longitude, double altitude = 0) const override;
    bool isAvailable(double latitude, double longitude) const override;
    
    // Gravity-specific queries
    GravityQueryResult queryGravity(double lat, double lon, double alt) const;
    double getGravityAnomaly(double lat, double lon, double alt) const;
    Eigen::Matrix<double, 5, 1> getGradientTensor(double lat, double lon, double alt) const;
    double getGeoidHeight(double lat, double lon) const;
    
    // Get gradient at position (for RBPF)
    Eigen::Matrix<double, 5, 1> getGradient(const Vector3d& position) const {
        // Convert ECEF or NED to lat/lon/alt
        double lat = position.x() / 111111.0;  // Simplified
        double lon = position.y() / (111111.0 * cos(lat * M_PI / 180.0));
        double alt = -position.z();
        return getGradientTensor(lat, lon, alt);
    }
    
protected:
    // MapManager tile loading
    std::shared_ptr<MapTile> loadTile(int tile_x, int tile_y) const override;
    
private:
    // Load coefficients from file
    bool loadCoefficients(const std::string& filename);
    bool loadGrid(const std::string& filename);
    
    // Spherical harmonic synthesis
    double synthesizePoint(double lat, double lon, double r) const;
    Vector3d synthesizeGradient(double lat, double lon, double r) const;
    Matrix3d synthesizeTensor(double lat, double lon, double r) const;
    
    // Legendre polynomial computation
    void computeLegendre(double lat, int max_degree) const;
    double getLegendre(int n, int m) const;
    
    // Normalization factors
    void computeNormalizationFactors(int max_degree);
    double getNormFactor(int n, int m) const;
    
    // Coordinate transformations
    void geodeticToSpherical(double lat, double lon, double alt,
                            double& theta, double& lambda, double& r) const;
    
    // Utility functions
    double factorial(int n) const;
    double computeGravityDisturbance(double anomaly, double lat, double alt) const;
};

/**
 * Factory specialization for XGM2019e
 */
class GravityMapManager : public XGM2019eMap {
public:
    GravityMapManager(const std::string& data_path)
        : XGM2019eMap(data_path, XGM2019eConfig{}) {}
};

} // namespace Navigation