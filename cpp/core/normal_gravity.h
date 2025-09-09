#pragma once
#include <Eigen/Dense>

/**
 * @brief WGS84 Normal Gravity Model
 * 
 * Provides WGS84 ellipsoidal normal gravity for INS mechanization.
 * This separates normal gravity (used in propagation) from EGM2008 
 * anomalies (used for map matching), preventing double-counting.
 */
class NormalGravity {
public:
    /**
     * @brief Compute WGS84 normal gravity at given position
     * @param lat_rad Latitude in radians
     * @param h_ellip Height above ellipsoid in meters
     * @return Normal gravity vector in ECEF frame [m/s^2]
     */
    static Eigen::Vector3d computeNormalGravity(double lat_rad, double h_ellip);
    
    /**
     * @brief Compute normal gravity magnitude at sea level
     * @param lat_rad Latitude in radians  
     * @return Normal gravity magnitude [m/s^2]
     */
    static double computeNormalGravityMagnitude(double lat_rad);
    
    /**
     * @brief Compute gravity gradient due to normal field
     * @param lat_rad Latitude in radians
     * @param h_ellip Height above ellipsoid in meters
     * @return Normal gravity gradient tensor in ECEF [s^-2]
     */
    static Eigen::Matrix3d computeNormalGradient(double lat_rad, double h_ellip);

private:
    // WGS84 Constants
    static constexpr double WGS84_A = 6378137.0;           // Semi-major axis [m]
    static constexpr double WGS84_F = 1.0/298.257223563;   // Flattening
    static constexpr double WGS84_GM = 3.986004418e14;     // GM [m^3/s^2]
    static constexpr double WGS84_OMEGA = 7.292115e-5;     // Earth rotation [rad/s]
    
    // Derived constants
    static constexpr double WGS84_B = WGS84_A * (1.0 - WGS84_F);  // Semi-minor axis
    static constexpr double WGS84_E2 = 2.0*WGS84_F - WGS84_F*WGS84_F;  // First eccentricity squared
    
    // Normal gravity parameters (WGS84)
    static constexpr double GAMMA_A = 9.7803253359;        // Gravity at equator [m/s^2]
    static constexpr double GAMMA_B = 9.8321849378;        // Gravity at pole [m/s^2]
    static constexpr double K = 0.00193185265241;          // Normal gravity formula constant
};

inline double NormalGravity::computeNormalGravityMagnitude(double lat_rad) {
    double sin2_lat = std::sin(lat_rad) * std::sin(lat_rad);
    return GAMMA_A * (1.0 + K * sin2_lat) / std::sqrt(1.0 - WGS84_E2 * sin2_lat);
}

inline Eigen::Vector3d NormalGravity::computeNormalGravity(double lat_rad, double h_ellip) {
    // Somigliana's formula for normal gravity
    double gamma_0 = computeNormalGravityMagnitude(lat_rad);
    
    // Height correction (approximate)
    double gamma_h = gamma_0 * (1.0 - 2.0 * h_ellip / WGS84_A);
    
    // Convert to ECEF components (pointing toward Earth center)
    double cos_lat = std::cos(lat_rad);
    double sin_lat = std::sin(lat_rad);
    
    // Note: This is simplified - for full accuracy need proper ellipsoidal calculation
    return Eigen::Vector3d(0, 0, -gamma_h);  // Simplified: pure vertical
}