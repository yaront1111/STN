#include "types.h"
#include <cmath>

// WGS84 Constants
static constexpr double WGS84_A = 6378137.0;           // Semi-major axis (m)
static constexpr double WGS84_F = 1.0 / 298.257223563; // Flattening
static constexpr double WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F; // Eccentricity squared

void State::fromGeodetic(double lat_rad, double lon_rad, double alt_m) {
    // Convert geodetic coordinates to ECEF
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat_rad) * std::sin(lat_rad));
    
    p_ECEF.x() = (N + alt_m) * std::cos(lat_rad) * std::cos(lon_rad);
    p_ECEF.y() = (N + alt_m) * std::cos(lat_rad) * std::sin(lon_rad);
    p_ECEF.z() = (N * (1.0 - WGS84_E2) + alt_m) * std::sin(lat_rad);
}

Eigen::Vector3d State::toGeodetic() const {
    // Convert ECEF to geodetic coordinates
    double x = p_ECEF.x();
    double y = p_ECEF.y();
    double z = p_ECEF.z();
    
    double p = std::sqrt(x*x + y*y);
    double lon = std::atan2(y, x);
    
    // Iterative solution for latitude
    double lat = std::atan2(z, p * (1.0 - WGS84_E2));
    double lat_prev = 0.0;
    double N = 0.0;
    
    for (int i = 0; i < 10; ++i) {
        lat_prev = lat;
        N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat) * std::sin(lat));
        lat = std::atan2(z + WGS84_E2 * N * std::sin(lat), p);
        
        if (std::abs(lat - lat_prev) < 1e-12) break;
    }
    
    double alt = p / std::cos(lat) - N;
    
    return Eigen::Vector3d(lat, lon, alt);
}
