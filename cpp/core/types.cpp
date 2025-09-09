#include "types.h"
#include <cmath>

void State::fromGeodetic(double lat_rad, double lon_rad, double alt_m) {
    // WGS84 parameters
    const double a = 6378137.0;  // Semi-major axis
    const double e2 = 0.00669437999014;  // First eccentricity squared
    
    double sin_lat = std::sin(lat_rad);
    double cos_lat = std::cos(lat_rad);
    double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
    
    p_ECEF.x() = (N + alt_m) * cos_lat * std::cos(lon_rad);
    p_ECEF.y() = (N + alt_m) * cos_lat * std::sin(lon_rad);
    p_ECEF.z() = (N * (1.0 - e2) + alt_m) * sin_lat;
    
    // Initialize other states
    v_ECEF = Eigen::Vector3d::Zero();
    q_ECEF_B = Eigen::Quaterniond::Identity();
    b_a = Eigen::Vector3d::Zero();
    b_g = Eigen::Vector3d::Zero();
    dt = df = ddf = 0.0;
    t = 0.0;
}

Eigen::Vector3d State::toGeodetic() const {
    // ECEF to geodetic conversion (iterative method)
    const double a = 6378137.0;
    const double e2 = 0.00669437999014;
    
    double p = std::sqrt(p_ECEF.x() * p_ECEF.x() + p_ECEF.y() * p_ECEF.y());
    double lon = std::atan2(p_ECEF.y(), p_ECEF.x());
    
    // Initial guess
    double lat = std::atan2(p_ECEF.z(), p * (1.0 - e2));
    double alt = 0.0;
    
    // Iterate
    for (int i = 0; i < 5; ++i) {
        double sin_lat = std::sin(lat);
        double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
        alt = p / std::cos(lat) - N;
        lat = std::atan2(p_ECEF.z(), p * (1.0 - e2 * N / (N + alt)));
    }
    
    return Eigen::Vector3d(lat, lon, alt);
}