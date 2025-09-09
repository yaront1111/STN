#include <iostream>
#include "cpp/core/gravity_gradient_provider.h"

int main() {
    std::cout << "Testing Gravity Model Loading...\n";
    
    GravityGradientProvider model;
    
    // Test at a specific location (Zurich)
    double lat = 47.3977 * M_PI/180;
    double lon = 8.5456 * M_PI/180;
    double alt = 5000;
    
    // Convert to ECEF
    double a = 6378137.0;  // WGS84 semi-major axis
    double f = 1.0/298.257223563;  // flattening
    double e2 = 2*f - f*f;
    
    double N = a / std::sqrt(1 - e2 * std::sin(lat) * std::sin(lat));
    Eigen::Vector3d pos_ECEF;
    pos_ECEF.x() = (N + alt) * std::cos(lat) * std::cos(lon);
    pos_ECEF.y() = (N + alt) * std::cos(lat) * std::sin(lon);
    pos_ECEF.z() = (N * (1 - e2) + alt) * std::sin(lat);
    
    std::cout << "Position ECEF: " << pos_ECEF.transpose() << " m\n";
    
    // Debug position
    double r = pos_ECEF.norm();
    std::cout << "Radius: " << r << " m (Earth radius = 6.371e6 m)\n";
    std::cout << "Altitude above Earth: " << (r - 6.371e6) << " m\n\n";
    
    // Get gradient
    auto tensor = model.getGradient(pos_ECEF);
    
    std::cout << "Gravity Gradient Tensor:\n";
    std::cout << "  T_xx = " << tensor.T(0,0) << " E\n";
    std::cout << "  T_yy = " << tensor.T(1,1) << " E\n";
    std::cout << "  T_zz = " << tensor.T(2,2) << " E\n";
    std::cout << "  Trace = " << tensor.T.trace() << " E\n";
    std::cout << "  Norm = " << tensor.T.norm() << " E\n";
    
    // Check if reasonable
    if (tensor.T.allFinite() && tensor.T.norm() < 100.0) {
        std::cout << "\n✓ Tensor values are reasonable\n";
    } else {
        std::cout << "\n✗ Tensor values are unreasonable!\n";
    }
    
    return 0;
}