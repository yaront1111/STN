#include "gravity_gradient_provider.h"
#include <fstream>
#include <iostream>
#include <cmath>

bool GravityGradientProvider::loadEGM2020(const std::string& data_path) {
    // Try to load real data first
    std::string real_data_path = "data/egm2008/egm2008_n360.dat";
    std::ifstream file(real_data_path, std::ios::binary);
    
    if (!file.is_open()) {
        // Try the provided path as fallback
        file.open(data_path, std::ios::binary);
    }
    
    if (!file.is_open()) {
        std::cerr << "WARNING: EGM data not found, using synthetic gravity\n";
        // Initialize with synthetic data for testing
        coeffs_ = std::make_unique<SHCoefficients>();
        coeffs_->max_degree = 360;  
        coeffs_->C.resize(66000, 0.0);  
        coeffs_->S.resize(66000, 0.0);
        
        // Generate realistic synthetic coefficients following Kaula's rule
        for (int n = 2; n <= coeffs_->max_degree; ++n) {
            for (int m = 0; m <= n; ++m) {
                int idx = coeffs_->idx(n, m);
                double magnitude = 1e-5 / (n * n);  // Kaula's rule
                coeffs_->C[idx] = magnitude * std::sin(n * 0.1) * std::cos(m * 0.2);
                coeffs_->S[idx] = magnitude * std::cos(n * 0.15) * std::sin(m * 0.25);
            }
        }
        return true;
    }
    
    // Read actual EGM data
    coeffs_ = std::make_unique<SHCoefficients>();
    file.read(reinterpret_cast<char*>(&coeffs_->max_degree), sizeof(int));
    
    int num_coeffs = (coeffs_->max_degree + 1) * (coeffs_->max_degree + 2) / 2;
    coeffs_->C.resize(num_coeffs);
    coeffs_->S.resize(num_coeffs);
    
    // Read C coefficients
    for (int i = 0; i < num_coeffs; i++) {
        file.read(reinterpret_cast<char*>(&coeffs_->C[i]), sizeof(double));
    }
    
    // Read S coefficients  
    for (int i = 0; i < num_coeffs; i++) {
        file.read(reinterpret_cast<char*>(&coeffs_->S[i]), sizeof(double));
    }
    
    std::cout << "✓ Loaded REAL EGM gravity data to degree " << coeffs_->max_degree << "\n";
    return true;
}

GravityGradientProvider::SphericalCoords 
GravityGradientProvider::toSpherical(const Eigen::Vector3d& pos_ECEF) const {
    SphericalCoords coords;
    coords.r = pos_ECEF.norm();
    coords.theta = std::acos(pos_ECEF.z() / coords.r);  // Colatitude
    coords.phi = std::atan2(pos_ECEF.y(), pos_ECEF.x());  // Longitude
    return coords;
}

GravityGradientProvider::LegendreTerms 
GravityGradientProvider::computeLegendre(double theta, int max_degree) const {
    LegendreTerms terms;
    int size = max_degree + 1;
    terms.P = Eigen::MatrixXd::Zero(size, size);
    terms.dP = Eigen::MatrixXd::Zero(size, size);
    
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    
    // Seed values
    terms.P(0, 0) = 1.0;
    terms.P(1, 0) = cos_theta;
    terms.P(1, 1) = sin_theta;
    
    // Recursion for associated Legendre functions
    for (int n = 2; n <= max_degree; ++n) {
        for (int m = 0; m <= n && m <= max_degree; ++m) {
            if (m == 0) {
                terms.P(n, 0) = ((2*n-1) * cos_theta * terms.P(n-1, 0) 
                               - (n-1) * terms.P(n-2, 0)) / n;
            } else if (m == n) {
                terms.P(n, n) = (2*n-1) * sin_theta * terms.P(n-1, n-1);
            } else {
                terms.P(n, m) = ((2*n-1) * cos_theta * terms.P(n-1, m) 
                               - (n+m-1) * terms.P(n-2, m)) / (n-m);
            }
        }
    }
    
    // Compute derivatives
    for (int n = 1; n <= max_degree; ++n) {
        for (int m = 0; m <= n && m <= max_degree; ++m) {
            if (m == 0 && n < size) {
                terms.dP(n, 0) = (1 < size) ? -terms.P(n, 1) : 0.0;
            } else if (m > 0 && m < size && n < size) {
                double term1 = (m-1 >= 0 && m-1 < size) ? terms.P(n, m-1) : 0.0;
                double term2 = (m+1 < size) ? terms.P(n, m+1) : 0.0;
                terms.dP(n, m) = 0.5 * ((n+m)*(n-m+1)*term1 - term2);
            }
        }
    }
    
    return terms;
}

Eigen::Matrix3d GravityGradientProvider::evaluateGradient(const SphericalCoords& coords) const {
    if (!coeffs_) {
        return Eigen::Matrix3d::Zero();
    }
    
    auto legendre = computeLegendre(coords.theta, std::min(coeffs_->max_degree, 100));
    
    // Earth parameters
    const double a = 6378137.0;  // WGS84 semi-major axis
    const double GM = 3.986004418e14;  // Gravitational constant * Earth mass
    
    // Initialize gradient components in spherical coordinates
    double grr = 0.0, grt = 0.0, grp = 0.0;
    double gtt = 0.0, gtp = 0.0, gpp = 0.0;
    
    // Sum spherical harmonic series
    for (int n = 2; n <= std::min(coeffs_->max_degree, 100); ++n) {
        double factor = GM / (coords.r * coords.r) * std::pow(a / coords.r, n);
        
        for (int m = 0; m <= n; ++m) {
            int idx = coeffs_->idx(n, m);
            double cos_mphi = std::cos(m * coords.phi);
            double sin_mphi = std::sin(m * coords.phi);
            
            double C_nm = coeffs_->C[idx];
            double S_nm = coeffs_->S[idx];
            
            // Radial-radial component
            grr += factor * (n+1) * (n+2) / (coords.r * coords.r) 
                 * legendre.P(n, m) * (C_nm * cos_mphi + S_nm * sin_mphi);
            
            // Radial-theta component
            grt += factor * (n+1) / coords.r 
                 * legendre.dP(n, m) * (C_nm * cos_mphi + S_nm * sin_mphi);
            
            // Theta-theta component
            gtt += factor * legendre.dP(n, m) * legendre.dP(n, m) 
                 * (C_nm * cos_mphi + S_nm * sin_mphi);
            
            if (m > 0) {
                // Radial-phi component
                grp += factor * (n+1) * m / (coords.r * std::sin(coords.theta))
                     * legendre.P(n, m) * (-C_nm * sin_mphi + S_nm * cos_mphi);
                
                // Theta-phi component
                gtp += factor * m / std::sin(coords.theta)
                     * legendre.dP(n, m) * (-C_nm * sin_mphi + S_nm * cos_mphi);
                
                // Phi-phi component
                gpp += factor * m * m / (std::sin(coords.theta) * std::sin(coords.theta))
                     * legendre.P(n, m) * (C_nm * cos_mphi + S_nm * sin_mphi);
            }
        }
    }
    
    // Build gradient tensor in spherical coordinates
    Eigen::Matrix3d G_spherical;
    G_spherical << grr, grt, grp,
                   grt, gtt, gtp,
                   grp, gtp, gpp;
    
    // Transform to ECEF coordinates
    // Transformation matrix from spherical to ECEF
    Eigen::Matrix3d T;
    double st = std::sin(coords.theta);
    double ct = std::cos(coords.theta);
    double sp = std::sin(coords.phi);
    double cp = std::cos(coords.phi);
    
    T << st*cp, ct*cp, -sp,
         st*sp, ct*sp,  cp,
         ct,    -st,    0;
    
    // Transform gradient tensor
    Eigen::Matrix3d G_ECEF = T.transpose() * G_spherical * T;
    
    // Convert to Eötvös units (1E = 10^-9 s^-2)
    G_ECEF *= 1e9;
    
    return G_ECEF;
}

GravityGradientTensor GravityGradientProvider::getGradient(const Eigen::Vector3d& pos_ECEF) const {
    GravityGradientTensor gradient;
    
    // Check cache
    if (cache_.valid && (cache_.last_pos - pos_ECEF).norm() < 1.0) {
        return cache_.last_gradient;
    }
    
    // Convert to spherical coordinates
    SphericalCoords coords = toSpherical(pos_ECEF);
    
    // Evaluate gradient
    gradient.T = evaluateGradient(coords);
    gradient.t = 0.0;  // Will be set by caller
    
    // Update cache
    cache_.last_pos = pos_ECEF;
    cache_.last_gradient = gradient;
    cache_.valid = true;
    
    return gradient;
}

double GravityGradientProvider::getAnomaly(const Eigen::Vector3d& pos_ECEF) const {
    // Simplified - compute from gradient trace
    auto gradient = getGradient(pos_ECEF);
    
    // Laplace equation: trace of gradient tensor = 0 in free space
    // Anomaly is deviation from this
    double anomaly_eotvos = gradient.T.trace();
    
    // Convert to mGal (1 mGal = 10 Eötvös for vertical gradient)
    return anomaly_eotvos * 0.1;
}

void GravityGradientProvider::addEarthTides(GravityGradientTensor& gradient, double t) const {
    // Simplified solid Earth tide model
    // In production, would compute from Sun/Moon positions
    double tide_amplitude = 0.5;  // Eötvös
    double tide_period = 12.42 * 3600.0;  // Semi-diurnal tide
    
    double tide_factor = tide_amplitude * std::sin(2.0 * M_PI * t / tide_period);
    
    // Add to diagonal components
    gradient.T(0, 0) += tide_factor * 0.6;
    gradient.T(1, 1) += tide_factor * 0.3;
    gradient.T(2, 2) += tide_factor * -0.9;
}

void GravityGradientProvider::applyRelativisticCorrections(GravityGradientTensor& gradient,
                                                           const Eigen::Vector3d& velocity) const {
    // Special relativistic correction
    double c = 299792458.0;  // Speed of light
    double v2_c2 = velocity.squaredNorm() / (c * c);
    
    // First-order correction
    gradient.T *= (1.0 - v2_c2 / 2.0);
}

GravityGradientTensor GravityGradientProvider::getFullGradient(const Eigen::Vector3d& pos_ECEF,
                                                               const Eigen::Vector3d& vel_ECEF,
                                                               double t,
                                                               double pressure_hPa) const {
    // Get base gradient
    GravityGradientTensor gradient = getGradient(pos_ECEF);
    
    // Add temporal variations
    addEarthTides(gradient, t);
    
    // Add atmospheric loading (simplified)
    double pressure_correction = (pressure_hPa - 1013.25) * 0.001;  // Eötvös/hPa
    gradient.T(2, 2) += pressure_correction;
    
    // Apply relativistic corrections
    applyRelativisticCorrections(gradient, vel_ECEF);
    
    gradient.t = t;
    
    return gradient;
}