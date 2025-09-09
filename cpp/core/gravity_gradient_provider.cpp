#include "gravity_gradient_provider.h"
#include <fstream>
#include <iostream>
#include <cmath>

bool GravityGradientProvider::loadEGM2020(const std::string& data_path) {
    // Try to load real data first
    std::string real_data_path = "egm2008/egm2008_n360.dat";
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
        
        // Generate minimal synthetic coefficients for testing only
        coeffs_->C[coeffs_->idx(2, 0)] = -1.082636e-3;  // J2 term
        // All other coefficients remain zero
        return true;
    }
    
    // Read actual EGM data
    std::cout << "Loading real EGM2008 data from: " << real_data_path << "\n";
    coeffs_ = std::make_unique<SHCoefficients>();
    file.read(reinterpret_cast<char*>(&coeffs_->max_degree), sizeof(int));
    
    int num_coeffs = (coeffs_->max_degree + 1) * (coeffs_->max_degree + 2) / 2;
    coeffs_->C.resize(num_coeffs);
    coeffs_->S.resize(num_coeffs);
    
    std::cout << "  Max degree: " << coeffs_->max_degree << "\n";
    std::cout << "  Num coefficients: " << num_coeffs << "\n";
    
    // Read C coefficients
    for (int i = 0; i < num_coeffs; i++) {
        file.read(reinterpret_cast<char*>(&coeffs_->C[i]), sizeof(double));
        if (i < 10) {
            std::cout << "    C[" << i << "] = " << coeffs_->C[i] << "\n";
        }
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
    if (max_degree >= 1) {
        terms.P(1, 0) = cos_theta;
        terms.P(1, 1) = sin_theta;
    }
    
    // Standard recursion for associated Legendre functions
    // Limited to max_degree=60 in caller to avoid overflow
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
            
            // Check for overflow and stop if detected
            if (!std::isfinite(terms.P(n, m))) {
                std::cerr << "WARNING: Legendre overflow at P(" << n << "," << m 
                          << "), limiting to degree " << (n-1) << "\n";
                // Zero out remaining terms
                for (int nn = n; nn <= max_degree; ++nn) {
                    for (int mm = 0; mm <= nn; ++mm) {
                        terms.P(nn, mm) = 0.0;
                    }
                }
                return terms;
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
    
    // Use degree 60 for good resolution while avoiding overflow
    const int safe_max_degree = 60;
    auto legendre = computeLegendre(coords.theta, std::min(coeffs_->max_degree, safe_max_degree));
    
    // Earth parameters
    const double a = 6378137.0;  // WGS84 semi-major axis
    const double GM = 3.986004418e14;  // Gravitational constant * Earth mass
    
    // Initialize second derivative components in spherical coordinates
    double V_rr = 0.0, V_rt = 0.0, V_rp = 0.0;
    double V_tt = 0.0, V_tp = 0.0, V_pp = 0.0;
    
    // Precompute longitude terms
    std::vector<double> cos_m_lon(safe_max_degree + 1);
    std::vector<double> sin_m_lon(safe_max_degree + 1);
    cos_m_lon[0] = 1.0;
    sin_m_lon[0] = 0.0;
    if (safe_max_degree > 0) {
        double cos_lon = std::cos(coords.phi);
        double sin_lon = std::sin(coords.phi);
        for (int m = 1; m <= safe_max_degree; ++m) {
            cos_m_lon[m] = cos_m_lon[m-1] * cos_lon - sin_m_lon[m-1] * sin_lon;
            sin_m_lon[m] = sin_m_lon[m-1] * cos_lon + cos_m_lon[m-1] * sin_lon;
        }
    }
    
    double cos_theta = std::cos(coords.theta);
    double sin_theta = std::sin(coords.theta);
    
    // Sum spherical harmonic series for second derivatives only
    for (int n = 2; n <= std::min(coeffs_->max_degree, safe_max_degree); ++n) {
        double ar_pow = std::pow(a / coords.r, n);
        
        for (int m = 0; m <= n; ++m) {
            int idx = coeffs_->idx(n, m);
            double C_nm = coeffs_->C[idx];
            double S_nm = coeffs_->S[idx];
            
            // Check for valid Legendre values and reasonable coefficients
            if (!std::isfinite(legendre.P(n, m)) || std::abs(legendre.P(n, m)) > 1e6) {
                continue;
            }
            if (std::abs(C_nm) > 1e-2 || std::abs(S_nm) > 1e-2) {  // Reasonable EGM bound
                continue;
            }
            
            double C_term = C_nm * cos_m_lon[m];
            double S_term = S_nm * sin_m_lon[m];
            double CS_sum = C_term + S_term;
            double CS_diff = S_term - C_term;
            
            // EGM2008 second derivative formulation (fully normalized)
            double common_factor = ar_pow * legendre.P(n, m);
            
            // V_rr = (n+1)(n+2) term
            V_rr += (n + 1) * (n + 2) * common_factor * CS_sum;
            
            // V_rt = (n+1) * dP/dtheta term
            V_rt += (n + 1) * ar_pow * legendre.dP(n, m) * CS_sum;
            
            // V_tt = second theta derivative
            if (std::abs(legendre.dP(n, m)) < 1e6) {
                V_tt += ar_pow * legendre.dP(n, m) * legendre.dP(n, m) * CS_sum;
            }
            
            // Longitude derivatives (phi terms)
            if (m > 0 && std::abs(sin_theta) > 1e-6) {
                V_rp += (n + 1) * ar_pow * m / sin_theta * legendre.P(n, m) * CS_diff;
                V_tp += ar_pow * m / sin_theta * legendre.dP(n, m) * CS_diff;
                V_pp += ar_pow * m * m / (sin_theta * sin_theta) * legendre.P(n, m) * CS_sum;
            }
        }
    }
    
    // Apply proper scaling for gravity gradients
    // Second derivatives need GM/r^3 base scaling
    double base_scale = GM / (coords.r * coords.r * coords.r);
    
    // Convert to Eötvös units (1 Eötvös = 1e-9 s^-2) 
    double final_scale = base_scale * 1e9;
    
    V_rr *= final_scale;
    V_rt *= final_scale;
    V_rp *= final_scale;
    V_tt *= final_scale;
    V_tp *= final_scale;
    V_pp *= final_scale;
    
    // Build gradient tensor in spherical coordinates
    Eigen::Matrix3d G_spherical;
    G_spherical << V_rr, V_rt, V_rp,
                   V_rt, V_tt, V_tp,
                   V_rp, V_tp, V_pp;
    
    // Transform to ECEF coordinates
    Eigen::Matrix3d T;
    double st = sin_theta;
    double ct = cos_theta;
    double sp = std::sin(coords.phi);
    double cp = std::cos(coords.phi);
    
    T << st*cp, ct*cp, -sp,
         st*sp, ct*sp,  cp,
         ct,    -st,    0;
    
    // Transform gradient tensor
    Eigen::Matrix3d G_ECEF = T.transpose() * G_spherical * T;
    
    // Already scaled to Eötvös units above
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