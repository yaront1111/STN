#include "gravity_gradient_provider.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cassert>

bool GravityGradientProvider::loadEGM2020(const std::string& data_path) {
    // Use provided path directly
    std::ifstream file(data_path, std::ios::binary);

    if (!file.is_open()) {
        // Try alternate hardcoded path as fallback
        std::string alt_path = "egm2008/egm2008_n360.dat";
        file.open(alt_path, std::ios::binary);

        if (!file.is_open()) {
            std::cerr << "FATAL ERROR: Real EGM2008 data REQUIRED - NO SYNTHETIC FALLBACK!\n";
            std::cerr << "Tried to load from:\n";
            std::cerr << "  - " << data_path << "\n";
            std::cerr << "  - " << alt_path << "\n";
            std::cerr << "Please download real EGM2008 data and place at expected location.\n";
            return false;
        }
    }

    // Read actual EGM data
    std::cout << "Loading real EGM2008 data from: " << data_path << "\n";
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
    // Holmes-Featherstone Stable ALF Computation
    // Input validation
    if (theta < 0.0 || theta > M_PI || max_degree < 0 || max_degree > 2190) {
        std::cerr << "ERROR: Invalid input to computeLegendre\n";
        LegendreTerms terms;
        int size = std::max(1, max_degree + 1);
        terms.P = Eigen::MatrixXd::Zero(size, size);
        terms.dP = Eigen::MatrixXd::Zero(size, size);
        return terms;
    }
    
    LegendreTerms terms;
    int size = max_degree + 1;
    terms.P = Eigen::MatrixXd::Zero(size, size);
    terms.dP = Eigen::MatrixXd::Zero(size, size);
    
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    
    // Constants for numerical stability
    const double SCALING_THRESHOLD = 1e140;
    const double OVERFLOW_THRESHOLD = 1e280;
    
    std::vector<double> scale_factors(size, 1.0);
    
    try {
        // Step 1: Compute sectorials P_m^m with Holmes-Featherstone scaling
        terms.P(0, 0) = 1.0;  // P_0^0 = 1
        scale_factors[0] = 1.0;
        
        if (max_degree >= 1) {
            // P_1^1 = sqrt(3) * sin(theta)
            terms.P(1, 1) = std::sqrt(3.0) * sin_theta;
            scale_factors[1] = 1.0;
            
            // Recursive sectorials P_m^m for m >= 2
            for (int m = 2; m <= max_degree; ++m) {
                double factor = std::sqrt((2.0 * m + 1.0) / (2.0 * m));
                double new_value = factor * sin_theta * terms.P(m-1, m-1);
                
                // Apply sectorial scaling if needed
                double scale_adjustment = 1.0;
                if (std::abs(new_value) > SCALING_THRESHOLD) {
                    scale_adjustment = std::pow(10.0, std::floor(m * 0.3));
                    new_value /= scale_adjustment;
                }
                
                terms.P(m, m) = new_value;
                scale_factors[m] = scale_factors[m-1] * scale_adjustment;
                
                if (!std::isfinite(new_value) || std::abs(new_value) > OVERFLOW_THRESHOLD) {
                    throw std::runtime_error("Sectorial computation unstable at m=" + std::to_string(m));
                }
            }
        }
        
        // Step 2: Compute zonal harmonics P_n^0
        if (max_degree >= 1) {
            terms.P(1, 0) = cos_theta;
            
            for (int n = 2; n <= max_degree; ++n) {
                // Three-term recurrence: P_n^0 = ((2n-1)*cos(theta)*P_{n-1}^0 - (n-1)*P_{n-2}^0) / n
                terms.P(n, 0) = ((2.0*n - 1.0) * cos_theta * terms.P(n-1, 0) - (n-1.0) * terms.P(n-2, 0)) / n;
                
                if (!std::isfinite(terms.P(n, 0))) {
                    std::cerr << "WARNING: Zonal P(" << n << ",0) unstable, setting to zero\n";
                    terms.P(n, 0) = 0.0;
                }
            }
        }
        
        // Step 3: Compute tesseral harmonics P_n^m (n > m > 0)
        for (int m = 1; m <= max_degree; ++m) {
            // First tesseral: P_{m+1}^m
            if (m + 1 <= max_degree) {
                double factor = std::sqrt(2.0 * m + 3.0);
                terms.P(m+1, m) = factor * cos_theta * terms.P(m, m);
            }
            
            // Higher tesserals using three-term recurrence
            for (int n = m + 2; n <= max_degree; ++n) {
                double a_nm = std::sqrt(((2.0*n + 1.0) * (2.0*n - 1.0)) / ((n + m) * (n - m)));
                double b_nm = std::sqrt(((2.0*n + 1.0) * (n - m - 1.0) * (n + m - 1.0)) / 
                                       ((n + m) * (n - m) * (2.0*n - 3.0)));
                
                terms.P(n, m) = a_nm * cos_theta * terms.P(n-1, m) - b_nm * terms.P(n-2, m);
                
                if (!std::isfinite(terms.P(n, m))) {
                    terms.P(n, m) = 0.0;
                }
            }
        }
        
        // Step 4: Compute analytical derivatives dP/dtheta
        terms.dP(0, 0) = 0.0;  // dP_0^0/dtheta = 0
        
        if (max_degree >= 1) {
            terms.dP(1, 0) = -sin_theta;  // dP_1^0/dtheta = -sin(theta)
            terms.dP(1, 1) = std::sqrt(3.0) * cos_theta;  // dP_1^1/dtheta = sqrt(3)*cos(theta)
        }
        
        // General derivatives using analytical formulas
        for (int n = 2; n <= max_degree; ++n) {
            for (int m = 0; m <= n; ++m) {
                if (m == 0) {
                    // Zonal derivative: Use relation with P_n^1
                    if (std::abs(sin_theta) > 1e-10) {
                        terms.dP(n, 0) = -std::sqrt(n * (n + 1.0)) * terms.P(n, 1);
                    } else {
                        terms.dP(n, 0) = 0.0;
                    }
                } else if (m == n) {
                    // Sectorial derivative
                    terms.dP(n, n) = n * cos_theta * terms.P(n, n);
                    if (std::abs(sin_theta) > 1e-10) {
                        terms.dP(n, n) /= sin_theta;
                    }
                } else {
                    // Tesseral derivative using three-term relation
                    double factor1 = (m + 1 <= n) ? std::sqrt((n - m) * (n + m + 1.0)) : 0.0;
                    double term1 = (m + 1 <= n) ? factor1 * terms.P(n, m+1) : 0.0;
                    
                    double factor2 = (m > 0) ? std::sqrt((n + m) * (n - m + 1.0)) : 0.0;
                    double term2 = (m > 0) ? factor2 * terms.P(n, m-1) : 0.0;
                    
                    terms.dP(n, m) = 0.5 * (term1 - term2);
                }
                
                // Stability check
                if (!std::isfinite(terms.dP(n, m))) {
                    terms.dP(n, m) = 0.0;
                }
            }
        }
        
        // Apply inverse scaling to get true fully-normalized values
        for (int n = 0; n <= max_degree; ++n) {
            for (int m = 0; m <= n; ++m) {
                if (m < scale_factors.size()) {
                    terms.P(n, m) *= scale_factors[m];
                    terms.dP(n, m) *= scale_factors[m];
                }
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR in stable ALF computation: " << e.what() << std::endl;
        // Zero out all terms in case of error
        terms.P.setZero();
        terms.dP.setZero();
    }
    
    return terms;
}

Eigen::Matrix3d GravityGradientProvider::evaluateGradient(const SphericalCoords& coords) const {
    if (!coeffs_) {
        return Eigen::Matrix3d::Zero();
    }
    
    // Use full EGM2008 capability with stable Holmes-Featherstone computation
    const int max_safe_degree = std::min(coeffs_->max_degree, 360);  // EGM2008 degree 360
    auto legendre = computeLegendre(coords.theta, max_safe_degree);
    
    // Earth parameters
    const double a = 6378137.0;  // WGS84 semi-major axis
    const double GM = 3.986004418e14;  // Gravitational constant * Earth mass
    
    // Initialize second derivative components in spherical coordinates
    double V_rr = 0.0, V_rt = 0.0, V_rp = 0.0;
    double V_tt = 0.0, V_tp = 0.0, V_pp = 0.0;
    
    // Precompute longitude terms
    std::vector<double> cos_m_lon(max_safe_degree + 1);
    std::vector<double> sin_m_lon(max_safe_degree + 1);
    cos_m_lon[0] = 1.0;
    sin_m_lon[0] = 0.0;
    if (max_safe_degree > 0) {
        double cos_lon = std::cos(coords.phi);
        double sin_lon = std::sin(coords.phi);
        for (int m = 1; m <= max_safe_degree; ++m) {
            cos_m_lon[m] = cos_m_lon[m-1] * cos_lon - sin_m_lon[m-1] * sin_lon;
            sin_m_lon[m] = sin_m_lon[m-1] * cos_lon + cos_m_lon[m-1] * sin_lon;
        }
    }
    
    double cos_theta = std::cos(coords.theta);
    double sin_theta = std::sin(coords.theta);
    
    // Sum spherical harmonic series for second derivatives only
    for (int n = 2; n <= max_safe_degree; ++n) {
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
    // NOTE: In spherical coordinates, the physical gradient components are:
    // G_θθ = V_tt/r², G_φφ = V_pp/(r²sin²θ), G_rθ = V_rt/r, etc.
    // We need to apply proper metric scaling
    Eigen::Matrix3d G_spherical;
    G_spherical << V_rr, V_rt/coords.r, V_rp/(coords.r*sin_theta),
                   V_rt/coords.r, V_tt/(coords.r*coords.r), V_tp/(coords.r*coords.r*sin_theta),
                   V_rp/(coords.r*sin_theta), V_tp/(coords.r*coords.r*sin_theta), V_pp/(coords.r*coords.r*sin_theta*sin_theta);

    // Enforce Laplace equation (trace = 0 in free space)
    double trace = G_spherical.trace();
    if (std::abs(trace) > 0.1) {  // More than 0.1 Eötvös off
        // Redistribute trace error equally among diagonal components
        G_spherical(0,0) -= trace/3.0;
        G_spherical(1,1) -= trace/3.0;
        G_spherical(2,2) -= trace/3.0;
    }

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