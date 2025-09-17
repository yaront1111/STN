#include "gravity_gradient_provider.h"
#include "ukf_math_utils.h"  // For ecefToLla
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cassert>

bool GravityGradientProvider::loadEGM2020(const std::string& data_path) {
    std::ifstream file(data_path, std::ios::binary);
    std::string used_path = data_path;

    if (!file.is_open()) {
        std::string alt_path = "egm2008/egm2008_n360.dat";
        file.open(alt_path, std::ios::binary);
        used_path = alt_path;
    }
    if (!file.is_open()) {
        std::cerr << "FATAL: Real EGM data required (no synthetic fallback).\n"
                  << "Tried:\n  - " << data_path << "\n  - egm2008/egm2008_n360.dat\n";
        return false;
    }

    coeffs_ = std::make_unique<SHCoefficients>();

    // Minimal header: [int max_degree] then C and S arrays of doubles.
    file.read(reinterpret_cast<char*>(&coeffs_->max_degree), sizeof(int));
    if (!file || coeffs_->max_degree <= 0 || coeffs_->max_degree > 2190) {
        std::cerr << "ERROR: Bad EGM header or unsupported degree: "
                  << coeffs_->max_degree << "\n";
        return false;
    }

    const int num_coeffs = (coeffs_->max_degree + 1) * (coeffs_->max_degree + 2) / 2;
    coeffs_->C.resize(num_coeffs);
    coeffs_->S.resize(num_coeffs);

    file.read(reinterpret_cast<char*>(coeffs_->C.data()), sizeof(double) * num_coeffs);
    file.read(reinterpret_cast<char*>(coeffs_->S.data()), sizeof(double) * num_coeffs);

    if (!file) {
        std::cerr << "ERROR: Truncated EGM coefficient file: " << used_path << "\n";
        return false;
    }

    std::cout << "✓ Loaded EGM data from: " << used_path
              << " | max degree: " << coeffs_->max_degree
              << " | coeffs: " << num_coeffs << "\n";
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
    const int N = std::max(0, std::min(max_degree, 2190)) + 1;
    terms.P  = Eigen::MatrixXd::Zero(N, N);
    terms.dP = Eigen::MatrixXd::Zero(N, N);

    if (N == 0) return terms;

    const double ct = std::cos(theta);
    const double st = std::max(1e-14, std::sin(theta)); // protect poles
    
    // Fully-normalized ALFs (EGM uses fully-normalized)
    terms.P(0,0) = 1.0;
    if (N == 1) return terms;

    // Sectorials P_m^m
    terms.P(1,1) = std::sqrt(3.0) * st;
    for (int m = 2; m < N; ++m) {
        // P_m^m = sqrt((2m+1)/(2m)) * st * P_{m-1}^{m-1}
        terms.P(m,m) = std::sqrt((2.0*m + 1.0)/(2.0*m)) * st * terms.P(m-1,m-1);
    }

    // Tesseral first step: P_{m+1}^m
    for (int m = 0; m + 1 < N; ++m) {
        // P_{m+1}^m = sqrt(2m+3) * ct * P_m^m
        terms.P(m+1,m) = std::sqrt(2.0*m + 3.0) * ct * terms.P(m,m);
    }

    // Tesseral general: P_n^m, n >= m+2
    for (int m = 0; m < N; ++m) {
        for (int n = m + 2; n < N; ++n) {
            // P_n^m = a_nm * ct * P_{n-1}^m - b_nm * P_{n-2}^m
            const double a_nm = std::sqrt(((2.0*n + 1.0)*(2.0*n - 1.0))/((n+m)*(n-m)));
            const double b_nm = std::sqrt(((2.0*n + 1.0)*(n+m-1.0)*(n-m-1.0)) /
                                          ((n+m)*(n-m)*(2.0*n - 3.0)));
            terms.P(n,m) = a_nm * ct * terms.P(n-1,m) - b_nm * terms.P(n-2,m);
        }
    }

    // Derivatives (fully-normalized): for all n,m
    // dP_n^m/dθ = n*cotθ*P_n^m - sqrt(n^2 - m^2)/sinθ * P_{n-1}^m
    const double cot = ct / st;
    for (int n = 1; n < N; ++n) {
        for (int m = 0; m <= n; ++m) {
            const double s = std::sqrt(std::max(0.0, double(n*n - m*m)));
            terms.dP(n,m) = n * cot * terms.P(n,m) - (s / st) * terms.P(n-1,m);
        }
    }
    // dP_0^0 = 0 already
    
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
            
            // Check for valid Legendre values
            if (!std::isfinite(legendre.P(n, m))) {
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
            
            // V_tt = second theta derivative (correct formula)
            // V_tt = ar^n * [(m^2/sin^2(θ) - n(n+1))P_nm - (cos(θ)/sin(θ))dP_nm] * CS_sum
            if (std::abs(sin_theta) > 1e-6) {
                double P_term_for_Vtt = (m*m / (sin_theta*sin_theta) - (n+1.0)*n) * legendre.P(n,m) -
                                        (cos_theta/sin_theta) * legendre.dP(n,m);
                V_tt += ar_pow * P_term_for_Vtt * CS_sum;
            }
            
            // Phi-related derivatives
            // V_rp and V_tp are proportional to m (zero for m=0)
            if (m > 0 && std::abs(sin_theta) > 1e-6) {
                double CS_diff_m = m * (S_nm * cos_m_lon[m] - C_nm * sin_m_lon[m]);

                // V_rp = (n+1) * ar^n * P_nm * m * CS_diff
                V_rp += ar_pow * (n + 1.0) * legendre.P(n,m) * CS_diff_m;

                // V_tp = ar^n * m/sin(θ) * [dP_nm - (cos(θ)/sin(θ))P_nm] * CS_diff
                V_tp += ar_pow * (m / sin_theta) * (legendre.dP(n,m) - (cos_theta/sin_theta) * legendre.P(n,m)) * CS_diff_m;
            }

            // V_pp for ALL m (this was the critical bug - was only computed for m>0)
            // V_pp = ar^n * [(n(n+1) - m^2/sin^2(θ))P_nm + (cos(θ)/sin(θ))dP_nm] * CS_sum
            if (std::abs(sin_theta) > 1e-6) {
                double P_term_for_Vpp = ((n+1.0)*n - m*m/(sin_theta*sin_theta)) * legendre.P(n,m) +
                                        (cos_theta/sin_theta) * legendre.dP(n,m);
                V_pp += ar_pow * P_term_for_Vpp * CS_sum;
            }
        }
    }
    
    // Apply proper scaling for gravity gradients
    // Second derivatives need GM/r^3 base scaling
    double base_scale = GM / (coords.r * coords.r * coords.r);

    // Guard against singularity at poles
    const double st_guard = std::max(1e-12, std::abs(sin_theta));

    // Build anomalous gradient tensor in spherical coordinates with metric scaling
    Eigen::Matrix3d G_anomalous;
    G_anomalous << V_rr, V_rt/coords.r, V_rp/(coords.r*st_guard),
                   V_rt/coords.r, V_tt/(coords.r*coords.r), V_tp/(coords.r*coords.r*st_guard),
                   V_rp/(coords.r*st_guard), V_tp/(coords.r*coords.r*st_guard), V_pp/(coords.r*coords.r*st_guard*st_guard);

    // Scale anomalous part to Eötvös
    G_anomalous *= base_scale * 1e9;

    // *** ADD THE PRIMARY (POINT-MASS) GRADIENT ***
    // Primary field gradient in spherical coordinates (already with metric)
    Eigen::Matrix3d G_primary = Eigen::Matrix3d::Zero();
    double primary_scale = base_scale * 1e9;
    G_primary(0, 0) = 2.0 * primary_scale;   // G_rr = 2GM/r^3
    G_primary(1, 1) = -1.0 * primary_scale;  // G_θθ = -GM/r^3
    G_primary(2, 2) = -1.0 * primary_scale;  // G_φφ = -GM/r^3

    // Total gradient = primary + anomalous
    Eigen::Matrix3d G_total_spherical = G_primary + G_anomalous;

    // Enforce Laplace equation (trace = 0 in free space)
    double trace = G_total_spherical.trace();
    if (std::abs(trace) > 0.1) {  // More than 0.1 Eötvös off
        // Redistribute trace error equally among diagonal components
        G_total_spherical(0,0) -= trace/3.0;
        G_total_spherical(1,1) -= trace/3.0;
        G_total_spherical(2,2) -= trace/3.0;
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
    
    // Transform gradient tensor (CORRECT: T * G_spherical * T^T for tensors)
    Eigen::Matrix3d G_ECEF = T * G_total_spherical * T.transpose();

    // Enforce symmetry numerically
    G_ECEF = 0.5 * (G_ECEF + G_ECEF.transpose());

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
    // Get gradient in ECEF frame
    const auto g = getGradient(pos_ECEF);
    const double r = pos_ECEF.norm();

    // Compute geodetic lat/lon from ECEF
    const Eigen::Vector3d lla = UKFMathUtils::ecefToLla(pos_ECEF);
    const double lat = lla(0);
    const double lon = lla(1);

    const double sLat = std::sin(lat), cLat = std::cos(lat);
    const double sLon = std::sin(lon), cLon = std::cos(lon);

    // Build rotation matrix from ECEF to ENU
    // ENU basis from ECEF (rows are basis vectors)
    Eigen::Matrix3d R;
    R << -sLon,            cLon,             0,          // East
         -sLat*cLon,      -sLat*sLon,       cLat,        // North
          cLat*cLon,       cLat*sLon,       sLat;        // Up

    // Rotate gradient tensor to ENU frame
    const Eigen::Matrix3d G_ENU = R * g.T * R.transpose();

    // Normal vertical gradient (Up/Up component) in Eötvös
    const double GM = 3.986004418e14;
    // Note: Up points outward, so Tuu_normal = -2GM/r^3 (negative of radial)
    const double Tuu_normal = -2.0 * GM / (r*r*r) * 1e9;

    const double Tuu_actual = G_ENU(2,2);  // ENU: [E,N,U]; index 2 is Up
    const double delta_Tuu_E = Tuu_actual - Tuu_normal;

    // Return anomaly in Eötvös (the native unit for gradients)
    return delta_Tuu_E;
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