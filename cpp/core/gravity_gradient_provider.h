#pragma once
#include "types.h"
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>

/**
 * EGM2020 Gravity Gradient Provider
 * The core of gravity-primary navigation
 * 
 * Replaces terrain_provider.h completely
 */
class GravityGradientProvider {
public:
    /**
     * Initialize with EGM2020 data file
     * File format: spherical harmonic coefficients to degree 2190
     */
    bool loadEGM2020(const std::string& data_path);
    
    /**
     * Get gravity gradient tensor at position
     * @param pos_ECEF Position in ECEF coordinates (meters)
     * @return Full 3x3 gravity gradient tensor (Eötvös units)
     */
    GravityGradientTensor getGradient(const Eigen::Vector3d& pos_ECEF) const;
    
    /**
     * Get gravity anomaly at position
     * @param pos_ECEF Position in ECEF coordinates (meters)
     * @return Gravity anomaly in mGal
     */
    double getAnomaly(const Eigen::Vector3d& pos_ECEF) const;
    
    /**
     * Add temporal variations
     */
    void addEarthTides(GravityGradientTensor& gradient, double t) const;
    void addAtmosphericPressure(GravityGradientTensor& gradient, double pressure_hPa) const;
    void addOceanTides(GravityGradientTensor& gradient, double t) const;
    
    /**
     * Apply relativistic corrections
     */
    void applyRelativisticCorrections(GravityGradientTensor& gradient, 
                                     const Eigen::Vector3d& velocity) const;
    
    /**
     * Get gradient with all corrections applied
     */
    GravityGradientTensor getFullGradient(const Eigen::Vector3d& pos_ECEF,
                                          const Eigen::Vector3d& vel_ECEF,
                                          double t,
                                          double pressure_hPa = 1013.25) const;
    
private:
    // Spherical harmonic coefficients
    struct SHCoefficients {
        int max_degree = 2190;  // EGM2020 goes to degree 2190
        std::vector<double> C;   // Cosine coefficients
        std::vector<double> S;   // Sine coefficients
        
        // Get coefficient index
        int idx(int n, int m) const {
            return n * (n + 1) / 2 + m;
        }
    };
    
    std::unique_ptr<SHCoefficients> coeffs_;
    
    /**
     * Convert ECEF to spherical coordinates
     */
    struct SphericalCoords {
        double r;      // Radius (meters)
        double theta;  // Colatitude (radians)
        double phi;    // Longitude (radians)
    };
    
    SphericalCoords toSpherical(const Eigen::Vector3d& pos_ECEF) const;
    
    /**
     * Compute Legendre functions and derivatives
     */
    struct LegendreTerms {
        Eigen::MatrixXd P;     // Associated Legendre functions
        Eigen::MatrixXd dP;    // First derivatives
        Eigen::MatrixXd ddP;   // Second derivatives
    };
    
    LegendreTerms computeLegendre(double theta, int max_degree) const;
    
    /**
     * Evaluate spherical harmonic expansion for gradient
     */
    Eigen::Matrix3d evaluateGradient(const SphericalCoords& coords) const;
    
    /**
     * Earth tide model (IERS conventions)
     */
    struct TideModel {
        // Love numbers
        double h2 = 0.6078;
        double l2 = 0.0847;
        double k2 = 0.3025;
        
        Eigen::Matrix3d computeSolidEarthTide(const Eigen::Vector3d& pos_ECEF, 
                                              double t) const;
    };
    
    TideModel tide_model_;
    
    /**
     * Cache for performance
     */
    mutable struct Cache {
        Eigen::Vector3d last_pos;
        GravityGradientTensor last_gradient;
        bool valid = false;
    } cache_;
};