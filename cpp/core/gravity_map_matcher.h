#pragma once
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include "types.h"
#include "gravity_gradient_provider.h"

/**
 * GRAVITY ANOMALY MAP MATCHING
 * 
 * Similar to TERCOM (Terrain Contour Matching) but uses gravity anomalies
 * Records a sequence of gravity measurements along the flight path
 * Correlates this "gravity signature" with the EGM2008 map
 * Provides absolute position fixes when high-confidence matches are found
 */
class GravityMapMatcher {
public:
    struct Config {
        int signature_length;        // Number of measurements in signature
        double correlation_threshold; // Minimum correlation for valid match
        double search_radius_m;    // Search radius around estimated position
        double grid_resolution_m;   // Grid spacing for correlation search
        int min_measurements;        // Minimum measurements before attempting match
        
        Config() : 
            signature_length(60),
            correlation_threshold(0.95),
            search_radius_m(5000),
            grid_resolution_m(100),
            min_measurements(30) {}
    };
    
    struct GravityMeasurement {
        double timestamp;
        Eigen::Vector3d position_ECEF;    // INS-estimated position
        double anomaly_mgal;              // Measured gravity anomaly
        double gradient_trace;            // Trace of gradient tensor
    };
    
    struct MatchResult {
        bool valid;
        Eigen::Vector3d matched_position_ECEF;
        double confidence;                // 0-1, higher is better
        double position_uncertainty_m;    // Estimated accuracy
        std::vector<Eigen::Vector3d> search_path; // For debugging
    };
    
    GravityMapMatcher(const Config& cfg = Config());
    
    /**
     * Add a new gravity measurement to the signature buffer
     */
    void addMeasurement(const GravityMeasurement& meas);
    
    /**
     * Attempt to match the current signature against the gravity map
     * Returns a position fix if successful
     */
    MatchResult findMatch(const GravityGradientProvider& gravity_model);
    
    /**
     * Clear the measurement buffer (e.g., after successful match)
     */
    void reset();
    
    /**
     * Get current signature length
     */
    size_t getSignatureLength() const { return signature_buffer_.size(); }
    
private:
    Config cfg_;
    std::deque<GravityMeasurement> signature_buffer_;
    
    /**
     * Compute correlation between two gravity sequences
     * Returns correlation coefficient (-1 to 1)
     */
    double computeCorrelation(const std::vector<double>& measured,
                             const std::vector<double>& reference) const;
    
    /**
     * Extract gravity signature from map along a path
     */
    std::vector<double> extractMapSignature(
        const GravityGradientProvider& gravity_model,
        const std::vector<Eigen::Vector3d>& path) const;
    
    /**
     * Generate search grid around estimated position
     */
    std::vector<Eigen::Vector3d> generateSearchGrid(
        const Eigen::Vector3d& center_ECEF) const;
    
    /**
     * Compute Sum of Squared Differences (SSD) for fast matching
     */
    double computeSSD(const std::vector<double>& measured,
                     const std::vector<double>& reference) const;
    
    /**
     * Normalize gravity signature for correlation
     */
    std::vector<double> normalizeSignature(const std::vector<double>& sig) const;
};