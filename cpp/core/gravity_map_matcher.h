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
            signature_length(100),          // Increased for better correlation
            correlation_threshold(0.60),    // Lowered for initial matches
            search_radius_m(15000),         // Expanded for large errors
            grid_resolution_m(250),         // Coarser for speed
            min_measurements(50) {}         // More measurements required
    };
    
    struct GravityMeasurement {
        double timestamp;
        Eigen::Vector3d position_ECEF;    // INS-estimated position
        double anomaly_mgal;              // Measured gravity anomaly
        Eigen::Matrix3d gradient_tensor;  // Full 3x3 gradient tensor (9 components)

        // Helper to get flattened 9D vector for correlation
        Eigen::VectorXd getFlattenedSignature() const {
            Eigen::VectorXd sig(10);  // 1 anomaly + 9 tensor components
            sig(0) = anomaly_mgal;
            int idx = 1;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    sig(idx++) = gradient_tensor(i, j);
                }
            }
            return sig;
        }
    };
    
    struct MatchCandidate {
        Eigen::Vector3d position_ECEF;
        double gravity_correlation;        // Gravity signature correlation
        double terrain_correlation;        // Terrain profile correlation (if available)
        double combined_score;             // Combined matching score
        std::vector<Eigen::Vector3d> path; // The shifted path for this candidate
    };

    struct MatchResult {
        bool valid;
        Eigen::Vector3d matched_position_ECEF;
        double confidence;                // 0-1, higher is better
        double position_uncertainty_m;    // Estimated accuracy
        std::vector<Eigen::Vector3d> search_path; // For debugging
        std::vector<MatchCandidate> candidates;   // All high-correlation candidates for terrain validation
    };
    
    GravityMapMatcher(const Config& cfg = Config());
    
    /**
     * Add a new gravity measurement to the signature buffer
     */
    void addMeasurement(const GravityMeasurement& meas);
    
    /**
     * Attempt to match the current signature against the gravity map
     * Returns multiple candidate positions for terrain validation
     */
    MatchResult findMatch(const GravityGradientProvider& gravity_model);

    /**
     * Find multiple candidate matches above a threshold
     * Used for two-factor authentication with terrain
     */
    std::vector<MatchCandidate> findCandidates(
        const GravityGradientProvider& gravity_model,
        double min_correlation = 0.9,
        int max_candidates = 10);
    
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

    /**
     * Extract multi-dimensional gravity signature from map
     */
    std::vector<Eigen::VectorXd> extractMultiDimMapSignature(
        const GravityGradientProvider& gravity_model,
        const std::vector<Eigen::Vector3d>& path) const;

    /**
     * Normalize multi-dimensional signature
     */
    std::vector<Eigen::VectorXd> normalizeMultiDimSignature(
        const std::vector<Eigen::VectorXd>& sig) const;

    /**
     * Compute correlation for multi-dimensional signatures
     */
    double computeMultiDimCorrelation(
        const std::vector<Eigen::VectorXd>& measured,
        const std::vector<Eigen::VectorXd>& reference) const;
};